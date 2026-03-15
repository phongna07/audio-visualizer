#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <math.h>

// Connections to the INMP441
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

// Use I2S processor 0
#define I2S_PORT I2S_NUM_0

// OLED configuration (SSD1306 128x64 over I2C)
constexpr uint8_t OLED_ADDR = 0x3C;
constexpr uint8_t OLED_SDA = 21;
constexpr uint8_t OLED_SCL = 22;
constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;

// Visualizer DSP configuration
constexpr uint16_t SAMPLE_RATE = 44100;
constexpr uint16_t FFT_SAMPLES = 256;
constexpr uint8_t NUM_BARS = 24;
constexpr uint8_t TOP_MARGIN = 10;
constexpr uint8_t VISUAL_X_OFFSET = 3;
constexpr float DISPLAY_FLOOR = 0.006f;
constexpr float DISPLAY_GAIN = 66.0f;
constexpr float BAR_DISTRIBUTION_EXP = 1.08f;
constexpr float HIGH_BAND_BOOST_BASE = 0.92f;
constexpr float HIGH_BAND_BOOST_RANGE = 0.95f;
constexpr float BAND_EQ_ALPHA = 0.022f;
constexpr float GLOBAL_EQ_ALPHA = 0.04f;
constexpr float MIN_EQ_MULT = 0.55f;
constexpr float MAX_EQ_MULT = 2.20f;

struct AudioFrame {
  float samples[FFT_SAMPLES];
  uint32_t sequence;
};

QueueHandle_t g_audioFrameQueue = nullptr;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C g_display(U8G2_R0, U8X8_PIN_NONE);

void drawBootScreen() {
  g_display.clearBuffer();
  g_display.setCursor(12, 16);
  g_display.println("AI Companion");
  g_display.setCursor(8, 30);
  g_display.println("Audio Visualizer");
  g_display.drawLine(0, 46, SCREEN_WIDTH - 1, 46);
  g_display.sendBuffer();
}

bool setup_display() {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  g_display.setI2CAddress(OLED_ADDR << 1);
  g_display.begin();
  g_display.setBusClock(400000);
  g_display.setDrawColor(1);
  g_display.setFont(u8g2_font_5x7_tr);
  g_display.setFontPosTop();
  drawBootScreen();
  return true;
}

void setup_i2s() {
  const i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample =
          I2S_BITS_PER_SAMPLE_32BIT, // INMP441 sends 24-bit in 32-bit slots
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 64,
      .use_apll = false};

  const i2s_pin_config_t pin_config = {.bck_io_num = I2S_SCK,
                                       .ws_io_num = I2S_WS,
                                       .data_out_num = -1, // Not used
                                       .data_in_num = I2S_SD};

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void taskEar(void *parameter) {
  (void)parameter;

  AudioFrame frame = {};
  frame.sequence = 0;
  uint16_t frameIndex = 0;

  constexpr size_t READ_BLOCK = 64;
  int32_t rawBlock[READ_BLOCK];

  while (true) {
    size_t bytesRead = 0;
    i2s_read(I2S_PORT, rawBlock, sizeof(rawBlock), &bytesRead, portMAX_DELAY);

    size_t samplesRead = bytesRead / sizeof(int32_t);
    for (size_t i = 0; i < samplesRead; ++i) {
      int32_t aligned = rawBlock[i] >> 8;
      frame.samples[frameIndex++] = (float)aligned / 8388608.0f;

      if (frameIndex >= FFT_SAMPLES) {
        frame.sequence++;
        xQueueOverwrite(g_audioFrameQueue, &frame);
        frameIndex = 0;
      }
    }
  }
}

void taskEye(void *parameter) {
  (void)parameter;

  AudioFrame frame = {};
  double vReal[FFT_SAMPLES];
  double vImag[FFT_SAMPLES];
  ArduinoFFT<double> FFT(vReal, vImag, FFT_SAMPLES, SAMPLE_RATE);

  float smoothedBars[NUM_BARS] = {0.0f};
  float peakCaps[NUM_BARS] = {0.0f};
  float bandHistory[NUM_BARS];
  for (uint8_t i = 0; i < NUM_BARS; ++i) {
    bandHistory[i] = 0.02f;
  }
  float globalHistory = 0.02f;
  float shimmerOffset = 0.0f;

  uint32_t lastFrameMicros = micros();
  uint16_t fps = 0;

  TickType_t nextWake = xTaskGetTickCount();

  while (true) {
    if (xQueueReceive(g_audioFrameQueue, &frame, pdMS_TO_TICKS(100)) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    float mean = 0.0f;
    for (uint16_t i = 0; i < FFT_SAMPLES; ++i) {
      mean += frame.samples[i];
    }
    mean /= FFT_SAMPLES;

    for (uint16_t i = 0; i < FFT_SAMPLES; ++i) {
      vReal[i] = frame.samples[i] - mean;
      vImag[i] = 0.0;
    }

    FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    float spectrumEnergy = 0.0f;

    for (uint8_t bar = 0; bar < NUM_BARS; ++bar) {
      float ratioStart = (float)bar / NUM_BARS;
      float ratioEnd = (float)(bar + 1) / NUM_BARS;

      int startBin = 1 + (int)(powf(ratioStart, BAR_DISTRIBUTION_EXP) * ((FFT_SAMPLES / 2) - 2));
      int endBin = 1 + (int)(powf(ratioEnd, BAR_DISTRIBUTION_EXP) * ((FFT_SAMPLES / 2) - 2));

      if (endBin <= startBin) {
        endBin = startBin + 1;
      }

      float sum = 0.0f;
      float sumSquares = 0.0f;
      float maxMag = 0.0f;
      int count = 0;
      for (int bin = startBin; bin <= endBin; ++bin) {
        float value = (float)vReal[bin];
        sum += value;
        sumSquares += (value * value);
        if (value > maxMag) {
          maxMag = value;
        }
        count++;
      }

      float avgMag = (count > 0) ? (sum / count) : 0.0f;
      float rmsMag = (count > 0) ? sqrtf(sumSquares / count) : 0.0f;
      float mag = (rmsMag * 0.65f) + (maxMag * 0.35f);

      // Keep low bands full while preserving right-side responsiveness.
      mag = (mag * 0.85f) + (avgMag * 0.15f);

      float barPos = (float)bar / (NUM_BARS - 1);
      float bandBoost = HIGH_BAND_BOOST_BASE + (barPos * HIGH_BAND_BOOST_RANGE);
      mag *= bandBoost;

      bandHistory[bar] = (bandHistory[bar] * (1.0f - BAND_EQ_ALPHA)) + (mag * BAND_EQ_ALPHA);
      float eq = sqrtf((globalHistory + 0.0001f) / (bandHistory[bar] + 0.0001f));
      eq = fminf(MAX_EQ_MULT, fmaxf(MIN_EQ_MULT, eq));
      mag *= eq;

      spectrumEnergy += mag;

      float normalized = (mag - DISPLAY_FLOOR) * DISPLAY_GAIN;
      if (normalized < 0.0f) {
        normalized = 0.0f;
      }

      float compressed = log10f(1.0f + normalized) / log10f(1.0f + DISPLAY_GAIN);
      float targetHeight = compressed * (SCREEN_HEIGHT - TOP_MARGIN - 2);

      if (targetHeight > smoothedBars[bar]) {
        smoothedBars[bar] = (smoothedBars[bar] * 0.45f) + (targetHeight * 0.55f);
      } else {
        smoothedBars[bar] = (smoothedBars[bar] * 0.82f) + (targetHeight * 0.18f);
      }

      if (smoothedBars[bar] > peakCaps[bar]) {
        peakCaps[bar] = smoothedBars[bar];
      } else {
        peakCaps[bar] = fmaxf(0.0f, peakCaps[bar] - 0.75f);
      }
    }

    float frameMeanEnergy = spectrumEnergy / NUM_BARS;
    globalHistory = (globalHistory * (1.0f - GLOBAL_EQ_ALPHA)) + (frameMeanEnergy * GLOBAL_EQ_ALPHA);

    uint32_t now = micros();
    uint32_t frameMicros = now - lastFrameMicros;
    lastFrameMicros = now;
    if (frameMicros > 0) {
      fps = (uint16_t)(1000000UL / frameMicros);
    }

    shimmerOffset += 0.22f;
    if (shimmerOffset > 6.283185f) {
      shimmerOffset = 0.0f;
    }

    g_display.clearBuffer();

    int baseline = SCREEN_HEIGHT - 1;
    int drawableWidth = SCREEN_WIDTH - VISUAL_X_OFFSET;
    int barW = drawableWidth / NUM_BARS;

    for (uint8_t bar = 0; bar < NUM_BARS; ++bar) {
      int x = VISUAL_X_OFFSET + (bar * barW);
      int h = (int)smoothedBars[bar];
      if (h < 1) {
        h = 1;
      }

      g_display.drawBox(x, baseline - h, barW - 1, h);

      int capY = baseline - (int)peakCaps[bar];
      if (capY < TOP_MARGIN) {
        capY = TOP_MARGIN;
      }
      g_display.drawLine(x, capY, x + barW - 2, capY);

      int ghostHeight = (int)(h * 0.35f);
      g_display.setDrawColor(0);
      for (int gy = 0; gy < ghostHeight; gy += 3) {
        int px = x + ((bar & 1U) ? 1 : 0);
        int py = baseline - h + gy;
        if (py >= TOP_MARGIN && py < SCREEN_HEIGHT) {
          g_display.drawPixel(px, py);
        }
      }
      g_display.setDrawColor(1);
    }


    g_display.setCursor(VISUAL_X_OFFSET, 0);
    g_display.print("FFT ");
    g_display.print(FFT_SAMPLES);
    g_display.print("  ");
    g_display.print(fps);
    g_display.print("fps");

    g_display.sendBuffer();

    vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(25));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  if (!setup_display()) {
    Serial.println("SSD1306 init failed");
    while (true) {
      delay(1000);
    }
  }

  setup_i2s();

  g_audioFrameQueue = xQueueCreate(1, sizeof(AudioFrame));
  if (g_audioFrameQueue == nullptr) {
    Serial.println("Queue allocation failed");
    while (true) {
      delay(1000);
    }
  }

  BaseType_t earCreated = xTaskCreatePinnedToCore(taskEar, "EarTask", 4096, nullptr,
                                                   configMAX_PRIORITIES - 2, nullptr, 0);
  BaseType_t eyeCreated = xTaskCreatePinnedToCore(taskEye, "EyeTask", 8192, nullptr,
                                                   2, nullptr, 1);

  if (earCreated != pdPASS || eyeCreated != pdPASS) {
    Serial.println("Task creation failed");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Visualizer online: Core0 Ear, Core1 Eye");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}