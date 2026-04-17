#include <Arduino.h>

// ---------------------------------------------------------------------------
// Protocol
// ---------------------------------------------------------------------------
constexpr uint8_t CMD_START = 67;  // 'C' → begin streaming
constexpr uint8_t CMD_STOP  = 69;  // 'E' → stop streaming

// ---------------------------------------------------------------------------
// ADC configuration
// ---------------------------------------------------------------------------
constexpr uint32_t BAUD_RATE       = 3000000;
constexpr uint8_t  RESOLUTION_BITS = 12;
constexpr uint8_t  ADC_AVERAGING   = 2;

// ---------------------------------------------------------------------------
// Ranging / buffer
// ---------------------------------------------------------------------------
constexpr float    SAMPLE_RATE         = 178804.2f;  // measured at AVG=2, RES=12
constexpr float    SPEED_OF_SOUND_M_S  = 343.0f;
constexpr float    RANGING_DISTANCE_M  = 0.6f;
constexpr uint16_t SAMPLES_PER_CHANNEL = 1000;
constexpr uint32_t US_DELAY            = 21813;

// ---------------------------------------------------------------------------
// Pin mapping
// Channel labels S0..S3 correspond to MB0, MB1, MB3, MB2.
// S2/S3 are intentionally swapped to match the observed physical wiring.
// ---------------------------------------------------------------------------
constexpr uint8_t N_CHANNELS = 4;
constexpr uint8_t TRIGGER_PINS[N_CHANNELS] = {4, 5, 2, 3};     // MB0, MB1, MB3, MB2
constexpr uint8_t ANALOG_PINS[N_CHANNELS]  = {A2, A3, A0, A1}; // MB0, MB1, MB3, MB2
static const char* CHANNEL_LABELS[N_CHANNELS] = {"S0", "S1", "S2", "S3"};

// ---------------------------------------------------------------------------
// Sample buffer — allocated once, reused every frame
// ---------------------------------------------------------------------------
static uint16_t sample_buf[SAMPLES_PER_CHANNEL];

// ---------------------------------------------------------------------------
// Streaming state
// ---------------------------------------------------------------------------
static bool streaming = false;

// ---------------------------------------------------------------------------
// Core: sample into buffer first, then transmit — keeps ADC loop free of
// any serial overhead so it always runs at the rated 178804 Hz sample rate.
// ---------------------------------------------------------------------------
void emitFrame() {
  for (uint8_t ch = 0; ch < N_CHANNELS; ++ch) {
    // 1. Trigger ranging cycle
    digitalWrite(TRIGGER_PINS[ch], HIGH);
    delayMicroseconds(US_DELAY);

    // 2. Fill buffer (pure ADC — no serial writes here)
    for (uint16_t i = 0; i < SAMPLES_PER_CHANNEL; ++i) {
      sample_buf[i] = analogRead(ANALOG_PINS[ch]);
    }

    // 3. Stop ranging immediately after capture (not after TX)
    digitalWrite(TRIGGER_PINS[ch], LOW);

    // 4. Transmit buffer
    Serial.println(CHANNEL_LABELS[ch]);
    for (uint16_t i = 0; i < SAMPLES_PER_CHANNEL; ++i) {
      Serial.println(sample_buf[i]);
    }
    Serial.println("T");
  }
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming == CMD_START) {
      streaming = true;
    } else if (incoming == CMD_STOP) {
      if (streaming) {
        streaming = false;
        Serial.println("STREAM_END");
        Serial.flush();
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000) {}

  analogReadResolution(RESOLUTION_BITS);
  analogReadAveraging(ADC_AVERAGING);
  analogReference(0);

  for (uint8_t ch = 0; ch < N_CHANNELS; ++ch) {
    pinMode(TRIGGER_PINS[ch], OUTPUT);
    digitalWrite(TRIGGER_PINS[ch], LOW);
    pinMode(ANALOG_PINS[ch], INPUT);
  }
}

void loop() {
  handleSerialCommands();
  if (!streaming) {
    delay(1);
    return;
  }
  emitFrame();
  handleSerialCommands();
}
