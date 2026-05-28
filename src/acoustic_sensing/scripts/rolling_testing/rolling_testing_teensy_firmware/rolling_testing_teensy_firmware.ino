#include <Arduino.h>

namespace {

constexpr uint8_t CMD_START = 67;  // 'C'
constexpr uint8_t CMD_STOP = 69;   // 'E'

constexpr uint32_t BAUD_RATE = 3000000;
constexpr uint8_t RESOLUTION_BITS = 12;
constexpr uint8_t ADC_AVERAGING = 2;
constexpr uint8_t N_CHANNELS = 4;

constexpr float SAMPLE_RATE = 178804.2f;
constexpr float SPEED_OF_SOUND_M_S = 343.0f;
constexpr float RANGING_DISTANCE_M = 0.6f;
// Host-side parsers (EXPECTED_RF_SAMPLES) hard-code 1000, so keep this fixed
// at 1000 to match.  The float constants above are kept for documentation.
constexpr uint16_t SAMPLES_PER_CHANNEL = 1000;

// Match src/acoustic_sensing/teensy_firmware/teensy_tx 1.ino — straight
// channel-to-pin mapping (no S2/S3 swap).
// Channel labels S0..S3 correspond to MB0, MB1, MB2, MB3.
constexpr uint8_t TRIGGER_PINS[N_CHANNELS] = {2, 3, 4, 5};      // MB0, MB1, MB2, MB3
constexpr uint8_t ANALOG_PINS[N_CHANNELS] = {A0, A1, A2, A3};   // MB0, MB1, MB2, MB3
constexpr uint32_t US_DELAY = 21813;

bool streaming = false;

void emitFrame() {
  static const char* kChannelLabels[N_CHANNELS] = {"S0", "S1", "S2", "S3"};

  for (uint8_t ch = 0; ch < N_CHANNELS; ++ch) {
    digitalWrite(TRIGGER_PINS[ch], HIGH);
    delayMicroseconds(US_DELAY);
    Serial.println(kChannelLabels[ch]);
    for (uint16_t i = 0; i < SAMPLES_PER_CHANNEL; ++i) {
      const int sample = analogRead(ANALOG_PINS[ch]);
      Serial.println(sample);
    }
    digitalWrite(TRIGGER_PINS[ch], LOW);
    Serial.println("T");
  }
}

void stopStreaming() {
  if (!streaming) {
    return;
  }
  streaming = false;
  Serial.println("STREAM_END");
  Serial.flush();
}

void startStreaming() {
  streaming = true;
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming == CMD_START) {
      startStreaming();
    } else if (incoming == CMD_STOP) {
      stopStreaming();
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000) {
  }

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
