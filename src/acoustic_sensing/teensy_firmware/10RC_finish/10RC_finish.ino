#include <string.h>

#define RES 12
#define AVG 2

//Trigger a ranging cycle on
#define TRIGGER_PIN_MB1 2
#define TRIGGER_PIN_MB0 3
#define TRIGGER_PIN_MB3 4
#define TRIGGER_PIN_MB2 5

#define AR_PIN_MB1 A0
#define AR_PIN_MB0 A1
#define AR_PIN_MB3 A2
#define AR_PIN_MB2 A3

#define US_DELAY 21813

constexpr float SAMPLE_RATE = 178804.2; //Note that SAMPLE_RATE is a MEASURED quantity
constexpr float C = 343.0;  //speed of sound
constexpr float RANGING_DISTANCE = 0.6; //distance, in m, that one expects to range using the rangefinder
constexpr int BUFFER_SIZE = (SAMPLE_RATE / C) * 2 * RANGING_DISTANCE * 1.2;   //How large the buffer needs to be
constexpr int NUM_RANGING_CYCLES = 10;
constexpr uint8_t START_RF_CYCLE_BYTE = 0x43; // 67, 'C'
constexpr uint8_t FINISH_BYTE = 0x45; // 69, 'E'
constexpr unsigned long FINISH_ANNOUNCE_INTERVAL_MS = 10;
constexpr unsigned long POST_BURST_FINISH_DELAY_MS = 100;
constexpr int COMMAND_BUFFER_SIZE = 8;
constexpr int COMMAND_NONE = 0;
constexpr int COMMAND_START_BURST = 1;
constexpr int COMMAND_START_TEST = 2;
constexpr int COMMAND_END_TEST = 3;

//Pin being used for analog read: A0

/*
* Sample Rates given AVG and RES
* NOTE THAT THESE CHANGING DEPENGING ON WHAT ELSE THE TEENSY IS DOING; CHECK WITH PRODUCTION CODE
*
* AVG   RES   SAMPLE RATE
* 1     12    178814
* 2     12    178814    **** SELECTED *****
* 4     12    48561
* 1     10    208664
* 2     10    208664
* 1     8     313245
* 2     8     313245
*/

/*
* Serial Command Protocol:
*   0x43 (67, 'C') - Start one acquisition burst of 10 ranging cycles.
*                    Each cycle captures all 4 rangefinders and transmits one frame:
*                    S0 <samples> T  S1 <samples> T  S2 <samples> T  S3 <samples> T
*                    After the 10th frame, the Teensy waits briefly, then continuously
*                    sends 0x45 (69, 'E') until another 0x43 is received.
*   "TEST"         - Start continuous test streaming.
*   "TESTEND"      - Stop continuous test streaming after the current frame.
*/

void setup() {
  // put your setup code here, to run once:
  pinMode(TRIGGER_PIN_MB0, OUTPUT);
  pinMode(TRIGGER_PIN_MB1, OUTPUT);
  pinMode(TRIGGER_PIN_MB2, OUTPUT);
  pinMode(TRIGGER_PIN_MB3, OUTPUT);

  Serial.begin(3000000);
  analogReadResolution(RES);
  analogReadAveraging(AVG);
  analogReference(0);

  digitalWrite(TRIGGER_PIN_MB0, LOW);
  digitalWrite(TRIGGER_PIN_MB1, LOW);
  digitalWrite(TRIGGER_PIN_MB2, LOW);
  digitalWrite(TRIGGER_PIN_MB3, LOW);

  Serial.println("Ready");
}

uint16_t buffer0[BUFFER_SIZE];
uint16_t buffer1[BUFFER_SIZE];
uint16_t buffer2[BUFFER_SIZE];
uint16_t buffer3[BUFFER_SIZE];

void append_command_char(char* command_buffer, int& command_len, char c) {
  if (command_len < COMMAND_BUFFER_SIZE - 1) {
    command_buffer[command_len++] = c;
  } else {
    memmove(command_buffer, command_buffer + 1, COMMAND_BUFFER_SIZE - 2);
    command_buffer[COMMAND_BUFFER_SIZE - 2] = c;
  }
  command_buffer[command_len] = '\0';
}

bool command_buffer_ends_with(const char* command_buffer, int command_len, const char* suffix) {
  int suffix_len = strlen(suffix);
  if (command_len < suffix_len) return false;
  return strcmp(command_buffer + command_len - suffix_len, suffix) == 0;
}

void clear_command_buffer(char* command_buffer, int& command_len) {
  command_len = 0;
  command_buffer[0] = '\0';
}

int read_command(bool test_mode) {
  static char command_buffer[COMMAND_BUFFER_SIZE] = "";
  static int command_len = 0;
  bool saw_printable = false;

  while (Serial.available()) {
    int incoming = Serial.read();

    if (!test_mode && incoming == START_RF_CYCLE_BYTE) {
      clear_command_buffer(command_buffer, command_len);
      return COMMAND_START_BURST;
    }

    if (incoming >= 32 && incoming <= 126) {
      append_command_char(command_buffer, command_len, static_cast<char>(incoming));
      saw_printable = true;
    }
  }

  if (saw_printable) {
    if (test_mode && command_buffer_ends_with(command_buffer, command_len, "TESTEND")) {
      clear_command_buffer(command_buffer, command_len);
      return COMMAND_END_TEST;
    }

    if (!test_mode) {
      if (command_buffer_ends_with(command_buffer, command_len, "TESTEND")) {
        clear_command_buffer(command_buffer, command_len);
        return COMMAND_NONE;
      }
      if (command_buffer_ends_with(command_buffer, command_len, "TEST")) {
        clear_command_buffer(command_buffer, command_len);
        return COMMAND_START_TEST;
      }
    }
  }

  return COMMAND_NONE;
}

void capture_and_stream_frame() {
  // Rangefinder 0
  digitalWrite(TRIGGER_PIN_MB0, HIGH);
  delayMicroseconds(US_DELAY);
  for (int i = 0; i < BUFFER_SIZE; i++) buffer0[i] = analogRead(AR_PIN_MB0);
  digitalWrite(TRIGGER_PIN_MB0, LOW);

  // Rangefinder 1
  digitalWrite(TRIGGER_PIN_MB1, HIGH);
  delayMicroseconds(US_DELAY);
  for (int i = 0; i < BUFFER_SIZE; i++) buffer1[i] = analogRead(AR_PIN_MB1);
  digitalWrite(TRIGGER_PIN_MB1, LOW);

  // Rangefinder 2
  digitalWrite(TRIGGER_PIN_MB2, HIGH);
  delayMicroseconds(US_DELAY);
  for (int i = 0; i < BUFFER_SIZE; i++) buffer2[i] = analogRead(AR_PIN_MB2);
  digitalWrite(TRIGGER_PIN_MB2, LOW);

  // Rangefinder 3
  digitalWrite(TRIGGER_PIN_MB3, HIGH);
  delayMicroseconds(US_DELAY);
  for (int i = 0; i < BUFFER_SIZE; i++) buffer3[i] = analogRead(AR_PIN_MB3);
  digitalWrite(TRIGGER_PIN_MB3, LOW);

  // Stream frame immediately
  Serial.println("S0");
  for (int i = 0; i < BUFFER_SIZE; i++) Serial.println(buffer0[i]);
  Serial.println('T');

  Serial.println("S1");
  for (int i = 0; i < BUFFER_SIZE; i++) Serial.println(buffer1[i]);
  Serial.println('T');

  Serial.println("S2");
  for (int i = 0; i < BUFFER_SIZE; i++) Serial.println(buffer2[i]);
  Serial.println('T');

  Serial.println("S3");
  for (int i = 0; i < BUFFER_SIZE; i++) Serial.println(buffer3[i]);
  Serial.println('T');
}

void loop() {
  static unsigned long last_finish_announce_ms = 0;
  static bool burst_finished = false;
  static bool test_mode = false;

  int command = read_command(test_mode);

  if (command == COMMAND_START_TEST) {
    test_mode = true;
    burst_finished = false;
  } else if (command == COMMAND_START_BURST) {
    burst_finished = false;
    last_finish_announce_ms = millis();
    for (int cycle = 0; cycle < NUM_RANGING_CYCLES; cycle++) {
      capture_and_stream_frame();
    }
    delay(POST_BURST_FINISH_DELAY_MS);
    Serial.write(FINISH_BYTE);
    burst_finished = true;
    last_finish_announce_ms = millis();
  }

  if (test_mode) {
    capture_and_stream_frame();
    if (read_command(test_mode) == COMMAND_END_TEST) {
      test_mode = false;
      burst_finished = false;
    }
    return;
  }

  unsigned long now = millis();
  if (burst_finished && now - last_finish_announce_ms >= FINISH_ANNOUNCE_INTERVAL_MS) {
    Serial.write(FINISH_BYTE);
    last_finish_announce_ms = now;
  }
}
