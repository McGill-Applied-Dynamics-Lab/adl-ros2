#define RES 12
#define AVG 2

//Trigger a ranging cycle on
#define TRIGGER_PIN_MB3 2
#define TRIGGER_PIN_MB2 3
#define TRIGGER_PIN_MB0 4
#define TRIGGER_PIN_MB1 5

#define AR_PIN_MB3 A0
#define AR_PIN_MB2 A1
#define AR_PIN_MB0 A2
#define AR_PIN_MB1 A3

#define US_DELAY 21813

constexpr float SAMPLE_RATE = 178804.2; //Note that SAMPLE_RATE is a MEASURED quantity
constexpr float C = 343.0;  //speed of sound
constexpr float RANGING_DISTANCE = 0.6; //distance, in m, that one expects to range using the rangefinder
constexpr int BUFFER_SIZE = (SAMPLE_RATE / C) * 2 * RANGING_DISTANCE * 1.2;   //How large the buffer needs to be

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
*   0x43 (67, 'C') - Start streaming: continuously capture all 4 rangefinders
*                    and transmit each frame immediately over serial.
*                    Frame format: S0 <samples> T  S1 <samples> T  S2 <samples> T  S3 <samples> T
*                    Repeats until 0x45 is received between frames.
*   0x45 (69, 'E') - Stop streaming: finish current frame, then send "STREAM_END"
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
  int incoming = Serial.read();

  // 0x43 ('C') - Start continuous streaming
  if (incoming == 0x43) {
    while (true) {
      capture_and_stream_frame();
      // Check for stop command between frames
      if (Serial.available() && Serial.read() == 0x45) break;
    }
    Serial.println("STREAM_END");
  }
}
