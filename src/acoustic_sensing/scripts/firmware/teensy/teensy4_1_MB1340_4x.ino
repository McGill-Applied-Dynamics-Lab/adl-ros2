#define RES 12
#define AVG 2

//Trigger a ranging cycle on 
#define TRIGGER_PIN_MB0 2
#define TRIGGER_PIN_MB1 3
#define TRIGGER_PIN_MB2 4
#define TRIGGER_PIN_MB3 5

#define AR_PIN_MB0 A0
#define AR_PIN_MB1 A1
#define AR_PIN_MB2 A2
#define AR_PIN_MB3 A3

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

}

uint16_t buffer[BUFFER_SIZE];

void loop() {
  // put your main code here, to run repeatedly:
  
  
  // Rangefinder 0,  ==========================================
  digitalWrite(TRIGGER_PIN_MB0, HIGH);  //Trigger ranging cycle
  delayMicroseconds(US_DELAY);  //calibration delay of MB
  for (int i = 0; i < BUFFER_SIZE; i++){
    buffer[i] = analogRead(AR_PIN_MB0);
  }
  digitalWrite(TRIGGER_PIN_MB0, LOW);  //Stop ranging

  Serial.println("S0");    //Start serial transmit
  for (int i = 0; i < BUFFER_SIZE; i++){
    Serial.println(buffer[i]);
  }
  Serial.println('T');    //Stop serial transmit
  // =========================================================

  // Rangefinder 1,  ==========================================
  digitalWrite(TRIGGER_PIN_MB1, HIGH);  //Trigger ranging cycle
  delayMicroseconds(US_DELAY);  //calibration delay of MB
  for (int i = 0; i < BUFFER_SIZE; i++){
    buffer[i] = analogRead(AR_PIN_MB1);
  }
  digitalWrite(TRIGGER_PIN_MB1, LOW);  //Stop ranging

  Serial.println("S1");    //Start serial transmit
  for (int i = 0; i < BUFFER_SIZE; i++){
    Serial.println(buffer[i]);
  }
  Serial.println('T');    //Stop serial transmit
  // =========================================================


  // Rangefinder 2,  ==========================================
  digitalWrite(TRIGGER_PIN_MB2, HIGH);  //Trigger ranging cycle
  delayMicroseconds(US_DELAY);  //calibration delay of MB
  for (int i = 0; i < BUFFER_SIZE; i++){
    buffer[i] = analogRead(AR_PIN_MB2);
  }
  digitalWrite(TRIGGER_PIN_MB2, LOW);  //Stop ranging

  Serial.println("S2");    //Start serial transmit
  for (int i = 0; i < BUFFER_SIZE; i++){
    Serial.println(buffer[i]);
  }
  Serial.println('T');    //Stop serial transmit
  // =========================================================

  // Rangefinder 3,  ==========================================
  digitalWrite(TRIGGER_PIN_MB3, HIGH);  //Trigger ranging cycle
  delayMicroseconds(US_DELAY);  //calibration delay of MB
  for (int i = 0; i < BUFFER_SIZE; i++){
    buffer[i] = analogRead(AR_PIN_MB3);
  }
  digitalWrite(TRIGGER_PIN_MB3, LOW);  //Stop ranging

  Serial.println("S3");    //Start serial transmit
  for (int i = 0; i < BUFFER_SIZE; i++){
    Serial.println(buffer[i]);
  }
  Serial.println('T');    //Stop serial transmit
  // =========================================================

  

  

  


}
