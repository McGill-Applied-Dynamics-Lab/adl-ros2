#define RES 12
#define AVG 7

constexpr float SAMPLE_RATE = 178804.2; //Note that SAMPLE_RATE is a MEASURED quantity
constexpr int BUFFER_SIZE = 1e5;   //How large the buffer needs to be

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
* 2     8     313245adc teensy
*/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(3000000);
  analogReadResolution(RES);
  analogReadAveraging(AVG);
  analogReference(0);

}

uint16_t buffer1[BUFFER_SIZE];
uint16_t buffer2[BUFFER_SIZE];

void loop() {
  // put your main code here, to run repeatedly:

  uint32_t t = micros();
  for (int i = 0; i < BUFFER_SIZE; i++){
    buffer1[i] = analogRead(A7);
    buffer2[i] = analogRead(A5);

  }
  t = micros() - t;

  //For plotting==============================================
  Serial.println('S');    //Start signal
  for (int i = 0; i < BUFFER_SIZE; i++){
    Serial.println(buffer1[i]);
    Serial.println(buffer2[i]);
  }
  Serial.println('T');    //Stop signal
  // =========================================================

  // Printing the sample rate =================================
  float sample_rate = (float) BUFFER_SIZE / t * 1000000.0;
  Serial.print("ADC sampling rate: ");
  Serial.print(sample_rate);
  Serial.println(" Hz");
  Serial.print("Buffer size: ");
  Serial.println(BUFFER_SIZE);
  // ==========================================================

  delay(100); //Wait for 1s


}
