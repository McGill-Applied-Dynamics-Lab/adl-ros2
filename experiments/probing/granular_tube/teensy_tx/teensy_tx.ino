#define RES 12
#define AVG 8

constexpr uint32_t BUFFER_SIZE = 100000; // 100,000 samples (10 seconds at 10 kHz)

uint16_t buffer1[BUFFER_SIZE];
uint16_t buffer2[BUFFER_SIZE];

void setup() {
  Serial.begin(3000000);
  analogReadResolution(RES);
  analogReadAveraging(AVG);
  analogReference(0);
  Serial.println("Ready");
}

void loop() {
  if (Serial.available() > 0) {
    int incoming = Serial.read();
    
    // START RECORDING SIGNAL
    if (incoming == 49) { // ASCII '1'
      uint32_t t = micros();
      uint32_t count = 0;
      bool stop_received = false;

      // Recording loop
      while (count < BUFFER_SIZE) {
        uint32_t sample_start = micros();
        
        buffer1[count] = analogRead(A7);
        buffer2[count] = analogRead(A5);
        count++;

        // INTERRUPT CHECK: Did Python send the stop signal ('2')?
        if (Serial.available() > 0) {
          int check = Serial.read();
          if (check == 50) { // ASCII '2'
            stop_received = true;
            break; 
          }
        }

        // 10 kHz Precision Timer
        while (micros() - sample_start < 100) {
        }
      }
      t = micros() - t;

      if (!stop_received) {
        while (true) {
          if (Serial.available() > 0) {
            if (Serial.read() == 50) { 
              break;
            }
          }
        }
      }

      // ================= RAW BINARY DATA DUMP =================
      Serial.write('S'); // Start marker
      
      // 1. Send the number of samples recorded (4 bytes)
      Serial.write((uint8_t*)&count, sizeof(uint32_t));
      
      // 2. Blast the raw memory bytes for buffer 1
      Serial.write((uint8_t*)buffer1, count * sizeof(uint16_t));
      
      // 3. Blast the raw memory bytes for buffer 2
      Serial.write((uint8_t*)buffer2, count * sizeof(uint16_t));
      
      // 4. Send the time taken
      Serial.print("T:"); 
      Serial.println(t);
    }
  }
}