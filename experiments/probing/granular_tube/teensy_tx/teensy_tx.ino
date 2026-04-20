#define RES 12
#define AVG 8

constexpr uint32_t CHUNK_SIZE = 500;      // Stream chunks of 500 samples (50 ms at 10 kHz)

uint16_t chunk1[CHUNK_SIZE];
uint16_t chunk2[CHUNK_SIZE];

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

    // MODE 1: STREAM MODE - Continuously stream samples in chunks (unbounded duration)
    // Command: '1' (ASCII 49) to start, '2' (ASCII 50) to stop
    if (incoming == 49) { // ASCII '1' - START STREAMING
      uint32_t total_count = 0;
      uint32_t chunk_idx = 0;
      bool stop_requested = false;

      Serial.write('B'); // Start marker for streaming mode

      // Streaming recording loop - runs until Python sends '2'
      while (!stop_requested) {
        uint32_t sample_start = micros();

        chunk1[chunk_idx] = analogRead(A7);
        chunk2[chunk_idx] = analogRead(A5);
        chunk_idx++;
        total_count++;

        // Send chunk when it reaches CHUNK_SIZE samples
        if (chunk_idx >= CHUNK_SIZE) {
          Serial.write('C');                                    // Chunk marker
          Serial.write((uint8_t*)&chunk_idx, sizeof(uint32_t)); // Samples in this chunk
          Serial.write((uint8_t*)chunk1, chunk_idx * sizeof(uint16_t));
          Serial.write((uint8_t*)chunk2, chunk_idx * sizeof(uint16_t));

          chunk_idx = 0;
        }

        // INTERRUPT CHECK: Did Python send the stop signal ('2')?
        if (Serial.available() > 0) {
          int check = Serial.read();
          if (check == 50) { // ASCII '2' - STOP STREAMING
            stop_requested = true;
          }
        }

        // 10 kHz Precision Timer
        while (micros() - sample_start < 100) {
        }
      }

      // Send final partial chunk if any samples remain
      if (chunk_idx > 0) {
        Serial.write('C');                                    // Chunk marker
        Serial.write((uint8_t*)&chunk_idx, sizeof(uint32_t)); // Samples in final chunk
        Serial.write((uint8_t*)chunk1, chunk_idx * sizeof(uint16_t));
        Serial.write((uint8_t*)chunk2, chunk_idx * sizeof(uint16_t));
      }

      // Send end marker
      Serial.write('E');                                      // End marker
      Serial.write((uint8_t*)&total_count, sizeof(uint32_t)); // Total samples recorded
    }
  }
}
