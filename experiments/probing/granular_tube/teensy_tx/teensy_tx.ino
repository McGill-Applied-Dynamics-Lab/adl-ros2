#define RES 12
#define AVG 8

constexpr uint32_t CHUNK_SIZE = 500;       // Stream chunks of 500 samples (50 ms at 10 kHz)

uint16_t chunk1[CHUNK_SIZE];
uint16_t chunk2[CHUNK_SIZE];

void setup() {
  Serial.begin(3000000);
  analogReadResolution(RES);Caveat: as you noted last turn, the setup-time estimate runs without chunk-send overhead, so it'll report ~10 kHz rather than the slightly-lower true streaming rate (~9.7–9.9 kHz). If that bias matters, the cleanest follow-up is to instead update the firmware to maintain a running rate during streaming (total_count / (micros() - stream_start)) and stamp that into each chunk — same wire format, the rate just converges to truth as the stream progresses. Let me know if you want that next.
  analogReadAveraging(AVG);
  analogReference(0);
  Serial.println("Ready");
}

void loop() {
  if (Serial.available() > 0) {
    int incoming = Serial.read();

    // STREAM MODE - Continuously stream samples in chunks (unbounded duration).
    // Command: '1' (ASCII 49) to start, '2' (ASCII 50) to stop.
    //
    // Each 'C' chunk packet on the wire is:
    //   'C'                                     1 byte
    //   uint32  chunk_size                      4 bytes
    //   uint16[chunk_size] ch1                  2*chunk_size bytes
    //   uint16[chunk_size] ch2                  2*chunk_size bytes
    //   float32 chunk_rate_hz                   4 bytes  (intra-chunk sample rate)
    //   uint32  dt_since_last_us                4 bytes  (first-sample-of-this-chunk
    //                                                     minus first-sample-of-prev-chunk;
    //                                                     for the first chunk: minus stream_start_us)
    //
    // The receiver reconstructs per-sample timestamps as:
    //   t_chunk_start[k] = t_chunk_start[k-1] + dt_since_last_us[k] / 1e6
    //   t_sample[k][j]   = t_chunk_start[k] + j / chunk_rate_hz[k]
    if (incoming == 49) {
      uint32_t total_count = 0;
      uint32_t chunk_idx = 0;
      bool stop_requested = false;

      uint32_t stream_start_us = micros();
      uint32_t prev_first_sample_us = stream_start_us;
      uint32_t first_sample_us = 0;
      uint32_t last_sample_us = 0;

      Serial.write('B'); // Start marker for streaming mode

      while (!stop_requested) {
        uint32_t sample_start = micros();

        if (chunk_idx == 0) {
          first_sample_us = sample_start;
        }
        last_sample_us = sample_start;

        chunk1[chunk_idx] = analogRead(A7);
        chunk2[chunk_idx] = analogRead(A5);
        chunk_idx++;
        total_count++;

        if (chunk_idx >= CHUNK_SIZE) {
          uint32_t collect_us = last_sample_us - first_sample_us;
          float chunk_rate_hz = 0.0f;
          if (collect_us > 0 && chunk_idx > 1) {
            // (chunk_idx - 1) sample-to-sample intervals span collect_us.
            chunk_rate_hz =
                ((float)(chunk_idx - 1)) / ((float)collect_us * 1e-6f);
          }
          uint32_t dt_since_last_us = first_sample_us - prev_first_sample_us;
          prev_first_sample_us = first_sample_us;

          Serial.write('C');
          Serial.write((uint8_t*)&chunk_idx, sizeof(uint32_t));
          Serial.write((uint8_t*)chunk1, chunk_idx * sizeof(uint16_t));
          Serial.write((uint8_t*)chunk2, chunk_idx * sizeof(uint16_t));
          Serial.write((uint8_t*)&chunk_rate_hz, sizeof(float));
          Serial.write((uint8_t*)&dt_since_last_us, sizeof(uint32_t));

          chunk_idx = 0;
        }

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

      // Send final partial chunk if any samples remain.
      if (chunk_idx > 0) {
        uint32_t collect_us = last_sample_us - first_sample_us;
        float chunk_rate_hz = 0.0f;
        if (collect_us > 0 && chunk_idx > 1) {
          chunk_rate_hz =
              ((float)(chunk_idx - 1)) / ((float)collect_us * 1e-6f);
        }
        uint32_t dt_since_last_us = first_sample_us - prev_first_sample_us;

        Serial.write('C');
        Serial.write((uint8_t*)&chunk_idx, sizeof(uint32_t));
        Serial.write((uint8_t*)chunk1, chunk_idx * sizeof(uint16_t));
        Serial.write((uint8_t*)chunk2, chunk_idx * sizeof(uint16_t));
        Serial.write((uint8_t*)&chunk_rate_hz, sizeof(float));
        Serial.write((uint8_t*)&dt_since_last_us, sizeof(uint32_t));
      }

      Serial.write('E');
      Serial.write((uint8_t*)&total_count, sizeof(uint32_t));
    }
  }
}
