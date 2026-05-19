// quad_tx.ino — four-channel pressure streamer for the granular-bed setup.
//
// Reads analog pins 14, 15, 16, 17 (A0–A3) and streams the four 12-bit ADC
// channels over the Teensy USB serial link in fixed-size chunks. Wire layout
// matches teensy_tx_multi.ino's three-channel format with a fourth uint16[]
// block per chunk (see the 'C' comment below).
//
// Sample rate target: ~40 kHz (25 µs per 4-channel sample).
//
// Why the ADC library:
//   stock analogRead() with averaging takes ~20 µs per call on Teensy 3.x, so
//   four sequential reads cap out near 12 kHz. The Teensy core's ADC library
//   exposes both ADC modules and a synchronized-pair read, so we sample two
//   pins simultaneously and only need two reads per channel set (≈10 µs at
//   HIGH_SPEED with AVG=1).

#include <ADC.h>

#define RES 12
#define AVG 12                  // No on-chip averaging — keeps each pair < ~5 µs.
#define SAMPLE_PERIOD_US 50    // 25 µs ⇒ 40 kHz nominal

// Uncomment to disable streaming and instead print the measured ADC-pair
// timings every 500 ms over Serial. The first reported number is how long
// analogSynchronizedRead(14, 15) takes; the second is the same for (16, 17).
// The first number is also the inter-pair skew — i.e. how far behind ch3/ch4
// are sampled relative to ch1/ch2. Open the Arduino Serial Monitor with the
// host scripts NOT running to observe.
// #define DEBUG_TIMING

constexpr uint32_t CHUNK_SIZE = 500;  // 500 samples ⇒ 12.5 ms at 40 kHz

uint16_t chunk1[CHUNK_SIZE];
uint16_t chunk2[CHUNK_SIZE];
uint16_t chunk3[CHUNK_SIZE];
uint16_t chunk4[CHUNK_SIZE];

ADC *adc = new ADC();

void setup() {
  Serial.begin(3000000);  // USB-CDC: baud is informational, runs at ~12 MB/s.

  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);

  // ADC0 settings.
  adc->adc0->setAveraging(AVG);
  adc->adc0->setResolution(RES);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3);

#ifdef ADC_DUAL_ADCS
  // ADC1 settings (Teensy 3.1/3.2/3.5/3.6/4.x).
  adc->adc1->setAveraging(AVG);
  adc->adc1->setResolution(RES);
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
  adc->adc1->setReference(ADC_REFERENCE::REF_3V3);
#endif

  Serial.println("Ready");
}

void loop() {
#ifdef DEBUG_TIMING
  // Diagnostic mode — replaces the streaming protocol with a text-only loop
  // that times the two synchronizedRead() pair calls and prints stats every
  // 500 ms. Comment out #define DEBUG_TIMING above to restore streaming.
  const uint32_t N = 1000;
  uint32_t sum1 = 0, sum2 = 0;
  uint32_t max1 = 0, max2 = 0;
  for (uint32_t k = 0; k < N; k++) {
    uint32_t t0 = micros();
#ifdef ADC_DUAL_ADCS
    ADC::Sync_result r1 = adc->analogSynchronizedRead(14, 15);
    (void)r1;
#endif
    uint32_t t1 = micros();
#ifdef ADC_DUAL_ADCS
    ADC::Sync_result r2 = adc->analogSynchronizedRead(16, 17);
    (void)r2;
#endif
    uint32_t t2 = micros();
    uint32_t d1 = t1 - t0;
    uint32_t d2 = t2 - t1;
    sum1 += d1;
    sum2 += d2;
    if (d1 > max1) max1 = d1;
    if (d2 > max2) max2 = d2;
  }
  Serial.print("pair1(14,15) mean_us=");
  Serial.print((float)sum1 / N, 2);
  Serial.print(" max_us=");
  Serial.print(max1);
  Serial.print(" | pair2(16,17) mean_us=");
  Serial.print((float)sum2 / N, 2);
  Serial.print(" max_us=");
  Serial.print(max2);
  Serial.print("  =>  ch3,ch4 lag ch1,ch2 by ~");
  Serial.print((float)sum1 / N, 2);
  Serial.println(" us");
  delay(500);
  return;
#endif

  if (Serial.available() > 0) {
    int incoming = Serial.read();

    // STREAM MODE — continuous streaming in CHUNK_SIZE blocks.
    // Command: '1' (ASCII 49) starts, '2' (ASCII 50) stops.
    //
    // Each 'C' chunk packet on the wire:
    //   'C'                                     1 byte
    //   uint32  chunk_size                      4 bytes
    //   uint16[chunk_size] ch1                  2*chunk_size bytes  (pin 14 / A0)
    //   uint16[chunk_size] ch2                  2*chunk_size bytes  (pin 15 / A1)
    //   uint16[chunk_size] ch3                  2*chunk_size bytes  (pin 16 / A2)
    //   uint16[chunk_size] ch4                  2*chunk_size bytes  (pin 17 / A3)
    //   float32 chunk_rate_hz                   4 bytes  (intra-chunk sample rate)
    //   uint32  dt_since_last_us                4 bytes  (first-sample-of-this-chunk
    //                                                     minus first-sample-of-prev-chunk;
    //                                                     for the first chunk: minus stream_start_us)
    if (incoming == 49) {
      uint32_t total_count = 0;
      uint32_t chunk_idx = 0;
      bool stop_requested = false;

      uint32_t stream_start_us = micros();
      uint32_t prev_first_sample_us = stream_start_us;
      uint32_t first_sample_us = 0;
      uint32_t last_sample_us = 0;

      Serial.write('B');  // Start marker

      while (!stop_requested) {
        uint32_t sample_start = micros();

        if (chunk_idx == 0) {
          first_sample_us = sample_start;
        }
        last_sample_us = sample_start;

#ifdef ADC_DUAL_ADCS
        // Sample pin 14 + pin 15 in parallel on ADC0/ADC1, then pin 16 + 17.
        ADC::Sync_result r1 = adc->analogSynchronizedRead(14, 15);
        ADC::Sync_result r2 = adc->analogSynchronizedRead(16, 17);
        chunk1[chunk_idx] = (uint16_t)r1.result_adc0;
        chunk2[chunk_idx] = (uint16_t)r1.result_adc1;
        chunk3[chunk_idx] = (uint16_t)r2.result_adc0;
        chunk4[chunk_idx] = (uint16_t)r2.result_adc1;
#else
        // Single-ADC fallback (Teensy LC / 3.0). Won't hit 40 kHz here.
        chunk1[chunk_idx] = (uint16_t)adc->adc0->analogRead(14);
        chunk2[chunk_idx] = (uint16_t)adc->adc0->analogRead(15);
        chunk3[chunk_idx] = (uint16_t)adc->adc0->analogRead(16);
        chunk4[chunk_idx] = (uint16_t)adc->adc0->analogRead(17);
#endif
        chunk_idx++;
        total_count++;

        if (chunk_idx >= CHUNK_SIZE) {
          uint32_t collect_us = last_sample_us - first_sample_us;
          float chunk_rate_hz = 0.0f;
          if (collect_us > 0 && chunk_idx > 1) {
            chunk_rate_hz =
                ((float)(chunk_idx - 1)) / ((float)collect_us * 1e-6f);
          }
          uint32_t dt_since_last_us = first_sample_us - prev_first_sample_us;
          prev_first_sample_us = first_sample_us;

          Serial.write('C');
          Serial.write((uint8_t*)&chunk_idx, sizeof(uint32_t));
          Serial.write((uint8_t*)chunk1, chunk_idx * sizeof(uint16_t));
          Serial.write((uint8_t*)chunk2, chunk_idx * sizeof(uint16_t));
          Serial.write((uint8_t*)chunk3, chunk_idx * sizeof(uint16_t));
          Serial.write((uint8_t*)chunk4, chunk_idx * sizeof(uint16_t));
          Serial.write((uint8_t*)&chunk_rate_hz, sizeof(float));
          Serial.write((uint8_t*)&dt_since_last_us, sizeof(uint32_t));

          chunk_idx = 0;
        }

        if (Serial.available() > 0) {
          int check = Serial.read();
          if (check == 50) {  // ASCII '2' - STOP STREAMING
            stop_requested = true;
          }
        }

        // 40 kHz precision timer (25 µs period).
        while (micros() - sample_start < SAMPLE_PERIOD_US) {
        }
      }

      // Flush any partial chunk before the end marker.
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
        Serial.write((uint8_t*)chunk3, chunk_idx * sizeof(uint16_t));
        Serial.write((uint8_t*)chunk4, chunk_idx * sizeof(uint16_t));
        Serial.write((uint8_t*)&chunk_rate_hz, sizeof(float));
        Serial.write((uint8_t*)&dt_since_last_us, sizeof(uint32_t));
      }

      Serial.write('E');
      Serial.write((uint8_t*)&total_count, sizeof(uint32_t));
    }
  }
}
