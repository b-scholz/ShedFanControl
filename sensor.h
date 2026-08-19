//      ******************************************************************
//      *                                                                *
//      *   sensor.h  -  DHT11 temperature / humidity sensor: reading,    *
//      *                oversampling and fail detection.                 *
//      *                                                                *
//      *   Public interface (used by control_loop.h/FanController.ino):          *
//      *     init() / read()                                             *
//      *     getTemp() / getHumidity() / isOk()                          *
//      *                                                                *
//      ******************************************************************

#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <DHT.h> // Adafruit "DHT sensor library" (+ "Adafruit Unified Sensor")
#include "config.h"

// ---------------------------------------------------------------------------------
//                               DHT11 Pin and Constants
// ---------------------------------------------------------------------------------
//

//
// DHT11 temperature / humidity sensor
const byte DHT_PIN = 8;

//
// After this many consecutive failed sensor reads, fall back to a safe, quiet
// airing speed (MED) so the shed keeps being ventilated.
//
const byte DHT_SENSOR_FAIL_LIMIT = 5;

//
// The DHT11 itself only has whole-degree/whole-percent resolution, and can't
// be re-sampled faster than its own ~1-2s minimum interval, so "oversampling"
// here means averaging this many of the most recent readings (taken at the
// normal CONTROL_INTERVAL pace) rather than back-to-back fast samples like
// the battery ADC does.  Averaging out the quantization/noise between reads
// gives a smoother, more precise effective value for both control and
// display.  At 2s/reading, 8 samples span ~16s of history.
//
const byte DHT_OVERSAMPLE_COUNT = 8;

class Sensor
{
public:
  Sensor() : dht(DHT_PIN, DHT11) {}

  void init() {
    dht.begin();
  }

  //
  // Read the DHT11.  On a bad read (NaN) the previous good values are kept
  // until DHT_SENSOR_FAIL_LIMIT consecutive failures, after which isOk()
  // goes false.
  //
  // On a good read, the raw sample is folded into a running sum/count (no
  // need to keep the individual samples themselves - see the oversampling
  // note above) and getTemp()/getHumidity() become that running average,
  // refining with every read. Once DHT_OVERSAMPLE_COUNT samples have
  // accumulated, the sum/count reset and averaging starts fresh for the
  // next window - a "tumbling" window rather than a true sliding one, so
  // the reported value briefly steps to just the single newest raw sample
  // right after each reset before refining again, instead of smoothly
  // dropping only the oldest sample. That trade-off is what buys removing
  // the sample buffer entirely.
  //
  void read() {
    float h = dht.readHumidity();
    float t = dht.readTemperature(); // Celsius

    if (isnan(h) || isnan(t)) {
      if (failCount < 255) {
        failCount++;
      }
      if (failCount >= DHT_SENSOR_FAIL_LIMIT) {
        ok = false;
      }
      return;
    }

    tempSum += t;
    humSum += h;
    sampleCount++;

    temp     = tempSum / sampleCount;
    humidity = humSum / sampleCount;

    if (sampleCount >= DHT_OVERSAMPLE_COUNT) {
      tempSum = humSum = 0;
      sampleCount      = 0;
    }

    failCount = 0;
    ok        = true;
  }

  float getTemp() const {
    return temp;
  }
  float getHumidity() const {
    return humidity;
  }
  bool isOk() const {
    return ok;
  }

private:
  DHT dht;

  float temp      = NAN;   // running-average temperature, C - see read()
  float humidity  = NAN;   // running-average humidity, %RH - see read()
  byte  failCount = 0;     // consecutive failed reads since the last good one
  bool  ok        = false; // false once failCount reaches DHT_SENSOR_FAIL_LIMIT

  //
  // running-average accumulator for the current oversampling window - see
  // read() above. No per-sample buffer needed, just the sum and count.
  //
  float tempSum     = 0;
  float humSum      = 0;
  byte  sampleCount = 0; // samples folded into tempSum/humSum so far (< DHT_OVERSAMPLE_COUNT)
};

extern Sensor sensor; // defined in the main sketch

#endif // SENSOR_H
