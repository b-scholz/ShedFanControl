//      ******************************************************************
//      *                                                                *
//      *   control_loop.h  -  Control Loop: manual/auto mode, climate    *
//      *                      thresholds, the automatic target-RPM       *
//      *                      law, and the per-cycle orchestration that  *
//      *                      ties sensor/battery/fan control together.  *
//      *                                                                 *
//      *   Public interface (used by FanController.ino):                    *
//      *     init() / service()                                          *
//      *     get/setManualMode() / get/setManualPct()                    *
//      *     previewManualDuty(pct) - live speed while dragging the       *
//      *       manual-speed slider (a no-op unless manual mode is         *
//      *       already on)                                               *
//      *     get/setTempStart() / get/setTempHigh()                      *
//      *     get/setHumStart() / get/setHumHigh()                        *
//      *     getTempHistory() / getHumHistory() / getVoltageHistory()    *
//      *       - each returns the underlying History (history.h)         *
//      *       directly, fed every service() cycle from Sensor/Battery's  *
//      *       live readings - see graphs.h, which plots any History      *
//      *       the same way. History-keeping is ControlLoop's              *
//      *       responsibility, not Sensor's/Battery's: it's the one        *
//      *       place that already reads every value every cycle, so       *
//      *       feeding the history buffers here needs no extra plumbing.  *
//      *                                                                 *
//      *   FanController.ino never touches tempStart/manualMode/etc.        *
//      *   directly - only through the methods above, which also own     *
//      *   persisting each change to EEPROM. This is the "clear          *
//      *   separation" between the menu and the control loop it drives.  *
//      *                                                                 *
//      *   Depends on Sensor/Battery/FanControl only through references  *
//      *   passed into its constructor (dependency injection) - it has   *
//      *   no implicit coupling to their global instances.               *
//      *                                                                 *
//      ******************************************************************

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <Arduino.h>
#include <ArduinoUserInterface.h>
#include "config.h"
#include "module.h"
#include "history.h"
#include "sensor.h"
#include "battery.h"
#include "fans_controller.h"

// ---------------------------------------------------------------------------------
//                     History: temp/humidity/battery-voltage graphs
// ---------------------------------------------------------------------------------
//
// ControlLoop owns the 48h history behind the Temp/Humid/Battery/Charge graph
// screens - not Sensor/Battery, even though the raw values come from them -
// because this is the one place that already reads every value every cycle
// (see service() below), so feeding the history buffers here needs no extra
// plumbing. All three are simple ring buffers of HISTORY_POINTS points (see
// History, history.h - HISTORY_POINTS itself lives in config.h, since it's
// needed by files included before this one), fed by a running accumulator
// that averages the raw samples falling into each point's time bucket.
//
// CONTROL_INTERVAL (config.h, same reason) is this cadence in ms.
//
// how far back the temp/humidity graphs reach; matches BATTERY_HISTORY_HOURS
// below so both graphs cover the same span
//
const uint8_t TEMP_HUMID_HISTORY_HOURS = 48;

const uint16_t RAW_SAMPLES_PER_BUCKET =
    (uint16_t)(((uint32_t) TEMP_HUMID_HISTORY_HOURS * 60UL * 60UL * 1000UL)
               / HISTORY_POINTS / CONTROL_INTERVAL);

//
// how far back the battery voltage/charge graphs reach
//
const uint8_t BATTERY_HISTORY_HOURS = 48;

const uint16_t BATTERY_SAMPLES_PER_BUCKET =
    (uint16_t)(((uint32_t) BATTERY_HISTORY_HOURS * 60UL * 60UL * 1000UL)
               / HISTORY_POINTS / CONTROL_INTERVAL);


// ---------------------------------------------------------------------------------
//                          Default climate thresholds
// ---------------------------------------------------------------------------------
//
// Power-up defaults; changeable at runtime from the menu and stored in EEPROM
// (see control_loop.h, which owns this runtime state and its persistence).
// Units: temperature in degrees Celsius, humidity in %RH.
//
//   value <= *_START  -> fans off
//   *_START..*_HIGH   -> fans ramp LOW..MED (quiet band)
//   value >= *_HIGH   -> fans unlock HIGH (noisy)
//
const int DEFAULT_TEMP_START = 25;   // start airing at 25 C
const int DEFAULT_TEMP_HIGH  = 32;   // full speed at 32 C
const int DEFAULT_HUM_START  = 65;   // start airing at 65 %RH
const int DEFAULT_HUM_HIGH   = 85;   // full speed at 85 %RH

//
// Hysteresis (deadband): a threshold must be re-crossed by this much before the
// fans step down again, preventing chatter when a reading sits on a threshold.
//
const int TEMP_HYSTERESIS = 2;   // degrees C
const int HUM_HYSTERESIS  = 5;   // %RH

//
// EEPROM layout - offsets from EEPROM_CONTROL_BASE_IDX (config.h); nothing
// outside this file needs to know these addresses.
//
const int EEPROM_TEMP_START_IDX  = EEPROM_CONTROL_BASE_IDX;               // int  (3)
const int EEPROM_TEMP_HIGH_IDX   = EEPROM_TEMP_START_IDX + 3;             // int  (3)
const int EEPROM_HUM_START_IDX   = EEPROM_TEMP_HIGH_IDX  + 3;             // int  (3)
const int EEPROM_HUM_HIGH_IDX    = EEPROM_HUM_START_IDX  + 3;             // int  (3)
const int EEPROM_MANUAL_MODE_IDX = EEPROM_HUM_HIGH_IDX   + 3;             // byte (2)
const int EEPROM_MANUAL_PCT_IDX  = EEPROM_MANUAL_MODE_IDX + 2;            // int  (3)

class ControlLoop : public Module {
public:
  ControlLoop(ArduinoUserInterface &uiRef, Sensor &sensorRef, Battery &batteryRef, FansController &fanControlRef)
    : Module(uiRef), sensor(sensorRef), battery(batteryRef), fanControl(fanControlRef),
      tempHistory(RAW_SAMPLES_PER_BUCKET), humHistory(RAW_SAMPLES_PER_BUCKET),
      voltageHistory(BATTERY_SAMPLES_PER_BUCKET) {}

  void init() {
    tempStart = ui.readConfigurationInt(EEPROM_TEMP_START_IDX, DEFAULT_TEMP_START);
    tempHigh  = ui.readConfigurationInt(EEPROM_TEMP_HIGH_IDX,  DEFAULT_TEMP_HIGH);
    humStart  = ui.readConfigurationInt(EEPROM_HUM_START_IDX,  DEFAULT_HUM_START);
    humHigh   = ui.readConfigurationInt(EEPROM_HUM_HIGH_IDX,   DEFAULT_HUM_HIGH);

    //
    // guard against a blank/corrupt EEPROM producing nonsensical values
    //
    if (tempHigh <= tempStart) {
        tempHigh = tempStart + 1;
    }
    if (humHigh  <= humStart)  {
        humHigh  = humStart + 1;
    }

    manualMode = ui.readConfigurationByte(EEPROM_MANUAL_MODE_IDX, 0) != 0;
    manualPct  = ui.readConfigurationInt(EEPROM_MANUAL_PCT_IDX,  60);   // reasonable default test duty
    manualPct  = constrain(manualPct, 0, 100);

    //
    // back-date lastControl so the very first service() call (setup()
    // calls it once directly, so the first status screen isn't blank)
    // always runs immediately instead of waiting a full CONTROL_INTERVAL
    // from boot
    //
    lastControl = millis() - CONTROL_INTERVAL;
  }


  // -------------------------------------------------------------------------------
  //                    Public interface: manual/auto mode
  // -------------------------------------------------------------------------------

  void setManualMode(bool on) {
    manualMode = on;
    ui.writeConfigurationByte(EEPROM_MANUAL_MODE_IDX, manualMode ? 1 : 0);
  }

  bool getManualMode() const {
      return manualMode;
  }

  void setManualPct(byte pct) {
    manualPct = constrain(pct, 0, 100);
    ui.writeConfigurationInt(EEPROM_MANUAL_PCT_IDX, manualPct);
  }

  byte getManualPct() const {
      return manualPct;
  }

  //
  // live speed preview while the user is dragging the "Fan speed %" slider
  // (see FanController.ino) - takes effect immediately rather than waiting for
  // the next service() cycle, so it can be tested by ear. A no-op if manual
  // mode isn't actually on, so the menu never needs to check that itself.
  //
  void previewManualDuty(byte pct) {
    if (manualMode) {
        fanControl.driveManual((byte) constrain(pct, 0, 100), battery.getVoltage());
    }
  }


  // -------------------------------------------------------------------------------
  //                  Public interface: climate thresholds
  // -------------------------------------------------------------------------------

  void setTempStart(int c) {
    tempStart = c;
    if (tempHigh <= tempStart) {
        tempHigh = tempStart + 1;     // keep high > start
    }
    ui.writeConfigurationInt(EEPROM_TEMP_START_IDX, tempStart);
    ui.writeConfigurationInt(EEPROM_TEMP_HIGH_IDX, tempHigh);
  }

  int getTempStart() const {
    return tempStart;
  }

  void setTempHigh(int c) {
    tempHigh = c;
    if (tempStart >= tempHigh) {
      tempStart = tempHigh - 1;     // keep start < high
    }
    ui.writeConfigurationInt(EEPROM_TEMP_HIGH_IDX, tempHigh);
    ui.writeConfigurationInt(EEPROM_TEMP_START_IDX, tempStart);
  }

  int getTempHigh() const {
      return tempHigh;
  }

  void setHumStart(int pct) {
    humStart = pct;
    if (humHigh <= humStart) {
      humHigh = min(humStart + 1, 100);
    }
    ui.writeConfigurationInt(EEPROM_HUM_START_IDX, humStart);
    ui.writeConfigurationInt(EEPROM_HUM_HIGH_IDX, humHigh);
  }

  int getHumStart() const {
    return humStart;
  }

  void setHumHigh(int pct) {
    humHigh = pct;
    if (humStart >= humHigh) {
      humStart = humHigh - 1;
    }
    ui.writeConfigurationInt(EEPROM_HUM_HIGH_IDX, humHigh);
    ui.writeConfigurationInt(EEPROM_HUM_START_IDX, humStart);
  }

  int getHumHigh() const {
      return humHigh;
  }

  // -------------------------------------------------------------------------------
  //                    Public interface: history (graphs.h)
  // -------------------------------------------------------------------------------
  //
  // direct access to the underlying History objects (history.h), fed every
  // service() cycle below - see the interface note up top for why this
  // lives here rather than on Sensor/Battery.
  //
  const History &getTempHistory() const    {
      return tempHistory;
  }
  const History &getHumHistory() const     {
      return humHistory;
  }
  const History &getVoltageHistory() const {
      return voltageHistory;
  }


  // -------------------------------------------------------------------------------
  //                     Per-cycle control loop orchestration
  // -------------------------------------------------------------------------------
  //
  // Called frequently, but only samples the sensor, reads the tachs and
  // re-evaluates the fan speed once per CONTROL_INTERVAL (the DHT11
  // cannot be read faster than that).  Ties together every subsystem's
  // per-cycle work.
  //
  void service() {
    unsigned long now = millis();
    if (now - lastControl < CONTROL_INTERVAL) {
      return;
    }
    lastControl = now;

    sensor.read();
    fanControl.service();

    //
    // feed the history graphs only on valid reads, so a dead sensor pauses
    // history rather than polluting the averages with stale data. Values are
    // rounded to the byte-sized unit each History stores (whole degrees C,
    // whole %RH) - see History::addSample() in history.h.
    //
    if (sensor.isOk()) {
      tempHistory.addSample((uint8_t)(sensor.getTemp() + 0.5));
      humHistory.addSample((uint8_t)(sensor.getHumidity() + 0.5));
    }

    //
    // battery voltage has nothing to do with the DHT11, so it's read and
    // logged every cycle regardless of sensorOk. getVoltage() already has
    // the user-calibrated scalar applied - this loop only ever deals in
    // "the battery voltage," never a raw reading. Captured once here and
    // reused below (rather than a second ADC read) since driveAuto() also
    // needs it, to compensate the fans' PID gains for the battery's actual
    // voltage right now - see the note in fan.h. Rounded to tenths of a volt
    // for the history buffer, same as sensor.h's units above.
    //
    battery.read();
    float voltage = battery.getVoltage();
    voltageHistory.addSample((uint8_t)(voltage * 10.0 + 0.5));

    if (manualMode) {
      //
      // manual override: the user sets duty directly, bypassing the PID
      // entirely (for servicing/diagnostics) - see FanControl::driveManual()
      //
      fanControl.driveManual((byte) constrain(manualPct, 0, 100), voltage);
    } else {
      unsigned int targetRpm;
      if (!sensor.isOk()) {
        //
        // sensor is dead - keep airing the shed at a safe, quiet MED speed
        // rather than guessing or stopping entirely
        //
        fansRunning = true;
        highUnlocked = false;
        targetRpm = fanControl.getMedRpm(voltage);
      } else {
        targetRpm = computeAutoTargetRpm(sensor.getTemp(), sensor.getHumidity(), voltage);
      }
      fanControl.driveAuto(targetRpm, voltage);
    }
  }

private:
  Sensor &sensor;
  Battery &battery;
  FansController &fanControl;

  //
  // Manual or auto mode: if true, manual mode is used to control fans;
  // otherwise the temp/humidity sensor determines when the fans are
  // switched. Loaded from EEPROM in init(). Only ever changed through
  // setManualMode()/setManualPct() - see the interface note above.
  //
  bool manualMode;      // true = ignore sensor, run at manualPct
  int  manualPct;       // fan duty used while in manual mode (0..100 %)

  //
  // active thresholds, loaded from EEPROM at power-up (see config.h for
  // meaning). Only ever changed through setTempStart/High()/setHumStart/High().
  //
  int tempStart;
  int tempHigh;
  int humStart;
  int humHigh;

  //
  // hysteresis latches around the "start"/"high" thresholds - see
  // computeAutoTargetRpm(). Also touched directly by service()'s
  // sensor-fail fallback.
  //
  bool fansRunning = false;      // fans on at all (start-threshold latch)
  bool highUnlocked = false;     // allowed to use noisy HIGH speed (high-threshold latch)

  //
  // last time the control law was evaluated
  //
  unsigned long lastControl = 0;

  //
  // the 48h temp/humidity/battery-voltage history buffers behind the graph
  // screens - see the interface note up top and History (history.h).
  //
  History tempHistory;
  History humHistory;
  History voltageHistory;

  //
  // How hard this reading is asking the fans to work: 0.0 at (or below) the
  // start threshold, rising linearly to 1.0 at (or above) the high threshold.
  //
  float airingDemand(float value, int startT, int highT) const {
    if (value <= startT) { return 0.0; }
    if (value >= highT)  { return 1.0; }
    return (value - startT) / (float)(highT - startT);
  }

  //
  // The automatic speed decision.  Returns the target RPM for the current
  // climate (FanControl::driveAuto() then drives each fan's real speed to
  // this target - see the note in fan_control.h for why RPM instead of
  // duty).
  //
  //   * Below both "start" thresholds  -> OFF (target 0).
  //   * Between "start" and "high"      -> ramp LOW..MED RPM (the quiet band),
  //                                        driven by whichever of temperature/
  //                                        humidity is more demanding.
  //   * At or above either "high"        -> HIGH RPM (noisy) - the only case in
  //     threshold                         which we accept the noise and wear.
  //
  // Start/stop and the jump to HIGH each use hysteresis so a reading sitting
  // on a threshold cannot make the fans chatter. batteryVoltage is only
  // needed to evaluate the LOW/MED/HIGH RPM targets themselves at their
  // calibrated voltage - see FanControl::getLowRpm()/getMedRpm()/
  // getHighRpm() in fan_control.h.
  //
  unsigned int computeAutoTargetRpm(float temp, float humidity, float batteryVoltage) {
    //
    // on/off latch around the "start" thresholds
    //
    if (temp >= tempStart || humidity >= humStart) {
      fansRunning = true;
    } else if (temp <= tempStart - TEMP_HYSTERESIS && humidity <= humStart - HUM_HYSTERESIS) {
      fansRunning = false;
    }

    //
    // "unlock HIGH" latch around the "high" thresholds
    //
    if (temp >= tempHigh || humidity >= humHigh) {
      highUnlocked = true;
    } else if (temp <= tempHigh - TEMP_HYSTERESIS && humidity <= humHigh - HUM_HYSTERESIS) {
      highUnlocked = false;
    }

    if (!fansRunning) {
      return 0;
    }

    unsigned int lowRpm = fanControl.getLowRpm(batteryVoltage);
    unsigned int medRpm = fanControl.getMedRpm(batteryVoltage);

    if (highUnlocked) {
      return fanControl.getHighRpm(batteryVoltage);
    }

    //
    // quiet band: map the stronger of the two demands (0..1) onto LOW..MED RPM
    //
    float demand = max(airingDemand(temp, tempStart, tempHigh),
                       airingDemand(humidity, humStart, humHigh));

    int span = (int) medRpm - (int) lowRpm;
    int rpm  = (int) lowRpm + (int) (demand * span + 0.5);
    return (unsigned int) constrain(rpm, (int) lowRpm, (int) medRpm);
  }
};

extern ControlLoop controlLoop;   // defined in the main sketch

#endif  // CONTROL_LOOP_H
