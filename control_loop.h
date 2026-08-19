//      ******************************************************************
//      *                                                                *
//      *   control_loop.h  -  Control Loop: manual/auto mode, climate   *
//      *                      thresholds, the automatic target-RPM      *
//      *                      law, and the per-cycle orchestration that *
//      *                      ties sensor/battery/fan control together. *
//      *                                                                *
//      *   Public interface (used by FanController.ino):                *
//      *     init() / service()                                         *
//      *     get/setManualMode() / get/setManualPct()                   *
//      *     previewManualDuty(pct) - live speed while dragging the     *
//      *       manual-speed slider (a no-op unless manual mode is       *
//      *       already on)                                              *
//      *     get/setTempStart() / get/setTempHigh()                     *
//      *     get/setHumStart() / get/setHumHigh()                       *
//      *     getTempHistory() / getHumHistory() / getVoltageHistory()   *
//      *                                                                *
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
// how far back all three graphs (temp/humidity/battery voltage) reach
//
const uint8_t HISTORY_HOURS = 48;

const uint16_t SAMPLES_PER_DATAPOINT =
    (uint16_t) (((uint32_t) HISTORY_HOURS * 60UL * 60UL * 1000UL) / HISTORY_POINTS / CONTROL_INTERVAL);

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
const int DEFAULT_TEMP_START = 20; // start airing at 20 C
const int DEFAULT_TEMP_HIGH  = 32; // full speed at 32 C
const int DEFAULT_HUM_START  = 30; // start airing at 30 %RH
const int DEFAULT_HUM_HIGH   = 50; // full speed at 50 %RH

//
// Hysteresis (deadband): a threshold must be re-crossed by this much before the
// fans step down again, preventing chatter when a reading sits on a threshold.
//
const int TEMP_HYSTERESIS = 2; // degrees C
const int HUM_HYSTERESIS  = 5; // %RH

//
// EEPROM layout - offsets from EEPROM_CONTROL_BASE_IDX (config.h); nothing
// outside this file needs to know these addresses.
//
const int EEPROM_TEMP_START_IDX  = EEPROM_CONTROL_BASE_IDX;    // int  (3)
const int EEPROM_TEMP_HIGH_IDX   = EEPROM_TEMP_START_IDX + 3;  // int  (3)
const int EEPROM_HUM_START_IDX   = EEPROM_TEMP_HIGH_IDX + 3;   // int  (3)
const int EEPROM_HUM_HIGH_IDX    = EEPROM_HUM_START_IDX + 3;   // int  (3)
const int EEPROM_MANUAL_MODE_IDX = EEPROM_HUM_HIGH_IDX + 3;    // byte (2)
const int EEPROM_MANUAL_PCT_IDX  = EEPROM_MANUAL_MODE_IDX + 2; // int  (3)

class ControlLoop : public Module
{
public:
  ControlLoop(ArduinoUserInterface &uiRef, Sensor &sensorRef, Battery &batteryRef, FansController &fanControlRef)
      : Module(uiRef), sensor(sensorRef), battery(batteryRef), fanControl(fanControlRef),
        tempHistory(SAMPLES_PER_DATAPOINT), humHistory(SAMPLES_PER_DATAPOINT), voltageHistory(SAMPLES_PER_DATAPOINT) {}

  void init() {
    tempStart = ui.readConfigurationInt(EEPROM_TEMP_START_IDX, DEFAULT_TEMP_START);
    tempHigh  = ui.readConfigurationInt(EEPROM_TEMP_HIGH_IDX, DEFAULT_TEMP_HIGH);
    humStart  = ui.readConfigurationInt(EEPROM_HUM_START_IDX, DEFAULT_HUM_START);
    humHigh   = ui.readConfigurationInt(EEPROM_HUM_HIGH_IDX, DEFAULT_HUM_HIGH);

    //
    // guard against a blank/corrupt EEPROM producing nonsensical values
    //
    if (tempHigh <= tempStart) {
      tempHigh = tempStart + 1;
    }
    if (humHigh <= humStart) {
      humHigh = humStart + 1;
    }

    manualMode = ui.readConfigurationByte(EEPROM_MANUAL_MODE_IDX, 0) != 0;
    manualPct  = ui.readConfigurationInt(EEPROM_MANUAL_PCT_IDX, 60); // reasonable default test duty
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

  //
  // switch between automatic (sensor-driven) and manual control, and persist
  // the choice
  //
  void setManualMode(bool on) {
    manualMode = on;
    ui.writeConfigurationByte(EEPROM_MANUAL_MODE_IDX, manualMode ? 1 : 0);
  }

  bool getManualMode() const {
    return manualMode;
  }

  //
  // set the duty percent used while in manual mode, and persist it
  //
  void setManualPct(byte pct) {
    manualPct = constrain(pct, 0, 100);
    ui.writeConfigurationInt(EEPROM_MANUAL_PCT_IDX, manualPct);
  }

  byte getManualPct() const {
    return manualPct;
  }

  // live speed preview while the user is dragging the "Fan speed %" slider
  // (see FanController.ino)
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
      tempHigh = tempStart + 1; // keep high > start
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
      tempStart = tempHigh - 1; // keep start < high
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
  const History &getTempHistory() const {
    return tempHistory;
  }
  const History &getHumHistory() const {
    return humHistory;
  }
  const History &getVoltageHistory() const {
    return voltageHistory;
  }

  // -------------------------------------------------------------------------------
  //                     Per-cycle control loop orchestration
  // -------------------------------------------------------------------------------
  //
  // Samples the sensor, reads the tachs and re-evaluates the fan speed
  // once per control interval.
  //
  void service() {
    unsigned long now = millis();
    if (now - lastControl < CONTROL_INTERVAL) {
      return;
    }
    lastControl = now;

    sensor.read();
    fanControl.service();

    // populate temperature and humidity history; if sensor
    // fails populate zero values
    if (sensor.isOk()) {
      tempHistory.addSample((uint8_t) (sensor.getTemp() + 0.5));
      humHistory.addSample((uint8_t) (sensor.getHumidity() + 0.5));
    } else {
      tempHistory.addSample(0);
      humHistory.addSample(0);
    }

    // Rounded to tenths of a volt for the battery voltage history
    battery.read();
    float voltage = battery.getVoltage();
    voltageHistory.addSample((uint8_t) (voltage * 10.0 + 0.5));

    // if the sensor failed, switch to manual mode
    if (!manualMode && !sensor.isOk()) {
      setManualMode(true);
    }

    if (manualMode) {
      // manual override: the user set duty directly,
      fanControl.driveManual((byte) constrain(manualPct, 0, 100), voltage);
    } else {
      unsigned int targetRpm = computeRpm(sensor.getTemp(), sensor.getHumidity(), voltage);
      fanControl.driveAuto(targetRpm, voltage);
    }
  }

private:
  Sensor         &sensor;     // DHT11 temp/humidity source (sensor.h)
  Battery        &battery;    // battery voltage source (battery.h)
  FansController &fanControl; // the fan pair this loop drives (fans_controller.h)

  // Manual or auto mode: if true, manual mode is used to control fans;
  // otherwise the temp/humidity sensor determines when the fans are
  // switched. Loaded from EEPROM in init().
  bool manualMode; // true = ignore sensor, run at manualPct
  int  manualPct;  // fan duty used while in manual mode (0..100 %)

  // active thresholds, loaded from EEPROM at power-up (see config.h for
  // meaning). Only ever changed through setTempStart/High()/setHumStart/High().
  int tempStart;
  int tempHigh;
  int humStart;
  int humHigh;

  // hysteresis latches around the "start"/"high" thresholds - see
  // computeAutoTargetRpm(). Also touched directly by service()'s
  // sensor-fail fallback.
  bool fansRunning  = false; // fans on at all (start-threshold latch)
  bool highUnlocked = false; // allowed to use noisy HIGH speed (high-threshold latch)

  // last time the control law was evaluated
  unsigned long lastControl = 0;

  // the 48h temp/humidity/battery-voltage history buffers for graphs
  History tempHistory;
  History humHistory;
  History voltageHistory;

  // [0,1] work for fans to work: 0.0 at (or below) the
  // start threshold, rising linearly to 1.0 at (or above) the
  // high threshold.
  float airingDemand(float value, int startT, int highT) const {
    if (value <= startT) {
      return 0.0;
    }
    if (value >= highT) {
      return 1.0;
    }
    return (value - startT) / (float) (highT - startT);
  }

  //
  // Returns the target RPM for the current climate
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
  // calibrated voltage - see FansController::getLowRpm()/getMedRpm()/
  // getHighRpm() in fans_controller.h.
  //
  unsigned int computeRpm(float temp, float humidity, float batteryVoltage) {
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
    float demand = max(airingDemand(temp, tempStart, tempHigh), airingDemand(humidity, humStart, humHigh));

    int span = (int) medRpm - (int) lowRpm;
    int rpm  = (int) lowRpm + (int) (demand * span + 0.5);
    return (unsigned int) constrain(rpm, (int) lowRpm, (int) medRpm);
  }
};

extern ControlLoop controlLoop; // defined in the main sketch

#endif // CONTROL_LOOP_H
