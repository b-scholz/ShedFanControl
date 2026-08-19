//      ********************************************************************
//      *                                                                  *
//      *   fans_controller.h  -  Fans Controller: the shed's PAIR-level   *
//      *                     fan control law - drives left and right fan  *
//      *                                                                  *
//      *                                                                  *
//      *   Public interface (used by control_loop.h/FanController.ino):   *
//      *     service()            - refresh both fans' measured RPM;      *
//      *                            call once per control cycle, before   *
//      *                            either Drive                          *
//      *     driveAuto(rpm, v)    - PID toward a target RPM (0 = stop),   *
//      *                            both fans, gains compensated for      *
//      *                            live battery voltage v                *
//      *     driveManual(pct, v)  - open-loop duty (0 = stop), both fans; *
//      *                            v is only threaded through to the     *
//      *                            mismatch check's stall judgment       *
//      *                            (fans aren't PID-driven in this mode) *
//      *     isCommandedOn()      - fans currently commanded on           *
//      *     isMismatched()       - sustained left/right fault            *
//      *     getSpeedLabel(buf, v) - human-readable speed label           *
//      *     stopAll()            - stop + reset PID, both fans           *
//      *     get/Low/Med/HighRpm(v) - the pair-level LOW/MED/HIGH RPM     *
//      *                            targets, the lower of both fans' own  *
//      *                            autotune-measured values at v         *
//      *                            (fan_pid_control.h) - see below       *
//      *                                                                  *
//      ********************************************************************

#ifndef FANS_CONTROLLER_H
#define FANS_CONTROLLER_H

#include <Arduino.h>
#include <ArduinoUserInterface.h>
#include "config.h"
#include "module.h"
#include "fan.h"
#include "fan_pid_control.h"

// ---------------------------------------------------------------------------------
//                          Pair-level RPM targets
// ---------------------------------------------------------------------------------
//
// The controller keeps the fans inside the "quiet band" (LOW..MED RPM) under
// normal conditions and only unlocks HIGH when the temperature or humidity
// reaches its "high" threshold, because HIGH is loud and wears the fans out
// faster. Both fans are always *commanded* to the identical target RPM (see
// driveAuto() below) - a per-fan PID (fan_pid_control.h) then independently
// corrects each fan's own duty to actually hit it, so a left/right mismatch
// (fan wear, manufacturing variance, wiring resistance) is corrected
// automatically rather than assuming identical fans.
//
// Both fans are always *commanded* to the identical target RPM (never
// separate per-fan targets - see driveAuto()), so under normal operation
// their measured speeds converge to match each other, not just the
// setpoint. This only flags when their *measured* RPM nonetheless disagrees
// by more than this much, i.e. a real fault the PID couldn't correct (a
// stalled fan is reported separately via FanPIDControl::isStalled() and
// doesn't also trigger this). Debounced over FAN_MISMATCH_DEBOUNCE_CYCLES consecutive
// control cycles so a transient mismatch while the PID is still settling
// after a target change (e.g. LOW -> HIGH) isn't reported as a fault.
//
const unsigned int FAN_RPM_MISMATCH_RPM         = 300;
const byte         FAN_MISMATCH_DEBOUNCE_CYCLES = 3; // 3 * CONTROL_INTERVAL = 6s sustained

//
// EEPROM layout - base offsets for each FanPIDControl instance's own
// persisted PID gains (see fan_pid_control.h's PersistedParams and
// Module::loadState()/saveState()), as offsets from EEPROM_FAN_BASE_IDX
// (config.h). Each block holds one struct of 4 floats (16 bytes: Kp/Ki/Kd +
// the tuning-reference voltage) plus 4 unsigned ints (8 bytes: stallRpm/
// lowRpm/highRpm/maxRpm - see fan_pid_control.h) plus Module's 1-byte
// written-flag = 25 of 28 bytes used - nothing outside this file needs to
// know these addresses. Spaced 28 apart (not a tight 25) for a little
// headroom to grow PersistedParams later without needing to widen this
// block again.
//
const int EEPROM_FAN_LEFT_BASE_IDX  = EEPROM_FAN_BASE_IDX;      // PersistedParams (25 of 28 bytes)
const int EEPROM_FAN_RIGHT_BASE_IDX = EEPROM_FAN_BASE_IDX + 28; // PersistedParams (25 of 28 bytes)

//
// Compile-time guard against exactly the EEPROM overlap
//
static_assert(
    sizeof(PersistedParams) + 1 <= (unsigned) (EEPROM_FAN_RIGHT_BASE_IDX - EEPROM_FAN_LEFT_BASE_IDX),
    "PersistedParams (+ its 1-byte written-flag) no longer fits the EEPROM_FAN_LEFT/RIGHT_BASE_IDX spacing - widen it");

class FansController : public Module
{
public:
  FansController(ArduinoUserInterface &uiRef, Fan &leftFanRef, Fan &rightFanRef, PIDFanControl &leftPidRef,
                 PIDFanControl &rightPidRef)
      : Module(uiRef), leftFan(leftFanRef), rightFan(rightFanRef), leftPid(leftPidRef), rightPid(rightPidRef) {}

  // refresh both fans' measured RPM. Call once per control cycle, before
  // driveAuto()/driveManual(), so they act on fresh RPM data.
  //
  void service() {
    leftFan.read();
    rightFan.read();
  }

  //
  // Closed-loop fan speed control: drives each fan's PWM duty via its own
  // independent PID so its *actual measured RPM* tracks targetRpm - see the
  // note above for why this is more precise than commanding duty open-loop.
  // Call once per control cycle, right after service() has fresh RPM data.
  //
  void driveAuto(unsigned int targetRpm, float batteryVoltage) {
    fanTargetRpm    = targetRpm;
    lastDriveManual = false;
    if (fanTargetRpm == 0) {
      leftFan.setDuty(SPEED_OFF_PCT);
      rightFan.setDuty(SPEED_OFF_PCT);
      leftPid.resetPid();
      rightPid.resetPid();
      fanWasRunning = false;
      commandedOn   = false;
      updateMismatchCounter(batteryVoltage);
      return;
    }
    if (!fanWasRunning) {
      //
      // starting from a standstill: the PID has no RPM feedback to act on
      // yet this instant, so kick both fans together with a brief open-loop
      // full-power pulse to guarantee spin-up, then let the PID take over
      //
      kickstartBoth();
      leftPid.resetPid();
      rightPid.resetPid();
      fanWasRunning = true;
    }
    const float dt = CONTROL_INTERVAL / 1000.0; // seconds, for the I/D terms
    leftPid.runPid(fanTargetRpm, dt, batteryVoltage);
    rightPid.runPid(fanTargetRpm, dt, batteryVoltage);
    commandedOn = true;
    updateMismatchCounter(batteryVoltage);
  }

  //
  // Manual/test mode: drive both fans directly to the requested duty
  // percent, bypassing the PID entirely (this is for servicing/diagnostics/
  // fail-safe mode in case the sensor is not working)
  void driveManual(byte dutyPct, float batteryVoltage) {
    fanTargetRpm    = 0;
    lastDriveManual = true;
    lastManualPct   = dutyPct;
    leftPid.resetPid();
    rightPid.resetPid();
    if (!fanWasRunning && dutyPct > SPEED_OFF_PCT && dutyPct < SPEED_HIGH_PCT) {
      kickstartBoth();
    }
    leftFan.setDuty(dutyPct);
    rightFan.setDuty(dutyPct);
    fanWasRunning = (dutyPct > SPEED_OFF_PCT);
    commandedOn   = (dutyPct > SPEED_OFF_PCT);
    updateMismatchCounter(batteryVoltage);
  }

  bool isCommandedOn() const {
    return commandedOn;
  }

  //
  // sustained left/right RPM disagreement - see updateMismatchCounter()
  //
  bool isMismatched() const {
    return mismatchCount >= FAN_MISMATCH_DEBOUNCE_CYCLES;
  }

  //
  // the pair-level LOW/MED/HIGH RPM targets - see the note at the top of
  // this file for why each is the *lower* of both fans' own autotune-
  // measured RPM rather than a fixed constant.
  //
  unsigned int getLowRpm(float batteryVoltage) const {
    return min(leftPid.getLowRpm(batteryVoltage), rightPid.getLowRpm(batteryVoltage));
  }
  unsigned int getMedRpm(float batteryVoltage) const {
    return min(leftPid.getHighRpm(batteryVoltage), rightPid.getHighRpm(batteryVoltage));
  }
  unsigned int getHighRpm(float batteryVoltage) const {
    return min(leftPid.getMaxRpm(batteryVoltage), rightPid.getMaxRpm(batteryVoltage));
  }

  //
  // human-readable name for the current fan speed
  void getSpeedLabel(char *buf, float batteryVoltage) const {
    if (lastDriveManual) {
      sprintf(buf, "%d%%", lastManualPct);
      return;
    }

    switch (classifySpeedBand(fanTargetRpm, batteryVoltage)) {
      case SPEED_BAND_OFF:
        strcpy(buf, "OFF");
        break;
      case SPEED_BAND_LOW:
        strcpy(buf, "LOW");
        break;
      case SPEED_BAND_LOW_PLUS:
        strcpy(buf, "LOW+");
        break;
      case SPEED_BAND_MED:
        strcpy(buf, "MED");
        break;
      case SPEED_BAND_MED_PLUS:
        strcpy(buf, "MED+");
        break;
      case SPEED_BAND_HIGH:
        strcpy(buf, "HIGH");
        break;
    }
  }

  //
  // stop both fans and reset PID state
  //
  void stopAll() {
    leftFan.setDuty(SPEED_OFF_PCT);
    rightFan.setDuty(SPEED_OFF_PCT);
    leftPid.resetPid();
    rightPid.resetPid();
    fanWasRunning = false;
    commandedOn   = false;
  }

private:
  Fan           &leftFan;  // left fan's raw hardware (fan.h) - manual duty, kick-start, stop, RPM reads
  Fan           &rightFan; // right fan's raw hardware (fan.h)
  PIDFanControl &leftPid;  // left fan's PID (fan_pid_control.h) - closed-loop auto path
  PIDFanControl &rightPid; // right fan's PID (fan_pid_control.h)

  //
  // current fan output and the control state machine
  //
  unsigned int fanTargetRpm  = 0;     // last auto-path target RPM (0 = off, or manual is driving)
  bool         commandedOn   = false; // true whenever the fans are currently commanded to spin
  bool         fanWasRunning = false; // for kick-start edge detection, shared by auto and manual
  byte         mismatchCount = 0; // consecutive cycles the two fans' RPM has disagreed - see updateMismatchCounter()

  //
  // what the last Drive call was, so getSpeedLabel()/updateMismatchCounter()
  // are self-contained (no caller needs to separately pass "are we in
  // manual mode?" back in)
  //
  bool lastDriveManual = false;
  byte lastManualPct   = 0;

  enum SpeedBand
  {
    SPEED_BAND_OFF,
    SPEED_BAND_LOW,
    SPEED_BAND_LOW_PLUS,
    SPEED_BAND_MED,
    SPEED_BAND_MED_PLUS,
    SPEED_BAND_HIGH
  };

  //
  // classify a target RPM into the OFF/LOW/LOW+/MED/MED+/HIGH bands
  //
  SpeedBand classifySpeedBand(unsigned int targetRpm, float batteryVoltage) const {
    if (targetRpm == 0) {
      return SPEED_BAND_OFF;
    }
    unsigned int lowRpm  = getLowRpm(batteryVoltage);
    unsigned int medRpm  = getMedRpm(batteryVoltage);
    unsigned int highRpm = getHighRpm(batteryVoltage);
    if (targetRpm >= highRpm) {
      return SPEED_BAND_HIGH;
    }
    if (targetRpm > medRpm) {
      return SPEED_BAND_MED_PLUS; // between MED and HIGH (fallback only)
    }
    if (targetRpm >= medRpm) {
      return SPEED_BAND_MED;
    }
    if (targetRpm > lowRpm) {
      return SPEED_BAND_LOW_PLUS; // ramping through the quiet band
    }
    return SPEED_BAND_LOW;
  }

  //
  // set both fans to full duty and wait once
  //
  void kickstartBoth() {
    leftFan.setDuty(SPEED_HIGH_PCT);
    rightFan.setDuty(SPEED_HIGH_PCT);
    delay(FAN_KICKSTART_MS);
  }

  //
  // Both fans are always driven toward the identical target (whether that's
  // driveAuto()'s RPM or driveManual()'s duty) - this tracks whether their
  // *measured* RPM nonetheless disagrees, i.e. a real fault the PID
  // couldn't correct for. Debounced over FAN_MISMATCH_DEBOUNCE_CYCLES so
  // settling after a target change isn't flagged. Manual mode is excluded:
  // it has no RPM feedback (raw duty, identical on both fans), so a
  // difference there is just the fans' natural variance, not a fault.
  // Called automatically at the end of both Drive methods, so callers never
  // need to remember to check mismatch separately. batteryVoltage lets each
  // fan's isStalled() judge against its own calibrated threshold at the
  // live voltage, the same way runPid() compensates the gains.
  //
  void updateMismatchCounter(float batteryVoltage) {
    bool mismatchNow = !lastDriveManual && commandedOn && !leftPid.isStalled(leftFan.getRpm(), batteryVoltage) &&
                       !rightPid.isStalled(rightFan.getRpm(), batteryVoltage) &&
                       (abs((int) leftFan.getRpm() - (int) rightFan.getRpm()) > (int) FAN_RPM_MISMATCH_RPM);

    if (mismatchNow) {
      if (mismatchCount < 255) {
        mismatchCount++;
      }
    } else {
      mismatchCount = 0;
    }
  }
};

extern PIDFanControl  PidFanLeft;    // defined in the main sketch
extern PIDFanControl  PidFanRight;   // defined in the main sketch
extern FansController fanControl; // defined in the main sketch

#endif // FANS_CONTROLLER_H
