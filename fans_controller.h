//      ******************************************************************
//      *                                                                *
//      *   fans_controller.h  -  Fans Controller: the shed's PAIR-level  *
//      *                     fan control law - drives fanLeft/fanRight   *
//      *                     (fan_pid_control.h) toward the same target  *
//      *                     together, and cross-checks their measured   *
//      *                     RPM for a sustained mismatch fault.         *
//      *                                                                 *
//      *   Everything about ONE fan's own PID/autotune lives in           *
//      *   fan_pid_control.h's FanPIDControl class (which references a    *
//      *   fan.h Fan rather than deriving from it) - this file only       *
//      *   holds what's genuinely about the pair: the shared RPM          *
//      *   targets, the mismatch threshold, and orchestrating two Fan     *
//      *   AND two FanPIDControl references (constructor-injected, same  *
//      *   pattern as ControlLoop's Sensor/Battery/FansController         *
//      *   dependencies). FansController talks to the Fan objects         *
//      *   directly for raw duty/RPM (manual mode, kick-start, stop,      *
//      *   mismatch check) and to the FanPIDControl objects only for      *
//      *   the closed-loop auto path - see the note at the top of         *
//      *   fan_pid_control.h for why there's no pass-through for the      *
//      *   former on FanPIDControl itself.                                *
//      *                                                                 *
//      *   Doesn't init() the Fan/FanPIDControl objects it's given -      *
//      *   fanLeft/fanRight init() themselves, directly in setup()        *
//      *   (FanController.ino), before this class (or anything else)      *
//      *   ever touches them.                                             *
//      *                                                                 *
//      *   Public interface (used by control_loop.h/FanController.ino):    *
//      *     service()            - refresh both fans' measured RPM;     *
//      *                            call once per control cycle, before  *
//      *                            either Drive                         *
//      *     driveAuto(rpm, v)    - PID toward a target RPM (0 = stop),  *
//      *                            both fans, gains compensated for      *
//      *                            live battery voltage v                *
//      *     driveManual(pct, v)  - open-loop duty (0 = stop), both fans; *
//      *                            v is only threaded through to the     *
//      *                            mismatch check's stall judgment       *
//      *                            (fans aren't PID-driven in this mode) *
//      *     isCommandedOn()      - fans currently commanded on          *
//      *     isMismatched()       - sustained left/right fault           *
//      *     getSpeedLabel(buf, v) - human-readable speed label           *
//      *     stopAll()            - stop + reset PID, both fans          *
//      *     get/Low/Med/HighRpm(v) - the pair-level LOW/MED/HIGH RPM      *
//      *                            targets, the lower of both fans' own  *
//      *                            autotune-measured values at v         *
//      *                            (fan_pid_control.h) - see below        *
//      *                                                                 *
//      *   Per-fan readouts (getRpm(), on the raw fanLeftHw/fanRightHw     *
//      *   Fan objects - fan.h), per-fan PID gains (get/setKp/Ki/Kd(),     *
//      *   on fanLeft/fanRight - fan_pid_control.h), and the autotune      *
//      *   primitives (kickstart(), autotuneStep(), isStalled(),           *
//      *   FanPIDControl::computeAutotunePID()) are called directly by     *
//      *   FanController.ino - there's no per-side pass-through wrapper     *
//      *   of any of those here, since duplicating those                   *
//      *   interfaces per side is exactly the duplication this split       *
//      *   removes.                                                       *
//      *                                                                 *
//      ******************************************************************

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
// LOW/MED/HIGH aren't fixed constants here - see getLowRpm()/getMedRpm()/
// getHighRpm() below: each is the *lower* of the two fans' own autotune-
// measured RPM (at FAN_AUTOTUNE_LOW_PCT/HIGH_PCT/MAX_PCT respectively - see
// fan_pid_control.h), so the pair-level target is always something *both*
// fans can actually reach, not just the faster one. "Tune Left"/"Tune
// Right" in the PID tuning menu measures these directly; until a fan's been
// autotuned, its own values fall back to FAN_LOW_RPM_DEFAULT/
// FAN_HIGH_RPM_DEFAULT/FAN_MAX_RPM_DEFAULT (fan_pid_control.h).
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
const unsigned int FAN_RPM_MISMATCH_RPM = 300;
const byte FAN_MISMATCH_DEBOUNCE_CYCLES = 3;   // 3 * CONTROL_INTERVAL = 6s sustained

//
// EEPROM layout - base offsets for each FanPIDControl instance's own
// persisted PID gains (see fan_pid_control.h's PersistedGains and
// Module::loadState()/saveState()), as offsets from EEPROM_FAN_BASE_IDX
// (config.h). Each block holds one struct of 4 floats (16 bytes: Kp/Ki/Kd +
// the tuning-reference voltage) plus 4 unsigned ints (8 bytes: stallRpm/
// lowRpm/highRpm/maxRpm - see fan_pid_control.h) plus Module's 1-byte
// written-flag = 25 of 28 bytes used - nothing outside this file needs to
// know these addresses. Spaced 28 apart (not a tight 25) for a little
// headroom to grow PersistedGains later without needing to widen this
// block again.
//
const int EEPROM_FAN_LEFT_BASE_IDX  = EEPROM_FAN_BASE_IDX;        // PersistedGains (25 of 28 bytes)
const int EEPROM_FAN_RIGHT_BASE_IDX = EEPROM_FAN_BASE_IDX + 28;   // PersistedGains (25 of 28 bytes)


class FansController : public Module
{
public:
  //
  // Enter:  leftFanRef/rightFanRef = each side's raw hardware (fan.h) -
  //         FansController reaches these directly for manual-mode duty,
  //         kick-start, stop, and mismatch-check RPM reads
  //         leftPidRef/rightPidRef = each side's PID (fan_pid_control.h) -
  //         FansController reaches these for the closed-loop auto path.
  //         Both sides must already be init()'d by the caller (setup(),
  //         in FanController.ino) before any other method here is used.
  //
  FansController(ArduinoUserInterface &uiRef, Fan &leftFanRef, Fan &rightFanRef,
                 FanPIDControl &leftPidRef, FanPIDControl &rightPidRef)
    : Module(uiRef), leftFan(leftFanRef), rightFan(rightFanRef),
      leftPid(leftPidRef), rightPid(rightPidRef) {}

  //
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
  // batteryVoltage is this cycle's live battery reading, passed straight
  // through to each FanPIDControl's runPid() so it can compensate its gains
  // for the battery's actual voltage right now (see the note in
  // fan_pid_control.h) - it isn't otherwise used here.
  //
  void driveAuto(unsigned int targetRpm, float batteryVoltage) {
    fanTargetRpm = targetRpm;
    lastDriveManual = false;
    if (fanTargetRpm == 0) {
      //
      // true stop - bypass the PID entirely and reset its state, so the next
      // start (auto or manual) begins from a clean slate rather than picking
      // up stale integral windup from before the fans stopped
      //
      leftFan.setDuty(SPEED_OFF_PCT);
      rightFan.setDuty(SPEED_OFF_PCT);
      leftPid.resetPid();
      rightPid.resetPid();
      fanWasRunning = false;
      commandedOn = false;
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
    const float dt = CONTROL_INTERVAL / 1000.0;   // seconds, for the I/D terms
    leftPid.runPid(fanTargetRpm, dt, batteryVoltage);
    rightPid.runPid(fanTargetRpm, dt, batteryVoltage);
    commandedOn = true;
    updateMismatchCounter(batteryVoltage);
  }

  //
  // Manual/test mode: drive both fans directly to the requested duty
  // percent, bypassing the PID entirely (this is for servicing/diagnostics -
  // e.g. measuring the RPM a given duty produces, same measurement autotune
  // itself now makes automatically - see getLowRpm()/getMedRpm()/
  // getHighRpm() above). Kick-starts out of a standstill exactly
  // as the PID path does. Call once per control cycle while in manual mode
  // (see control_loop.h) - the PID is kept continuously reset so auto mode
  // starts clean whenever it resumes. batteryVoltage isn't used to drive
  // anything here (open-loop, no PID) - it's only threaded through to the
  // mismatch check, so a stalled fan's calibrated threshold (see
  // FanPIDControl::isStalled()) is judged at the right voltage even in
  // manual mode.
  //
  void driveManual(byte dutyPct, float batteryVoltage) {
    fanTargetRpm = 0;
    lastDriveManual = true;
    lastManualPct = dutyPct;
    leftPid.resetPid();
    rightPid.resetPid();
    if (!fanWasRunning && dutyPct > SPEED_OFF_PCT && dutyPct < SPEED_HIGH_PCT) {
      kickstartBoth();
    }
    leftFan.setDuty(dutyPct);
    rightFan.setDuty(dutyPct);
    fanWasRunning = (dutyPct > SPEED_OFF_PCT);
    commandedOn = (dutyPct > SPEED_OFF_PCT);
    updateMismatchCounter(batteryVoltage);
  }

  bool isCommandedOn() const {
      return commandedOn;
  }
  bool isMismatched() const  {
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
  // human-readable name for the current fan speed - entirely self-contained
  // (no caller needs to pass manual-mode state back in). No trailing manual-
  // mode "*" - see drawFanScreen() in FanController.ino. batteryVoltage is
  // only needed to evaluate the LOW/MED/HIGH band edges above at their
  // calibrated voltage - see getLowRpm()/getMedRpm()/getHighRpm().
  //  Enter:  buf = at least 6 bytes, receives the label
  //
  void getSpeedLabel(char *buf, float batteryVoltage) const {
    //
    // manual mode bypasses the PID entirely and has no RPM target, so show
    // the literal commanded duty instead of forcing it into an OFF/LOW/MED/
    // HIGH band that wouldn't mean anything here
    //
    if (lastDriveManual) {
      sprintf(buf, "%d%%", lastManualPct);
      return;
    }
    if (fanTargetRpm == 0) {
        strcpy(buf, "OFF");
        return;
    }
    unsigned int medRpm  = getMedRpm(batteryVoltage);
    unsigned int highRpm = getHighRpm(batteryVoltage);
    unsigned int lowRpm  = getLowRpm(batteryVoltage);
    if (fanTargetRpm >= highRpm) {
        strcpy(buf, "HIGH");
        return;
    }
    if (fanTargetRpm >  medRpm)  {
        strcpy(buf, "MED+");
        return;
    }   // between MED and HIGH (fallback only)
    if (fanTargetRpm >= medRpm)  {
        strcpy(buf, "MED");
        return;
    }
    if (fanTargetRpm >  lowRpm)  {
        strcpy(buf, "LOW+");
        return;
    }   // ramping through the quiet band
    strcpy(buf, "LOW");
  }

  //
  // stop both fans and reset PID state - a clean, known baseline. Used
  // before starting an autotune run (regardless of what was running before
  // the menu was opened) and after it finishes.
  //
  void stopAll() {
    leftFan.setDuty(SPEED_OFF_PCT);
    rightFan.setDuty(SPEED_OFF_PCT);
    leftPid.resetPid();
    rightPid.resetPid();
    fanWasRunning = false;
    commandedOn = false;
  }

private:
  Fan &leftFan;
  Fan &rightFan;
  FanPIDControl &leftPid;
  FanPIDControl &rightPid;

  //
  // current fan output and the control state machine
  //
  unsigned int fanTargetRpm = 0; // last auto-path target RPM (0 = off, or manual is driving)
  bool commandedOn = false;      // true whenever the fans are currently commanded to spin
  bool fanWasRunning = false;    // for kick-start edge detection, shared by auto and manual
  byte mismatchCount = 0;        // consecutive cycles the two fans' RPM has disagreed - see updateMismatchCounter()

  //
  // what the last Drive call was, so getSpeedLabel()/updateMismatchCounter()
  // are self-contained (no caller needs to separately pass "are we in
  // manual mode?" back in)
  //
  bool lastDriveManual = false;
  byte lastManualPct = 0;

  //
  // set both fans to full duty and wait once - unlike Fan::kickstart()
  // (which pulses one fan alone, for autotune), driveAuto()/driveManual()
  // need both fans pulsed *together*, so this doesn't reuse that method.
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
    bool mismatchNow = !lastDriveManual && commandedOn &&
                        !leftPid.isStalled(leftFan.getRpm(), batteryVoltage) &&
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

extern FanPIDControl fanLeft;    // defined in the main sketch
extern FanPIDControl fanRight;   // defined in the main sketch
extern FansController fanControl;   // defined in the main sketch

#endif  // FANS_CONTROLLER_H
