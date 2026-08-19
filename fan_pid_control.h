//      **********************************************************************
//      *                                                                    *
//      *   fan_pid_control.h  -  FanPIDControl: closed-loop RPM control     *
//      *             using a PID loop.                                      *
//      *                                                                    *
//      *   Public interface (used by fans_controller.h/FanController.ino):  *
//      *     init()               - the underlying Fan's pin/timer/ISR      *
//      *                            setup, plus loading saved gains         *
//      *     runPid(rpm, dt, v)   - one closed-loop PID step -> duty,       *
//      *                            applied via the Fan's setDuty();        *
//      *                            gains scaled for live voltage v         *
//      *     resetPid()           - clear the integrator/last-error         *
//      *     get/setKp/Ki/Kd(v)   - persisted PID gains; each setter        *
//      *                            takes the live battery voltage v        *
//      *                            (records it as the new tuning           *
//      *                            reference) and saves immediately,       *
//      *                            via Module's struct-based EEPROM        *
//      *                            helpers                                 *
//      *     getTuningVoltage()   - voltage the current gains were last     *
//      *                            set at                                  *
//      *     autotuneStep(pct)    - drive at pct, settle, return RPM        *
//      *     applyAutotuneResults(const AutotuneResult&) - one save of      *
//      *                          everything one "Tune Left"/"Tune Right"   *
//      *                          run measured                              *
//      *     get/isStalled(rpm, v) - this fan's own measured stall RPM,     *
//      *                          scaled by v/tuningVoltage the same way    *
//      *                          runPid() scales the gains (PWM-to-RPM     *
//      *                          scales with voltage too) - NOT a fixed    *
//      *                          constant, since it's calibrated per       *
//      *                          fan by autotune, same as Kp/Ki/Kd         *
//      *     getLowRpm(v)/getHighRpm(v)/getMaxRpm(v) - this fan's own       *
//      *                          measured RPM at the LOW/HIGH/MAX autotune *
//      *                          duty points, scaled by v/tuningVoltage    *
//      *                          the same way isStalled() scales stallRpm; *
//      *                          fans_controller.h combines both fans'     *
//      *                          values into the pair-level LOW/MED/HIGH   *
//      *                          targets                                   *
//      *     static computeAutotunePID(...) - pure math, no instance        *
//      *                          state needed                              *
//      *                                                                    *
//      **********************************************************************

#ifndef FAN_PID_CONTROL_H
#define FAN_PID_CONTROL_H

#include <Arduino.h>
#include <ArduinoUserInterface.h>
#include "config.h"
#include "module.h"
#include "fan.h"

//
// The automatic control law targets a specific RPM, a PID loop reads the
// fan's real tachometer speed and adjusts its PWM duty to hit that target.
//

//
// PID gains. Each FanPIDControl instance has its OWN independent set (a
// left/right RPM mismatch is corrected automatically - each fan gets
// whatever duty it individually needs to hit the same target RPM), editable
// per-fan from the "PID tuning" menu and persisted to EEPROM. The constants
// below are only the power-up *default* for a fan before it's been tuned.
// Error is in RPM, output is a PWM duty percentage (0..100), so these gains
// are tightly coupled to your fans' actual RPM range - *** re-tune (or
// re-autotune) after your fans' LOW/MED/HIGH RPM targets change, e.g. from
// re-autotuning either fan (see fans_controller.h's FansController::getLowRpm()/
// getMedRpm()/getHighRpm()) ***.
//
// Starting point for manual tuning: raise Kp until the fan starts hunting/
// oscillating around the target RPM, then back it off by ~30-50%. Add Ki
// slowly if there's a persistent steady-state RPM error that Kp alone won't
// close. FAN_PID_KD defaults to 0 (a PI controller) as the power-up default
// before any tuning has happened: with RPM only updated once per
// CONTROL_INTERVAL (2s), the derivative term reacts to a very coarse, noisy
// signal, so hand-tuning Kd is easy to get wrong - only raise it if you've
// verified it actually helps. Autotune ("Tune Left"/"Tune Right" in the
// menu - see computeAutotunePID() below) computes a small, conservative Kd
// automatically instead of leaving it at 0.
//
const float FAN_PID_KP = 0.0500;
const float FAN_PID_KI = 0.0100;
const float FAN_PID_KD = 0.0000;

//
// range/step of the Kp/Ki/Kd sliders in the "PID tuning" menu. Kp/Ki are
// capped well above any value autotune or hand-tuning should realistically
// need; Kd's range is smaller since it's rarely worth raising at all here
// (see the note above).
//
const float FAN_PID_KP_MIN  = 0.0;
const float FAN_PID_KP_MAX  = 1.0;
const float FAN_PID_KP_STEP = 0.005;
const float FAN_PID_KI_MIN  = 0.0;
const float FAN_PID_KI_MAX  = 0.5;
const float FAN_PID_KI_STEP = 0.001;
const float FAN_PID_KD_MIN  = 0.0;
const float FAN_PID_KD_MAX  = 0.5;
const float FAN_PID_KD_STEP = 0.001;

//
// Auto-tune ("Tune Left"/"Tune Right" in the menu): the target fan is
// stepped down from FAN_AUTOTUNE_LOW_PCT toward 0 (see FanController.ino's
// autotuneFan()) until it actually stops spinning, each step held for
// FAN_AUTOTUNE_SETTLE_CYCLES control cycles to reach steady RPM (see
// autotuneStep()) - this both measures rpmLow (the first step, at
// FAN_AUTOTUNE_LOW_PCT itself) and finds the fan's *real, measured* stall
// RPM, rather than assuming a fixed constant. HIGH and MAX are then each a
// single fixed-duty test. From the LOW/HIGH (duty, RPM) points,
// computeAutotunePID() computes the fan's process gain
// K = delta-RPM / delta-duty%, then derives Kp = 1/K and Ki = Kp / T (an
// IMC-style choice with the desired closed-loop time constant set equal to
// T = CONTROL_INTERVAL - the fastest this control loop can actually
// respond, and comfortably longer than a PC fan's true mechanical settling
// time, so this assumption holds in practice); Kd = Kp * T / 4, a
// conservative derivative time relative to that same T (see the note on
// FAN_PID_KD above for why this stays a small fraction of it).
//
// HIGH is kept inside a safe middle range (short of 100%) so repeated
// tuning runs aren't needlessly hard on the fan; MAX deliberately probes
// the fan's real ceiling, giving maxRpm - the actual RPM this fan reaches
// at 100% duty, at the voltage the run was calibrated at (see
// applyAutotuneResults() below).
//
const byte FAN_AUTOTUNE_LOW_PCT       = 40;
const byte FAN_AUTOTUNE_HIGH_PCT      = 90;
const byte FAN_AUTOTUNE_MAX_PCT       = 100;
const byte FAN_AUTOTUNE_SETTLE_CYCLES = 4; // 4 * CONTROL_INTERVAL = 8s per step

//
// Power-up defaults for getLowRpm()/getHighRpm()/getMaxRpm() below, before
// this fan has ever been autotuned - same role as FAN_PID_KP/KI/KD above,
// just for RPM instead of gains. fans_controller.h's FansController combines
// both fans' values into the pair-level LOW/MED/HIGH RPM targets, so an
// un-autotuned pair of fans still gets sensible LOW/MED/HIGH targets until
// you run "Tune Left"/"Tune Right".
//
const unsigned int FAN_LOW_RPM_DEFAULT  = 600;
const unsigned int FAN_HIGH_RPM_DEFAULT = 1200;
const unsigned int FAN_MAX_RPM_DEFAULT  = 2000;

//
// The 12V SLA battery this sketch is built around isn't actually constant -
// it swings from ~10.5V (near-empty) to ~14.8V (on charge), and PWM-to-RPM
// for a fixed duty scales roughly linearly with supply voltage, the same way
// a plain DC motor's speed does. So Kp/Ki/Kd tuned (by hand or autotune) at
// one voltage drift out of tune as the battery charges/discharges - runPid()
// compensates by scaling the stored gains by tuningVoltage / (the live
// battery voltage), where tuningVoltage is the voltage recorded at the
// moment those gains were last set (see setKp()/setKi()/setKd() and
// PersistedParams::tuningVoltage below). FAN_GAIN_TUNING_VOLTAGE_DEFAULT is
// only the assumed reference for the untuned power-up default gains, before
// any real measurement exists.
//
const float FAN_GAIN_TUNING_VOLTAGE_DEFAULT = 12.6; // nominal 12V SLA resting voltage

//
// Clamp on the voltage-compensation scale factor itself (tuningVoltage /
// liveVoltage) - normal SLA operation (10.5-14.8V against a ~12.6V tuning
// point) never gets close to this, so it never engages in practice; it only
// exists to keep a bad/glitched voltage reading from swinging the gains to
// something wild rather than just leaving the loop mistuned for one cycle.
//
const float FAN_VOLTAGE_SCALE_MIN = 0.5;
const float FAN_VOLTAGE_SCALE_MAX = 2.0;

//
// everything one "Tune Left"/"Tune Right" run measures, bundled so
// applyAutotuneResults() below takes one clearly-named argument instead of
// eight positional floats/ints that are easy to transpose by mistake at
// the call site (FanController.ino's autotuneFan()).
//
struct AutotuneResult
{
  float        kp;
  float        ki;
  float        kd;
  unsigned int stallRpm;
  unsigned int lowRpm;
  unsigned int highRpm;
  unsigned int maxRpm;
  float        tuningVoltage;
};

//
// this fan's entire persisted state - see Module::loadState()/
// saveState(). Default member initializers double as the power-up
// defaults for a never-written EEPROM offset. stallRpm/lowRpm/highRpm/
// maxRpm default to 0 ("never calibrated") until the first "Tune Left"/
// "Tune Right" run - see isStalled()/getLowRpm()/getHighRpm()/getMaxRpm()
// and applyAutotuneResults() below.
//
struct PersistedParams
{
  float        kp            = FAN_PID_KP;
  float        ki            = FAN_PID_KI;
  float        kd            = FAN_PID_KD;
  float        tuningVoltage = FAN_GAIN_TUNING_VOLTAGE_DEFAULT;
  unsigned int stallRpm      = 0;
  unsigned int lowRpm        = 0;
  unsigned int highRpm       = 0;
  unsigned int maxRpm        = 0;
};

class PIDFanControl : public Module
{
public:
  //
  // Enter:  uiRef           = shared ArduinoUserInterface instance
  //         fanRef          = the Fan (fan.h) this PID drives - referenced,
  //                           not owned; the caller constructs and owns it
  //         eepromBaseIdxIn = base offset this fan's PID gains persist to
  //                           (see Module::loadState()/saveState()) - must
  //                           be different for each FanPIDControl instance
  //
  PIDFanControl(ArduinoUserInterface &uiRef, Fan &fanRef, int eepromBaseIdxIn)
      : Module(uiRef), fan(fanRef), eepromBaseIdx(eepromBaseIdxIn) {}

  void init() {
    fan.init();

    PersistedParams gains;
    loadState(eepromBaseIdx, gains);
    pidKp         = gains.kp;
    pidKi         = gains.ki;
    pidKd         = gains.kd;
    tuningVoltage = gains.tuningVoltage;
    stallRpm      = gains.stallRpm;
    lowRpm        = gains.lowRpm;
    highRpm       = gains.highRpm;
    maxRpm        = gains.maxRpm;
  }

  //
  // clear the PID's memory - call whenever this fan stops, or control
  // switches between manual and automatic, so old error history never leaks
  // into a new run.
  //
  void resetPid() {
    pidIntegral  = 0;
    pidLastError = 0;
  }

  //
  // One closed-loop PID step: error is in RPM, output is a PWM duty percent
  // (0..100) - applied immediately via the underlying Fan's setDuty(), and
  // also returned. Anti-windup: the integral is only updated when doing so
  // wouldn't push the output past the duty range -
  // otherwise a target RPM the fan physically can't reach (or hasn't
  // reached yet, e.g. right after kick-start) would let the integral term
  // wind up unboundedly.
  //
  // pidKp/pidKi/pidKd are only correct at tuningVoltage (see the note on
  // FAN_GAIN_TUNING_VOLTAGE_DEFAULT above) - batteryVoltage is this cycle's
  // live reading, used to scale them back to what they should be right now.
  //
  byte runPid(unsigned int targetRpm, float dt, float batteryVoltage) {
    float voltageScale = 1.0;
    if (batteryVoltage > 0.0) {
      voltageScale = constrain(tuningVoltage / batteryVoltage, FAN_VOLTAGE_SCALE_MIN, FAN_VOLTAGE_SCALE_MAX);
    }

    float kp = pidKp * voltageScale;
    float ki = pidKi * voltageScale;
    float kd = pidKd * voltageScale;

    float error      = (float) targetRpm - (float) fan.getRpm();
    float derivative = (error - pidLastError) / dt;

    float candidateIntegral = pidIntegral + error * dt;
    float output            = kp * error + ki * candidateIntegral + kd * derivative;

    if (output >= 0.0 && output <= 100.0) {
      pidIntegral = candidateIntegral; // safe to accept - not saturating
    }

    pidLastError = error;
    byte dutyOut = (byte) constrain(output, 0.0, 100.0);
    fan.setDuty(dutyOut);
    return dutyOut;
  }

  //
  // getters and setters; values of setters are persisted in EEPROM
  float getKp() const {
    return pidKp;
  }
  float getKi() const {
    return pidKi;
  }
  float getKd() const {
    return pidKd;
  }
  float getTuningVoltage() const {
    return tuningVoltage;
  }

  void setKp(float kp, float voltageNow) {
    pidKp         = kp;
    tuningVoltage = voltageNow;
    savePersistedParams();
  }

  void setKi(float ki, float voltageNow) {
    pidKi         = ki;
    tuningVoltage = voltageNow;
    savePersistedParams();
  }

  void setKd(float kd, float voltageNow) {
    pidKd         = kd;
    tuningVoltage = voltageNow;
    savePersistedParams();
  }

  //
  // is rpmValue below this fan's own measured stall RPM? stallRpm was
  // measured at tuningVoltage (same calibration run as Kp/Ki/Kd - see
  // applyAutotuneResults() below), so - like runPid() - the stored value is
  // scaled by batteryVoltage/tuningVoltage.
  //
  bool isStalled(unsigned int rpmValue, float batteryVoltage) const {
    if (stallRpm == 0) {
      return false;
    }
    return rpmValue < calibratedRpm(stallRpm, batteryVoltage);
  }

  //
  // this fan's own measured RPM at the LOW/HIGH/MAX autotune duty points
  // (FAN_AUTOTUNE_LOW_PCT/HIGH_PCT/MAX_PCT), scaled by
  // batteryVoltage/tuningVoltage the same way isStalled() scales stallRpm.
  //
  unsigned int getLowRpm(float batteryVoltage) const {
    return lowRpm == 0 ? FAN_LOW_RPM_DEFAULT : calibratedRpm(lowRpm, batteryVoltage);
  }
  unsigned int getHighRpm(float batteryVoltage) const {
    return highRpm == 0 ? FAN_HIGH_RPM_DEFAULT : calibratedRpm(highRpm, batteryVoltage);
  }
  unsigned int getMaxRpm(float batteryVoltage) const {
    return maxRpm == 0 ? FAN_MAX_RPM_DEFAULT : calibratedRpm(maxRpm, batteryVoltage);
  }

  void applyAutotuneResults(const AutotuneResult &result) {
    pidKp         = result.kp;
    pidKi         = result.ki;
    pidKd         = result.kd;
    stallRpm      = result.stallRpm;
    lowRpm        = result.lowRpm;
    highRpm       = result.highRpm;
    maxRpm        = result.maxRpm;
    tuningVoltage = result.tuningVoltage;
    savePersistedParams();
  }

  // -------------------------------------------------------------------------------
  //                           Auto-tune primitives
  // -------------------------------------------------------------------------------
  //

  //
  // drive this fan (only) at dutyPct, wait FAN_AUTOTUNE_SETTLE_CYCLES
  // control cycles for it to settle, and return its steady-state RPM
  // (averaged over the last two samples for a bit of noise immunity).
  // Blocks for FAN_AUTOTUNE_SETTLE_CYCLES * CONTROL_INTERVAL (~8s by
  // default).
  //
  unsigned int autotuneStep(byte dutyPct) {
    fan.setDuty(dutyPct);
    unsigned int rpmNow = 0, rpmPrev = 0;
    for (byte cycle = 0; cycle < FAN_AUTOTUNE_SETTLE_CYCLES; cycle++) {
      delay(CONTROL_INTERVAL);
      fan.read();
      rpmPrev = rpmNow;
      rpmNow  = fan.getRpm();
    }
    return (unsigned int) (((unsigned long) rpmNow + rpmPrev) / 2);
  }

  //
  // From a low-duty/high-duty (rpmLow, rpmHigh) measurement pair, compute a
  // fan's process gain K = delta-RPM / delta-duty% and derive Kp = 1/K,
  // Ki = Kp / T, Kd = Kp * T / 4 (T = CONTROL_INTERVAL - see the note on
  // FAN_AUTOTUNE_* above). stallRpm is this same run's freshly-measured
  // stall boundary (from the downward sweep in FanController.ino's
  // autotuneFan(), not a fixed constant) - rpmLow at or below it means the
  // fan was already stalled at the "low" test point, so the pair can't
  // support a sane computation; so does a flat/reversed response, which
  // usually means a wiring or power problem rather than just needing a
  // lower/higher test duty. Pure math, the same for every fan, so it needs
  // no instance state either.
  //
  static bool computeAutotunePID(unsigned int stallRpm, unsigned int rpmLow, unsigned int rpmHigh, float &outKp,
                                 float &outKi, float &outKd) {
    if (rpmLow <= stallRpm || rpmHigh <= rpmLow + 10) {
      return false;
    }

    float processGain =
        (float) (rpmHigh - rpmLow) / (float) (FAN_AUTOTUNE_HIGH_PCT - FAN_AUTOTUNE_LOW_PCT); // RPM per % duty
    float timeConstant = CONTROL_INTERVAL / 1000.0;                                          // seconds

    outKp = 1.0 / processGain;
    outKi = outKp / timeConstant;
    outKd = outKp * timeConstant / 4.0;
    return true;
  }

private:
  //
  // the Fan this PID drives
  //
  Fan &fan;

  //
  // wiring/identity - fixed for this instance's lifetime, passed in by
  // whichever code constructs fanLeft/fanRight (fans_controller.h)
  //
  const int eepromBaseIdx;

  //
  // this fan's PID state and gains - independent per instance.
  // Gains default to FAN_PID_KP/KI/KD above until either hand-tuned via
  // the "PID tuning" menu or set by autotune, at which point they're
  // persisted to EEPROM (loaded back in init()).
  //
  float pidKp = FAN_PID_KP, pidKi = FAN_PID_KI, pidKd = FAN_PID_KD;
  float pidIntegral  = 0;
  float pidLastError = 0;

  //
  // the battery voltage pidKp/Ki/Kd are only correct at - see the note on
  // FAN_GAIN_TUNING_VOLTAGE_DEFAULT and runPid() above. Updated by every
  // setKp()/setKi()/setKd()/applyAutotuneResults() call
  //
  float tuningVoltage = FAN_GAIN_TUNING_VOLTAGE_DEFAULT;

  //
  // this fan's own measured stall RPM, its RPM at the LOW/HIGH/MAX autotune
  // duty points, from the last "Tune Left"/"Tune Right" run - see
  // isStalled()/getLowRpm()/getHighRpm()/getMaxRpm()/applyAutotuneResults()
  // above. 0 = never calibrated.
  //
  unsigned int stallRpm = 0;
  unsigned int lowRpm   = 0;
  unsigned int highRpm  = 0;
  unsigned int maxRpm   = 0;

  //
  // scale a stored RPM (measured at tuningVoltage) to what it should be at
  // batteryVoltage right now - shared by isStalled()/getLowRpm()/
  // getHighRpm()/getMaxRpm(). Same clamp as runPid()'s gain scaling, just
  // not inverted: RPM at a fixed duty scales *up* with voltage, where the
  // gains needed to scale *down* to compensate.
  //
  unsigned int calibratedRpm(unsigned int storedRpm, float batteryVoltage) const {
    float voltageRatio = 1.0;
    if (batteryVoltage > 0.0 && tuningVoltage > 0.0) {
      voltageRatio = constrain(batteryVoltage / tuningVoltage, FAN_VOLTAGE_SCALE_MIN, FAN_VOLTAGE_SCALE_MAX);
    }
    return (unsigned int) ((float) storedRpm * voltageRatio);
  }

  void savePersistedParams() {
    PersistedParams params;
    params.kp            = pidKp;
    params.ki            = pidKi;
    params.kd            = pidKd;
    params.tuningVoltage = tuningVoltage;
    params.stallRpm      = stallRpm;
    params.lowRpm        = lowRpm;
    params.highRpm       = highRpm;
    params.maxRpm        = maxRpm;
    saveState(eepromBaseIdx, params);
  }
};

#endif // FAN_PID_CONTROL_H
