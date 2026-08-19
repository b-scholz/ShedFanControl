//      ******************************************************************
//      *                                                                *
//      *   fan.h  -  Fan: one physical fan's raw PWM + tach hardware -   *
//      *              which pin, which Timer1 compare register, how to   *
//      *              write a duty percent, and turning counted tach     *
//      *              pulses into a measured RPM.
//      *                                                                 *
//      *   Public interface:                                             *
//      *     init()               - this fan's pin/ISR bring-up; the      *
//      *                            shared Timer1 PWM config runs once,   *
//      *                            on the first Fan to init(), however   *
//      *                            many exist                            *
//      *     read()               - refresh measured RPM from the tach    *
//      *                            pulses counted over the last interval*
//      *     getRpm()             - last measured RPM                    *
//      *     setDuty(pct)         - direct open-loop PWM duty (0 = off)  *
//      *     getDuty()            - last duty percent passed to setDuty()*
//      *     kickstart()          - one open-loop full-duty pulse        *
//      *                                                                 *
//      ******************************************************************

#ifndef FAN_H
#define FAN_H

#include <Arduino.h>
#include <ArduinoUserInterface.h>
#include "config.h"
#include "module.h"

//
// Fan PWM outputs (the fan's blue wire) on D9 and D10: both are
// driven by Timer1 reconfigured to 25 kHz (see Fan::init() in
// fan.h).
//
const byte FAN_PWM_LEFT_PIN  = 9;  // OC1A
const byte FAN_PWM_RIGHT_PIN = 10; // OC1B

// Fan tachometer inputs (the fan's green TACH wire).  These MUST be the
// hardware-interrupt pins D2 and D3 on the Nano.
const byte FAN_TACH_LEFT_PIN  = 2; // INT0, ISR channel 0
const byte FAN_TACH_RIGHT_PIN = 3; // INT1, ISR channel 1

//
// 25 kHz on a 16 MHz AVR:  TOP = 16,000,000 / 25,000 - 1 = 639.
// OCR1A/OCR1B range from 0 (0%) to PWM_TOP (100%).
//
const unsigned int PWM_TOP = 639;

//
// How many tach-interrupt channels the ISR-dispatch mechanism below
// supports - a fact about the Nano's hardware (it has exactly two
// external-interrupt-capable pins, INT0=D2 and INT1=D3), not about how many
// Fan objects the application happens to create. The mechanism scales with
// this constant - see the note on tachISR()/tachISRTable below.
//
const byte TACH_ISR_CHANNEL_COUNT = 2;

const byte FAN_TACH_PULSES_PER_REV = 2; // standard for PC fans

//
// Duty range bounds - off and full duty come up often enough (manual mode,
// stop, kick-start) to be worth naming rather than writing 0/100 inline.
//
const byte SPEED_OFF_PCT  = 0;
const byte SPEED_HIGH_PCT = 100;

//
// Starting from a standstill, briefly pulse full duty to guarantee spin-up
// before settling at a lower target speed - see kickstart() below.
//
const unsigned int FAN_KICKSTART_MS = 300;

class Fan : public Module
{
public:
  //
  // Enter:  uiRef        = shared ArduinoUserInterface instance
  //         pwmPinIn     = this fan's PWM output pin (D9 or D10)
  //         ocrRegIn     = &OCR1A or &OCR1B - this fan's Timer1 compare
  //                        register, i.e. which of the two PWM channels
  //                        this instance drives
  //         tachPinIn    = this fan's tachometer input pin (D2 or D3)
  //         isrChannelIn = 0 or 1 - which entry of tachISRTable (see
  //                        below) this fan's tach interrupt uses; must be
  //                        < TACH_ISR_CHANNEL_COUNT
  //
  Fan(ArduinoUserInterface &uiRef, byte pwmPinIn, volatile uint16_t *ocrRegIn, byte tachPinIn, byte isrChannelIn)
      : Module(uiRef), pwmPin(pwmPinIn), ocrReg(ocrRegIn), tachPin(tachPinIn), isrChannel(isrChannelIn) {}

  //
  // this fan's PWM/tach pin bring-up. The shared Timer1 PWM mode config (25
  // kHz fast PWM, both OC1A/OC1B channels - both fans' duty share one timer)
  // is hardware, not per-fan state - a singleton flag guards it so it only
  // actually runs once, on the first Fan instance to init(), however many
  // exist.
  //
  void init() {
    if (!timer1Configured) {
      setupTimer1();
      timer1Configured = true;
    }

    pinMode(pwmPin, OUTPUT);
    *ocrReg = 0;

    pinMode(tachPin, INPUT_PULLUP);
    instances[isrChannel] = this;
    attachInterrupt(digitalPinToInterrupt(tachPin), tachISRTable[isrChannel], FALLING);
  }

  //
  // write a duty percent (0..100) straight to this fan's PWM output by
  // loading its Timer1 compare register - open-loop, no feedback.
  //
  void setDuty(byte pct) {
    duty    = pct;
    *ocrReg = (unsigned int) ((unsigned long) pct * PWM_TOP / 100);
  }

  //
  // last duty percent passed to setDuty() - this instance's own record, not
  // read back from the hardware.
  //
  byte getDuty() const {
    return duty;
  }

  //
  // refresh measured RPM from the tach pulses counted over the last
  // interval. Call once per control cycle - getRpm() right after this
  // returns fresh data.
  //
  void read() {
    unsigned int c = readTachCount();

    //
    // RPM = pulses / pulsesPerRev / (interval[s]) * 60
    //
    unsigned long k = 60000UL / (CONTROL_INTERVAL * FAN_TACH_PULSES_PER_REV);
    rpm             = (unsigned int) (c * k);
  }

  unsigned int getRpm() const {
    return rpm;
  }

  //
  // open-loop kick-start pulse (~300ms at full duty) - guarantees spin-up
  // out of a standstill before settling at a lower target speed/duty, since
  // there's no RPM feedback to act on yet at this instant.
  //
  void kickstart() {
    setDuty(SPEED_HIGH_PCT);
    delay(FAN_KICKSTART_MS);
  }

private:
  const byte               pwmPin;     // this fan's PWM output pin (D9 or D10)
  volatile uint16_t *const ocrReg;     // this fan's Timer1 compare register (&OCR1A or &OCR1B)
  const byte               tachPin;    // this fan's tachometer input pin (D2 or D3)
  const byte               isrChannel; // this fan's index into instances[]/tachISRTable below

  byte                  duty      = 0; // last duty percent passed to setDuty()
  volatile unsigned int tachCount = 0; // tach pulses counted since the last readTachCount()
  unsigned int          rpm       = 0; // last RPM computed by read()

  static bool timer1Configured; // guards the shared Timer1 PWM setup to run only once

  //
  // read and clear the tach pulse count accumulated since the last call -
  // read() above turns this into an RPM figure.
  //
  unsigned int readTachCount() {
    noInterrupts();
    unsigned int c = tachCount;
    tachCount      = 0;
    interrupts();
    return c;
  }

  //
  // Fast PWM, TOP = ICR1 (WGM13:10 = 1110), non-inverting on A and B,
  // prescaler = 1.  Frequency = 16 MHz / (PWM_TOP + 1) = 25 kHz.
  //
  static void setupTimer1() {
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    ICR1   = PWM_TOP;
  }

  //
  // attachInterrupt() needs a plain function pointer, which a non-static
  // member function isn't (it has an implicit `this`). Rather than hand-
  // writing one trampoline function per fan (baking a "there are exactly
  // two fans" assumption into this class), tachISR<CHANNEL>() is a single
  // template generating one trampoline per channel actually referenced
  // below - the mechanism scales with TACH_ISR_CHANNEL_COUNT (a hardware
  // fact), not with how many Fan objects happen to exist. tachISRTable
  // turns that into a plain runtime-indexable lookup, since isrChannel
  // itself is only known at construction, not compile time.
  //
  static Fan *instances[TACH_ISR_CHANNEL_COUNT];

  template <byte CHANNEL> static void tachISR() {
    if (instances[CHANNEL]) {
      instances[CHANNEL]->tachCount++;
    }
  }

  typedef void (*TachISRFunc)();
  static const TachISRFunc tachISRTable[TACH_ISR_CHANNEL_COUNT];
};

bool Fan::timer1Configured = false;

Fan *Fan::instances[TACH_ISR_CHANNEL_COUNT] = {nullptr, nullptr};

const Fan::TachISRFunc Fan::tachISRTable[TACH_ISR_CHANNEL_COUNT] = {
    Fan::tachISR<0>,
    Fan::tachISR<1>,
};

extern Fan fanLeft;  // defined in the main sketch
extern Fan fanRight; // defined in the main sketch

#endif // FAN_H
