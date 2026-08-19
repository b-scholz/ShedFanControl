//      ********************************************************************
//      *                                                                  *
//      *   battery.h  -  12V SLA battery voltage sensing                  *
//      *                                                                  *
//      *   Public interface:                                              *
//      *     init()                     - initialize battery sensing      *
//      *     read()                     - fresh, oversampled ADC read     *
//      *                                  (ADC times the calibrated       *
//      *                                  volts-per-count scalar)         *
//      *     getVoltage()               - last read()'s result, for       *
//      *                                  display without a fresh ADC     *
//      *                                  read                            *
//      *     get/setScalar()              - the calibrated ADC-counts-to  *
//      *                                  volts scalar (see "Batt         *
//      *                                  calib" in FanController.ino)    *
//      *                                                                  *
//      ********************************************************************

#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>
#include <ArduinoUserInterface.h>
#include "config.h"
#include "module.h"

// ---------------------------------------------------------------------------------
//                       Battery sensing pins and divider
// ---------------------------------------------------------------------------------

// 12V SLA battery voltage sense on pin A7, read through a resistor-divider that
// scales the battery voltage down into the 0-5V ADC range.
const byte BATTERY_ADC_PIN = A7;

//
// Divider: R1 from Vbatt+ down to the ADC node, R2 from the ADC node to GND.
// With 100k/33k, max measurable Vbatt is ~20V (5V / (33/133)) - comfortably
// above any realistic SLA charging voltage (usually <= 14.8V), so normal
// readings stay well inside the ADC's usable range.
//
const float BATTERY_R1 = 100000.0;
const float BATTERY_R2 = 33000.0;

//
// Power-up default for the ADC-counts-to-volts scalar from the divider math.
//
const float BATTERY_VOLTS_SCALAR_DEFAULT = (5.0 / 1023.0) * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;

//
// Number of analogRead() samples averaged into one voltage reading, to quiet
// down ADC noise.
//
const byte BATTERY_ADC_OVERSAMPLE = 8;

//
// "Batt calib" in the menu shows the current live voltage on a slider; drag
// it until it matches a multimeter on the battery terminals, then Set. The
// menu (FanController.ino) rescales voltsScalar proportionally - newScalar =
// oldScalar * targetVoltage / voltageAtSliderOpenTime - so the *same* ADC
// reading now yields exactly the target voltage, and persists the result via
// Battery::setScalar(). Range of the slider around the current reading:
//
const float BATTERY_CALIBRATION_RANGE = 5.0;
const float BATTERY_CALIBRATION_STEP  = 0.1;

//
// Below this (resting) voltage, warn that the battery is getting low - SLA
// batteries are damaged by deep discharge, so this is set well above the
// ~10.5V absolute cutoff.
//
const float BATTERY_LOW_VOLTAGE = 11.5;

//
// EEPROM layout - offset from EEPROM_BATTERY_BASE_IDX (config.h); nothing
// outside this file needs to know this address. BatteryCharge's own
// EEPROM_SOC_CURVE_BASE_IDX (battery_charge.h) chains off this one, so the
// two files' persisted fields never collide within the shared block.
//
const int EEPROM_BATTERY_VOLTS_SCALAR_IDX = EEPROM_BATTERY_BASE_IDX; // int (3) - scalar * 100000

class Battery : public Module
{
public:
  explicit Battery(ArduinoUserInterface &uiRef) : Module(uiRef) {}

  void init() {
    int defaultScaled = (int) round(BATTERY_VOLTS_SCALAR_DEFAULT * 100000.0);
    voltsScalar       = ui.readConfigurationInt(EEPROM_BATTERY_VOLTS_SCALAR_IDX, defaultScaled) / 100000.0;
  }

  //
  // Refresh the battery voltage: a fresh, oversampled ADC read (to quiet
  // down ADC noise) through the resistor divider, times the calibrated
  // volts-per-count scalar - cached for getVoltage() below, the same shape
  // as Sensor::read()/getTemp(). This is what everything outside this class
  // should call every cycle - callers like control_loop.h operate purely in
  // terms of "the battery voltage" and have no reason to know the scalar
  // exists, let alone its value.
  //
  void read() {
    unsigned long sum = 0;
    for (byte i = 0; i < BATTERY_ADC_OVERSAMPLE; i++)
      sum += analogRead(BATTERY_ADC_PIN);

    float avgCount = (float) sum / BATTERY_ADC_OVERSAMPLE;
    voltage        = avgCount * voltsScalar;
  }

  //
  // the most recent calibrated reading (from the last read() call) - for
  // display, without triggering a fresh ADC read
  //
  float getVoltage() const {
    return voltage;
  }

  float getScalar() const {
    return voltsScalar;
  }

  //
  // apply a newly-calibrated scalar and persist it to EEPROM - see "Batt
  // calib" in FanController.ino.
  //
  void setScalar(float scalar) {
    voltsScalar = scalar;
    ui.writeConfigurationInt(EEPROM_BATTERY_VOLTS_SCALAR_IDX, (int) round(scalar * 100000.0));
  }

private:
  float voltage = NAN;

  //
  // calibrated ADC-counts-to-volts scalar, correcting for divider resistor
  // tolerance, the Nano's actual 5V rail, etc. - see setScalar().
  // Defaults to BATTERY_VOLTS_SCALAR_DEFAULT; loaded from EEPROM in init().
  //
  float voltsScalar = BATTERY_VOLTS_SCALAR_DEFAULT;
};

extern Battery battery; // defined in the main sketch

#endif // BATTERY_H
