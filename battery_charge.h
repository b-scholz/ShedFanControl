//      ********************************************************************
//      *                                                                  *
//      *   battery_charge.h  -  Battery state-of-charge estimate: turns   *
//      *                        a resting battery voltage into a charge   *
//      *                        percent, via a runtime-editable curve.    *
//      *                                                                  *
//      *   Public interface (used by FanController.ino/graphs.h):         *
//      *     init()                                                       *
//      *     percentFromVoltage(v)       - SOC estimate                   *
//      *     get/setSocVoltage(i) / getSocPercentLabel(i)                 *
//      *     BATTERY_SOC_TABLE_SIZE      - point count (fixed)            *
//      *                                                                  *
//      *   BatteryCharge translates a voltage into "how full is the
//      *   battery"
//      *                                                                  *
//      ********************************************************************

#ifndef BATTERY_CHARGE_H
#define BATTERY_CHARGE_H

#include <Arduino.h>
#include <ArduinoUserInterface.h>
#include "config.h"
#include "module.h"
#include "battery.h"

//
// Approximate state-of-charge curve for a 12V SLA battery at rest (no charge
// or load current flowing).  Under charge the reading will read high, under
// load it will read low - this is a rough gauge, not a fuel-gauge-accurate
// measurement, since there's no current sensing.  This table is only the
// power-up *default* - see BatteryCharge::socVoltage[] below.
//
struct BatterySocPoint
{
  float voltage;
  byte  percent;
};

const BatterySocPoint BATTERY_SOC_TABLE[] = {
    {12.70, 100}, {12.40, 80}, {12.20, 60}, {12.00, 40}, {11.80, 20}, {11.50, 10}, {11.00, 0},
};

const byte BATTERY_SOC_TABLE_SIZE = sizeof(BATTERY_SOC_TABLE) / sizeof(BATTERY_SOC_TABLE[0]);

//
// BATTERY_SOC_TABLE above is only the power-up default.  The voltage at each
// of its fixed percent points (100/80/60/40/20/10/0%) can be recalibrated at
// runtime from the "Charge curve" menu - see BatteryCharge::setSocVoltage() -
// since every battery (chemistry, age, temperature) actually rests at a
// slightly different voltage for a given charge level. Persisted in EEPROM;
// this is the slider range/step FanController.ino uses for editing each point.
//
const float BATTERY_SOC_VOLTAGE_MIN  = 9.0;
const float BATTERY_SOC_VOLTAGE_MAX  = 15.0;
const float BATTERY_SOC_VOLTAGE_STEP = 0.1;

//
// EEPROM layout - offset from EEPROM_BATTERY_BASE_IDX (config.h), chained
// off battery.h's EEPROM_BATTERY_VOLTS_SCALAR_IDX so the two files' fields
// never collide within the shared block; nothing outside this file needs to
// know this address.
//
// one int per BATTERY_SOC_TABLE point (tenths of a volt)
const int EEPROM_SOC_CURVE_BASE_IDX = EEPROM_BATTERY_VOLTS_SCALAR_IDX + 3; // int[BATTERY_SOC_TABLE_SIZE] (3 each)

class BatteryCharge : public Module
{
public:
  explicit BatteryCharge(ArduinoUserInterface &uiRef) : Module(uiRef) {}

  void init() {
    for (byte i = 0; i < BATTERY_SOC_TABLE_SIZE; i++) {
      int defaultTenths = (int) round(BATTERY_SOC_TABLE[i].voltage * 10.0);
      int tenths        = ui.readConfigurationInt(EEPROM_SOC_CURVE_BASE_IDX + i * 3, defaultTenths);
      socVoltage[i]     = tenths / 10.0;
    }
  }

  //
  // Approximate state-of-charge from voltage, linearly interpolated between
  // the (user-calibrated) resting-voltage breakpoints in socVoltage[] - the
  // percent at each point is the fixed BATTERY_SOC_TABLE[].percent.  Only
  // meaningful with no significant charge/load current flowing - see the
  // caveat in README.md.
  //
  byte percentFromVoltage(float v) const {
    if (v >= socVoltage[0]) {
      return 100;
    }
    for (byte i = 1; i < BATTERY_SOC_TABLE_SIZE; i++) {
      if (v >= socVoltage[i]) {
        float vLo = socVoltage[i], vHi = socVoltage[i - 1];
        byte  pLo = BATTERY_SOC_TABLE[i].percent, pHi = BATTERY_SOC_TABLE[i - 1].percent;
        float frac = (v - vLo) / (vHi - vLo);
        return (byte) (pLo + frac * (pHi - pLo) + 0.5);
      }
    }
    return 0; // at or below the lowest table entry
  }

  float getSocVoltage(byte index) const {
    return socVoltage[index];
  }

  //
  // the fixed percent label for curve point `index` (100/80/60/40/20/10/0) -
  // only the voltage at each point is editable, not this
  //
  byte getSocPercentLabel(byte index) const {
    return BATTERY_SOC_TABLE[index].percent;
  }

  //
  // recalibrate curve point `index`'s voltage and persist it to EEPROM -
  // see "Charge curve" in FanController.ino.
  //
  void setSocVoltage(byte index, float volts) {
    socVoltage[index] = volts;
    ui.writeConfigurationInt(EEPROM_SOC_CURVE_BASE_IDX + index * 3, (int) round(volts * 10.0));
  }

private:
  //
  // runtime-editable copy of BATTERY_SOC_TABLE's voltage breakpoints (the
  // percent side of each point stays fixed; only the voltage each
  // corresponds to is adjustable). Seeded from the config.h defaults, then
  // overridden from EEPROM (if previously saved) in init().
  //
  float socVoltage[BATTERY_SOC_TABLE_SIZE];
};

extern BatteryCharge batteryCharge; // defined in the main sketch

#endif // BATTERY_CHARGE_H
