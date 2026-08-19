//      ********************************************************************
//      *                                                                  *
//      *   module.h - Shared base for subsystem classes that need the     *
//      *                ArduinoUserInterface instance with EEPROM helpers *
//      ********************************************************************

#ifndef MODULE_H
#define MODULE_H

#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoUserInterface.h>

class Module
{
protected:
  ArduinoUserInterface &ui; // shared UI instance - EEPROM helpers, and LCD/slider calls for Display

  explicit Module(ArduinoUserInterface &uiRef) : ui(uiRef) {}

  //
  // Generic struct-based EEPROM persistence: a module's entire persisted
  // state lives in one plain struct (its member initializers double as the
  // power-up defaults), and loadState()/saveState() move that whole struct
  // to/from EEPROM as a single block at a caller-supplied base offset - no
  // per-field address bookkeeping needed. One raw flag byte at the base
  // offset (not part of T) marks whether that offset has ever been written;
  // on a virgin chip (0xFF) loadState() leaves `state` untouched, i.e. at
  // T's own default values, mirroring the ArduinoUserInterface library's
  // own readConfigurationX()/writeConfigurationX() "unwritten EEPROM reads
  // as 0xFF" convention. Because only the *base offset* needs to be unique
  // (not any individual field), two instances of the same class - e.g.
  // FansController's left/right FanPIDControl - get fully independent persisted
  // state just by being constructed with two different offsets.
  //
  template <typename T> void loadState(int eepromBaseIdx, T &state) const {
    if (EEPROM.read(eepromBaseIdx) == 0xFF) {
      return;
    }
    EEPROM.get(eepromBaseIdx + 1, state);
  }

  template <typename T> void saveState(int eepromBaseIdx, const T &state) {
    EEPROM.update(eepromBaseIdx, 1);
    EEPROM.put(eepromBaseIdx + 1, state);
  }
};

#endif // MODULE_H
