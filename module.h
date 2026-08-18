//      ******************************************************************
//      *                                                                *
//      *   module.h  -  Shared base for subsystem classes that need the  *
//      *                ArduinoUserInterface instance (EEPROM helpers,   *
//      *                and - for Display - the LCD/slider primitives),  *
//      *                plus a generalized struct-based EEPROM helper.   *
//      *                                                                *
//      *   Battery, BatteryCharge, Fan, FanPIDControl, FansController,    *
//      *   ControlLoop and Display all derive from this; Fan and          *
//      *   FanPIDControl derive from it independently - FanPIDControl     *
//      *   holds its Fan by reference (composition), not inheritance.     *
//      *   Sensor doesn't (it never touches EEPROM or the                *
//      *   LCD, so it has no use for `ui` and stays a plain standalone   *
//      *   class).                                                       *
//      *                                                                 *
//      *   `ui` is `protected`, not `public`: it's shared implementation *
//      *   detail these subsystem classes need internally, not part of   *
//      *   any of their own public interfaces - FanController.ino/graphs.h*
//      *   only ever call each class's own public methods, and            *
//      *   never see this reference at all.                              *
//      *                                                                *
//      ******************************************************************

#ifndef MODULE_H
#define MODULE_H

#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoUserInterface.h>

class Module
{
protected:
  ArduinoUserInterface &ui;

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
  template <typename T>
  void loadState(int eepromBaseIdx, T &state) const
  {
    if (EEPROM.read(eepromBaseIdx) == 0xFF) {
      return;
    }
    EEPROM.get(eepromBaseIdx + 1, state);
  }

  template <typename T>
  void saveState(int eepromBaseIdx, const T &state)
  {
    EEPROM.update(eepromBaseIdx, 1);
    EEPROM.put(eepromBaseIdx + 1, state);
  }
};

#endif  // MODULE_H
