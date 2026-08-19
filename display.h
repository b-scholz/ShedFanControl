//      *********************************************************************
//      *                                                                   *
//      *   display.h  -  Nokia 5110 LCD pins, contrast and backlight       *
//      *                 (on-activity, auto-off) control.                  *
//      *                                                                   *
//      *   Public interface (used by FanController.ino's menu commands     *
//      *   and status loop):                                               *
//      *     init()                       - pins, contrast, backlight      *
//      *     setBacklight(on)/backlightService(ev)/backlightNoteActivity() *
//      *                                   - runtime backlight behavior    *
//      *                                   (not menu-driven)               *
//      *     getContrast()                - current persisted value        *
//      *     previewContrast(value)       - apply without saving           *
//      *     setContrast(value)           - apply and save                 *
//      *     getBacklightTimeout()        - current timeout (s)            *
//      *     setBacklightTimeout(secs)    - set and save                   *
//      *                                                                   *
//      *********************************************************************

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <ArduinoUserInterface.h>
#include "config.h"
#include "module.h"

//
// The Nokia 5110 LCD and the four push buttons (Up/Down/Select/Back on a
// single analog pin through a resistor ladder), with a switched backlight.
// Cf. "Arduino UI Shield" from the ArduinoUserInterface library.
//
const byte LCD_CLOCK_PIN        = A0;
const byte LCD_DATA_IN_PIN      = A1;
const byte LCD_DATA_CONTROL_PIN = A2;
const byte LCD_CHIP_ENABLE_PIN  = A3;

// Button pin (analog signal using a resistor ladder for buttons)
const byte LCD_BUTTONS_PIN = A6;

// LCD Backlight pin
const byte LCD_BACKLIGHT_PIN = A4;

// Backlight signals - negated, see setBacklight() below
const byte LCD_BACKLIGHT_ON  = LOW;
const byte LCD_BACKLIGHT_OFF = HIGH;

//
// The LCD backlight turns on whenever a button is pressed and off again
// after this many seconds of no further button activity - editable at
// runtime from the menu ("Backlight s") and stored in EEPROM.  Only tracked
// while the live status screen is showing (the same as the fan control law
// itself), not while a menu or slider is open.
//
const unsigned int LCD_BACKLIGHT_TIMEOUT_DEFAULT = 10;
const unsigned int LCD_BACKLIGHT_TIMEOUT_MIN     = 1;
const unsigned int LCD_BACKLIGHT_TIMEOUT_MAX     = 60;
const unsigned int LCD_BACKLIGHT_TIMEOUT_STEP    = 5;

const byte CONFIG_CONTRAST_DEFAULT = 60; // matches the on-device contrast set via the one-shot in init()

//
// EEPROM layout - offsets from EEPROM_LCD_BASE_IDX (config.h); nothing
// outside this file needs to know these addresses.
//
const int EEPROM_CONTRAST_IDX = EEPROM_LCD_BASE_IDX; // byte (2)
// bytes [EEPROM_CONTRAST_IDX+2, +2] were the one-shot "halve contrast" flag;
// it already fired and is retired - left unused rather than reused, so a new
// one-shot can never be confused with an old, already-consumed one.
const int EEPROM_SET_CONTRAST_60_ONESHOT_IDX = EEPROM_CONTRAST_IDX + 2 + 2;            // byte (2)
const int EEPROM_BACKLIGHT_TIMEOUT_IDX       = EEPROM_SET_CONTRAST_60_ONESHOT_IDX + 2; // int  (3)

class Display : public Module
{
public:
  explicit Display(ArduinoUserInterface &uiRef) : Module(uiRef) {}

  //
  // LCD bring-up: apply the saved contrast (with a one-time migration to the
  // new default), and start the backlight on with its idle timeout loaded
  // from EEPROM.  Called once from setup(), after ui.connectToPins().
  //
  void init() {
    ui.lcdSetContrast(getContrast());

    Serial.print(F("Contrast at boot: "));
    Serial.println(getContrast());

    //
    // ONE-TIME migration: set contrast to exactly 60, matching the new
    // CONFIG_CONTRAST_DEFAULT.  Guarded by its own EEPROM flag so it only
    // ever fires once, no matter how many times the board resets/power-
    // cycles afterward - a later menu change to contrast is never
    // overwritten by this.
    //
    if (ui.readConfigurationByte(EEPROM_SET_CONTRAST_60_ONESHOT_IDX, 0) == 0) {
      ui.writeConfigurationByte(EEPROM_CONTRAST_IDX, 60);
      ui.writeConfigurationByte(EEPROM_SET_CONTRAST_60_ONESHOT_IDX, 1);
      ui.lcdSetContrast(60);

      Serial.println(F("Contrast set to 60"));
    }

    //
    // backlight on at boot (powering up counts as activity), timer starts
    // now
    //
    pinMode(LCD_BACKLIGHT_PIN, OUTPUT);
    setBacklight(true);
    lastActivity = millis();

    backlightTimeout =
        (unsigned int) ui.readConfigurationInt(EEPROM_BACKLIGHT_TIMEOUT_IDX, LCD_BACKLIGHT_TIMEOUT_DEFAULT);
    backlightTimeout = constrain(backlightTimeout, LCD_BACKLIGHT_TIMEOUT_MIN, LCD_BACKLIGHT_TIMEOUT_MAX);
  }

  //
  // current persisted contrast value, guarded against a corrupt/out-of-range
  // EEPROM value (>127, the LCD's actual usable range)
  //
  byte getContrast() const {
    int value = ui.readConfigurationByte(EEPROM_CONTRAST_IDX, CONFIG_CONTRAST_DEFAULT);
    if (value > 127) {
      value = CONFIG_CONTRAST_DEFAULT;
    }
    return (byte) value;
  }

  //
  // apply a contrast value to the LCD without persisting it - for live
  // preview while a slider is being dragged
  //
  void previewContrast(byte value) {
    ui.lcdSetContrast(value);
  }

  //
  // apply a contrast value and persist it to EEPROM
  //
  void setContrast(byte value) {
    ui.lcdSetContrast(value);
    ui.writeConfigurationByte(EEPROM_CONTRAST_IDX, value);
  }

  unsigned int getBacklightTimeout() const {
    return backlightTimeout;
  }

  void setBacklightTimeout(unsigned int seconds) {
    backlightTimeout = seconds;
    ui.writeConfigurationInt(EEPROM_BACKLIGHT_TIMEOUT_IDX, backlightTimeout);
  }

  //
  // drive LCD_BACKLIGHT_PIN - negated: LCD_BACKLIGHT_ON is LOW. A no-op if
  // the backlight is already in the requested state, so callers can call
  // this unconditionally without spamming digitalWrite().
  //
  void setBacklight(bool on) {
    if (on == backlightOn) {
      return;
    }
    digitalWrite(LCD_BACKLIGHT_PIN, on ? LCD_BACKLIGHT_ON : LCD_BACKLIGHT_OFF);
    backlightOn = on;
  }

  //
  // treat "now" as fresh button activity: turn the backlight on (if it
  // wasn't already) and restart the idle timeout.  Used both for real
  // button events (see backlightService()) and for activity this loop
  // can't otherwise see - e.g. coming back from the menu, where button
  // presses happened but this loop wasn't running to observe them.
  //
  void backlightNoteActivity() {
    setBacklight(true);
    lastActivity = millis();
  }

  //
  // per-loop backlight service: any button activity keeps it on and resets
  // the timeout; otherwise, once the timeout has elapsed, switch it off.
  // Call once per statusScreen() loop iteration with that iteration's
  // button event.
  //
  void backlightService(byte buttonEvent) {
    if (buttonEvent != BUTTON_ID_NONE) {
      backlightNoteActivity();
    } else if (backlightOn && millis() - lastActivity >= (unsigned long) backlightTimeout * 1000UL) {
      setBacklight(false);
    }
  }

private:
  //
  // LCD backlight: on after any button press, off after backlightTimeout
  // seconds of no further button activity. backlightTimeout is loaded from
  // EEPROM in init().
  //
  bool          backlightOn  = true; // current physical backlight state
  unsigned long lastActivity = 0;    // millis() timestamp of the last noted button activity
  unsigned int  backlightTimeout;    // idle seconds before auto-off; loaded from EEPROM in init()
};

extern Display display; // defined in the main sketch

#endif // DISPLAY_H
