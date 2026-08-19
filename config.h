//      ******************************************************************
//      *                                                                *
//      *   config.h  -  Constants for app shared across sub-systems     *
//      *                                                                *
//      ******************************************************************

#ifndef FAN_CONTROLLER_CONFIG_H
#define FAN_CONTROLLER_CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------------------------
//                       Control cadence / history resolution
// ---------------------------------------------------------------------------------
//
// We sample, read the tachs and re-evaluate the control law on this interval
// (see ControlLoop::service() in control_loop.h. Constant is given in ms.
// Smaller intervals are not advisable because the DHT sensor can only be
// sampled every 1-2s.
const unsigned long CONTROL_INTERVAL = 2000;

//
// Number of historical data points for temperature, humidity, and voltage.
//
const uint8_t HISTORY_POINTS = 24;

// ---------------------------------------------------------------------------------
//               EEPROM address allocation - one reserved block per file
// ---------------------------------------------------------------------------------
//
// Each subsystem file (display.h, control_loop.h, battery.h,
// battery_charge.h, fans_controller.h) defines its OWN named EEPROM_..._IDX
// constants (using the ArduinoUserInterface library's helpers, which store a
// byte in 2 EEPROM bytes and an int in 3) as offsets from its reserved block
// below, and reads/writes them only through its own getter/setter interface
// - nothing else needs to know a given setting's EEPROM address at all.
//
// The block *bases* are the one thing that has to stay centralized and
// coordinated here: they're spaced out with headroom for each subsystem to
// grow without needing to renumber anything else, which is what guarantees
// no two subsystems' addresses ever collide on the Nano's single shared
// EEPROM. Bump EEPROM_NEXT_FREE_BASE_IDX (and add a new BASE constant above
// it) before allocating a block beyond it.
//
const int EEPROM_LCD_BASE_IDX       = 0;  // display.h     (uses  7 of 16 bytes)
const int EEPROM_CONTROL_BASE_IDX   = 16; // control_loop.h (uses 17 of 32 bytes)
const int EEPROM_BATTERY_BASE_IDX   = 48; // battery.h + battery_charge.h (uses 24 of 40 bytes)
const int EEPROM_FAN_BASE_IDX       = 88; // fans_controller.h (uses 50 of 56 bytes - two PersistedParams, 25 each)
const int EEPROM_NEXT_FREE_BASE_IDX = 144;

#endif // FAN_CONTROLLER_CONFIG_H
