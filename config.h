//      ******************************************************************
//      *                                                                *
//      *   config.h  -  The truly cross-cutting things every subsystem  *
//      *                needs: the shared control cadence/history       *
//      *                resolution below, and the EEPROM address        *
//      *                allocation.                                     *
//      *                                                                *
//      *   CONTROL_INTERVAL and HISTORY_POINTS specifically have to     *
//      *   live here (not with ControlLoop in control_loop.h, which     *
//      *   otherwise owns this cadence/sizing conceptually) because     *
//      *   fan.h/fan_pid_control.h/fans_controller.h/history.h all use them *
//      *   and are included *before* control_loop.h in FanController.ino*
//      *   - this sketch is one big translation unit (headers are       *
//      *   concatenated, not separately compiled), so a constant has to *
//      *   actually be declared before its first use. config.h is the   *
//      *   one header every one of those files already includes first,  *
//      *   so it's the only place that's guaranteed early enough.       *
//      *                                                                *
//      *   Every other piece of tuning (DHT11 pin/timing, LCD/backlight,*
//      *   battery, battery charge curve, fan, climate thresholds,      *
//      *   per-buffer history span) lives with the file that owns it -  *
//      *   see sensor.h, display.h, battery.h, battery_charge.h,        *
//      *   fans_controller.h/fan.h/fan_pid_control.h, and control_loop.h    *
//      *   respectively.                                                *
//      *                                                                *
//      ******************************************************************

#ifndef FAN_CONTROLLER_CONFIG_H
#define FAN_CONTROLLER_CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------------------------
//                       Control cadence / history resolution
// ---------------------------------------------------------------------------------
//
// The DHT11 can only be read about once every 1-2 s.  We sample, read the
// tachs and re-evaluate the control law on this interval (see
// ControlLoop::service() in control_loop.h, which owns everything else about
// this cadence). Constant is given in ms.
//
const unsigned long CONTROL_INTERVAL = 2000;

//
// Shared by all three of ControlLoop's History buffers (temp/humidity/
// battery voltage - see control_loop.h) - simple ring buffers of this many
// points, fed by a running accumulator that averages the raw samples
// falling into each point's time bucket. (Each buffer's own "how far back"
// span - TEMP_HUMID_HISTORY_HOURS / BATTERY_HISTORY_HOURS in control_loop.h
// - determines its own bucket size, but the point *count* is shared here so
// every graph plots at the same resolution against the same RAM budget.)
//
// 84 px of screen width is the real ceiling on useful resolution, and the
// Nano's 2 KB of SRAM is a tighter one still.  24 points over 48h = 2h/point.
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
const int EEPROM_LCD_BASE_IDX      = 0;     // display.h     (uses  7 of 16 bytes)
const int EEPROM_CONTROL_BASE_IDX  = 16;    // control_loop.h (uses 17 of 32 bytes)
const int EEPROM_BATTERY_BASE_IDX  = 48;    // battery.h + battery_charge.h (uses 24 of 40 bytes)
const int EEPROM_FAN_BASE_IDX      = 88;    // fans_controller.h (uses 50 of 56 bytes - two PersistedGains, 25 each)
const int EEPROM_NEXT_FREE_BASE_IDX = 144;

#endif  // FAN_CONTROLLER_CONFIG_H
