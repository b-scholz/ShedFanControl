//      ******************************************************************
//      *                                                                *
//      *                    Shed Fan Controller                         *
//      *                                                                *
//      *   Airs out a garden shed by driving two 4-wire PC fans (left   *
//      *   and right) whenever it gets too warm or too damp inside.     *
//      *                                                                *
//      *   Hardware:  Arduino Uno/Nano  +  DHT11 (temp/humidity)        *
//      *              +  Nokia 5110 LCD  +  4 buttons (Arduino UI       *
//      *              Shield layout)  +  2 four-wire PWM fans.          *
//      *                                                                *
//      *   Display / menus / buttons use Stan Reifel's                  *
//      *   "ArduinoUserInterface" library:                              *
//      *      https://github.com/Stan-Reifel/ArduinoUserInterface       *
//      *                                                                *
//      *   Fan drive                                                    *
//      *   ---------                                                    *
//      *   4-wire fans are powered continuously from +12V; their speed  *
//      *   is set on the PWM (blue) wire with a 25 kHz signal generated *
//      *   by Timer1 on pins D9/D10.  The TACH wire is counted to show  *
//      *   real RPM and to catch a stalled/failed fan.                  *
//      *                                                                *
//      *   Control philosophy                                           *
//      *   ------------------                                           *
//      *   The fans are regulated from LOW to HIGH RPM. The control     *
//      *   law picks a target RPM, not a PWM duty cycle - a per-fan PID *
//      *   (FansController::driveAuto()) reads each fan's real tachometer*
//      *   speed and adjusts its own duty to hit that target, closed-   *
//      *   loop.                                                        *
//      *                                                                *
//      *   Code layout                                                  *
//      *   -----------                                                  *
//      *   Each subsystem is its own C++ class, in its own file, owning *
//      *   its own state and EEPROM load:                               *
//      *     Sensor      (sensor.h)      - Temp/Humid Sensor (DHT11)     *
//      *     Battery     (battery.h)     - Battery Reading (voltage,    *
//      *                                  calibration, charge curve)    *
//      *     Fan          (fan.h)        - ONE physical fan's raw PWM     *
//      *                                  + tach hardware: pin, OCR1       *
//      *                                  register, setDuty()/getRpm().    *
//      *                                  No PID/EEPROM at all.            *
//      *     FanPIDControl(fan_pid_control.h) - closed-loop PID on top of  *
//      *                                  a Fan it holds *by reference*    *
//      *                                  (composition, not inheritance)  *
//      *                                  - adds gains, autotune. Two      *
//      *                                  instances, fanLeft/fanRight,     *
//      *                                  below - all per-fan state is a  *
//      *                                  constructor param or private    *
//      *                                  member, so there's no Left/     *
//      *                                  Right-suffixed duplication.     *
//      *     FansController(fans_controller.h) - PAIR-level control law  *
//      *                                  over fanLeft/fanRight: drives   *
//      *                                  both to the same target        *
//      *                                  RPM/duty, cross-checks them     *
//      *                                  for a sustained mismatch fault  *
//      *     ControlLoop (control_loop.h)- manual/auto mode, climate    *
//      *                                  thresholds, and the per-cycle *
//      *                                  logic that drives Sensor/      *
//      *                                  Battery/FansController every    *
//      *                                  cycle                          *
//      *     Display     (display.h)     - Nokia 5110 LCD pins,         *
//      *                                  contrast, backlight           *
//      *   Every class above keeps its state in `private` members,       *
//      *   exposing only `public` getter/setter/action methods - none    *
//      *   have any menu/ui.displaySlider() code of their own. Battery/  *
//      *   Fan/FanPIDControl/FansController/ControlLoop/Display share a   *
//      *   `protected ui` reference (module.h's Module base class) for   *
//      *   EEPROM access; Sensor never touches EEPROM, so it's a plain   *
//      *   standalone class. ControlLoop takes its Sensor/Battery/       *
//      *   FansController dependencies (and FansController its two Fan&  *
//      *   plus two FanPIDControl&) as constructor references rather than *
//      *   reaching for globals - likewise each FanPIDControl takes its   *
//      *   own Fan& (composition, not inheritance - see fan_pid_control.h*
//      *   for why). Module also provides a generalized struct-based      *
//      *   EEPROM helper (loadState()/saveState()) - a module's whole    *
//      *   persisted state is one POD struct, saved/loaded as a block    *
//      *   at a given base offset, which is how fanLeft/fanRight each    *
//      *   get independent persisted PID gains from the same             *
//      *   FanPIDControl class without hand-numbered per-field EEPROM     *
//      *   addresses per side.                                           *
//      *                                                                *
//      *   The live status screens and every menu command/tree live      *
//      *   directly in this file (below the singleton block, ahead of    *
//      *   setup()/loop()) rather than in their own files - free          *
//      *   functions, not a class, since the ArduinoUserInterface         *
//      *   library's MENU_ITEM table needs plain C-compatible function    *
//      *   pointers, which non-static member functions aren't. They      *
//      *   talk to the subsystem objects above only through their        *
//      *   public methods, never internal state directly - that's what   *
//      *   keeps this section free to change how the menu is organized   *
//      *   without needing to know how (or whether) any given setting    *
//      *   is stored. graphs.h's Graph class does the actual plotting;    *
//      *   the screens here just assemble ControlLoop's history buffers   *
//      *   onto it.                                                       *
//      *                                                                *
//      ******************************************************************

// https://github.com/Stan-Reifel/ArduinoUserInterface
#include <ArduinoUserInterface.h>
#include "config.h"
#include "sensor.h"
#include "battery.h"
#include "battery_charge.h"
#include "fan.h"
#include "fan_pid_control.h"
#include "fans_controller.h"
#include "control_loop.h"
#include "display.h"
#include "graphs.h"

//
// the one instance of each subsystem - declared in this order because
// ControlLoop's constructor takes references to sensor/battery/fanControl,
// so they must already be declared (their storage/addresses are fixed
// before any constructor runs, but keeping this order is simply clearer to
// read). Every other file reaches these only through the `extern` in each
// one's own header - never a redeclaration of internal state.
//
ArduinoUserInterface ui;
Sensor               sensor;
Battery              battery(ui);
BatteryCharge        batteryCharge(ui);
Fan                  fanLeft(ui, FAN_PWM_LEFT_PIN, &OCR1A, FAN_TACH_LEFT_PIN, 0);
Fan                  fanRight(ui, FAN_PWM_RIGHT_PIN, &OCR1B, FAN_TACH_RIGHT_PIN, 1);
PIDFanControl        PidFanLeft(ui, fanLeft, EEPROM_FAN_LEFT_BASE_IDX);
PIDFanControl        PidFanRight(ui, fanRight, EEPROM_FAN_RIGHT_BASE_IDX);
FansController       fanControl(ui, fanLeft, fanRight, PidFanLeft, PidFanRight);
Display              display(ui);
ControlLoop          controlLoop(ui, sensor, battery, fanControl);

//
// the four graph specializations (graphs.h) - title + label formatter (+
// transform for charge) each; no cross-object dependencies beyond
// batteryCharge (already declared above) and the formatter functions
// graphs.h itself defines.
//
Graph tempGraph("Temp 48h", graphFormatPlain);
Graph humidityGraph("Humid 48h", graphFormatPlain);
Graph batteryGraph("Battery 48h", graphFormatVolts);
Graph chargeGraph("Charge 48h", graphFormatPlain, graphChargeFromVoltsTenths);

// ---------------------------------------------------------------------------------
//                          Live status ("home") screens
// ---------------------------------------------------------------------------------
//
// Dispatch, and the Climate and Fans screen content. The Battery/Charge/
// Temperature/Humidity graph screens live in graphs.h - drawMainScreen()
// below just dispatches to them.
//

//
// which main/home screen is showing, cycled with the bottom-left (Select)
// button; Back always opens the menu regardless of which one is active
//
enum MainScreen
{
  MAIN_SCREEN_CLIMATE        = 0, // temperature, humidity, battery
  MAIN_SCREEN_BATTERY_GRAPH  = 1, // 48h battery voltage graph
  MAIN_SCREEN_CHARGE_GRAPH   = 2, // 48h battery charge (%) graph
  MAIN_SCREEN_TEMP_GRAPH     = 3, // 48h temperature graph
  MAIN_SCREEN_HUMIDITY_GRAPH = 4, // 48h humidity graph
  MAIN_SCREEN_FAN            = 5  // fan speed + each fan's RPM
};
const byte  NUM_MAIN_SCREENS = 6;
static byte mainScreenIndex  = MAIN_SCREEN_CLIMATE;

void drawClimateScreen(); // forward decl - defined below, used by drawMainScreen()
void drawFanScreen();     // forward decl - defined below, used by drawMainScreen()

//
// draw whichever main screen is currently selected.  On a full redraw the
// display is cleared and a uniform "Next"/"Menu" button bar is drawn (Select
// always cycles screens here, unlike the same graph content's standalone
// menu screen where Select means something screen-specific); periodic
// in-place refreshes skip both, so the same screen's readout can update
// without flickering.
//
void drawMainScreen(bool fullRedraw) {
  if (fullRedraw) {
    ui.lcdClearDisplay();
  }

  switch (mainScreenIndex) {
    case MAIN_SCREEN_CLIMATE:
      drawClimateScreen();
      break;
    case MAIN_SCREEN_BATTERY_GRAPH:
      batteryGraph.draw(controlLoop.getVoltageHistory());
      break;
    case MAIN_SCREEN_CHARGE_GRAPH:
      chargeGraph.draw(controlLoop.getVoltageHistory());
      break;
    case MAIN_SCREEN_TEMP_GRAPH:
      tempGraph.draw(controlLoop.getTempHistory());
      break;
    case MAIN_SCREEN_HUMIDITY_GRAPH:
      humidityGraph.draw(controlLoop.getHumHistory());
      break;
    case MAIN_SCREEN_FAN:
      drawFanScreen();
      break;
  }

  if (fullRedraw) {
    ui.drawButtonBar("Next", "Menu");
  }
}

//
// print one row's fixed-width label (left-justified at column 0) and value
// (right-justified against the last column) - the shape every row of the
// Climate and Fans screens shares below; only the row, label, value, and
// value's field width change between calls. width should be the longest
// that field can ever be, so a shorter new reading always blanks out
// whatever longer text was there before - see
// lcdPrintStringRightJustified()'s padToNumberOfCharacters parameter.
//
void printLabeledValue(byte row, const char *label, const char *value, byte width) {
  ui.lcdSetCursorXY(0, row);
  ui.lcdPrintString(label);
  ui.lcdSetCursorXY(LCD_LAST_COLUMN_X, row);
  ui.lcdPrintStringRightJustified(value, width);
}

//
// main screen 1/6: temperature, humidity, battery voltage and charge status
// (fan status lives only on the dedicated Fans screen)
//
void drawClimateScreen() {
  char buf[12];

  //
  // line 0: a sensor warning, which needs the user's attention, or else a
  // static title
  //
  ui.lcdSetCursorXY(LCD_WIDTH_IN_PIXELS / 2, 0);
  ui.lcdPrintStringCentered(sensor.isOk() ? "Shed airing" : "SENSOR FAIL", 11);

  //
  // line 1: temperature
  //
  if (sensor.isOk()) {
    dtostrf(sensor.getTemp(), 1, 1, buf);
    strcat(buf, "C");
  } else {
    strcpy(buf, "--");
  }
  printLabeledValue(1, "Temp:", buf, 6); // widest: "50.0C"

  //
  // line 2: humidity
  //
  if (sensor.isOk()) {
    dtostrf(sensor.getHumidity(), 1, 1, buf);
    strcat(buf, "%");
  } else {
    strcpy(buf, "--");
  }
  printLabeledValue(2, "Humid:", buf, 6); // widest: "100.0%"

  //
  // line 3: battery voltage, or a LOW warning below BATTERY_LOW_VOLTAGE -
  // SLA batteries are damaged by deep discharge, so this stays visible on
  // every visit to this screen rather than being tucked away in a menu
  //
  float voltage = battery.getVoltage();
  if (isnan(voltage)) {
    strcpy(buf, "--");
  } else if (voltage < BATTERY_LOW_VOLTAGE) {
    strcpy(buf, "LOW!");
  } else {
    dtostrf(voltage, 1, 1, buf);
    strcat(buf, "V");
  }
  printLabeledValue(3, "Batt:", buf, 6); // widest: "20.4V" / "LOW!"

  //
  // line 4: battery charge status (resting-voltage estimate - see the
  // caveat in README.md). Fan status now lives only on the Fans screen.
  //
  if (isnan(voltage)) {
    strcpy(buf, "--");
  } else {
    sprintf(buf, "%d%%", batteryCharge.percentFromVoltage(voltage));
  }
  printLabeledValue(4, "Charge:", buf, 4); // widest: "100%"
}

//
// main screen 6/6: fan speed and each fan's individual RPM ("STALL" for a
// fan that's commanded on but not spinning fast enough). The pair-level bits
// (mismatch, combined speed label, commanded-on) come from fanControl's
// public methods; each fan's own measured RPM comes straight from the raw
// fanLeftHw/fanRightHw Fan objects (fan.h) - this screen never reaches into
// any object's private state.
//
void drawFanScreen() {
  char buf[12];

  ui.lcdSetCursorXY(LCD_WIDTH_IN_PIXELS / 2, 0);
  ui.lcdPrintStringCentered(fanControl.isMismatched() ? "MISMATCH!" : "Fans", 9);

  float voltage = battery.getVoltage();

  char fanLabel[8];
  fanControl.getSpeedLabel(fanLabel, voltage);
  if (controlLoop.getManualMode()) {
    strcat(fanLabel, "*");
  }
  ui.lcdSetCursorXY(LCD_WIDTH_IN_PIXELS / 2, 1);
  ui.lcdPrintStringCentered(fanLabel, 5); // widest: "HIGH*" / "100%*"

  bool         running  = fanControl.isCommandedOn();
  unsigned int leftRpm  = fanLeft.getRpm();
  unsigned int rightRpm = fanRight.getRpm();

  if (running && PidFanLeft.isStalled(leftRpm, voltage)) {
    strcpy(buf, "STALL");
  } else {
    itoa(running ? leftRpm : 0, buf, 10);
  }
  printLabeledValue(3, "Left:", buf, 5); // widest: "STALL" / 4-digit RPM

  if (running && PidFanRight.isStalled(rightRpm, voltage)) {
    strcpy(buf, "STALL");
  } else {
    itoa(running ? rightRpm : 0, buf, 10);
  }
  printLabeledValue(4, "Right:", buf, 5);
}

// ---------------------------------------------------------------------------------
//                              Menu commands
// ---------------------------------------------------------------------------------
//
// No subsystem class (ControlLoop/Battery/BatteryCharge/Display/FansController)
// has any ui.displaySlider()/menu code of its own - each instead exposes a
// small public getter/setter interface, and every command below talks to
// the global controlLoop/battery/batteryCharge/display/fanControl objects
// ONLY through those public methods, never their private state directly.
// That's what keeps this section free to change how the menu is organized
// without needing to know how (or whether) any given setting is stored.
//

// ---------------------------------------------------------------------------------
//                       Menu commands: threshold sliders
// ---------------------------------------------------------------------------------
//
// each menuCommandSetX() references setXCallback() before its own definition
// further down this file - forward declare them all up front.
//
void setTempStartCallback(byte op, int value);
void setTempHighCallback(byte op, int value);
void setHumStartCallback(byte op, int value);
void setHumHighCallback(byte op, int value);

void menuCommandSetTempStart() {
  ui.displaySlider(10, 40, 1, controlLoop.getTempStart(), "Temp start C", setTempStartCallback);
}
void setTempStartCallback(byte op, int value) {
  if (op == SLIDER_DISPLAY_VALUE_SET) {
    controlLoop.setTempStart(value);
  }
}

void menuCommandSetTempHigh() {
  ui.displaySlider(11, 45, 1, controlLoop.getTempHigh(), "Temp high C", setTempHighCallback);
}
void setTempHighCallback(byte op, int value) {
  if (op == SLIDER_DISPLAY_VALUE_SET) {
    controlLoop.setTempHigh(value);
  }
}

void menuCommandSetHumStart() {
  ui.displaySlider(30, 95, 1, controlLoop.getHumStart(), "Humid start%", setHumStartCallback);
}
void setHumStartCallback(byte op, int value) {
  if (op == SLIDER_DISPLAY_VALUE_SET) {
    controlLoop.setHumStart(value);
  }
}

void menuCommandSetHumHigh() {
  ui.displaySlider(31, 100, 1, controlLoop.getHumHigh(), "Humid high %", setHumHighCallback);
}
void setHumHighCallback(byte op, int value) {
  if (op == SLIDER_DISPLAY_VALUE_SET) {
    controlLoop.setHumHigh(value);
  }
}

// ---------------------------------------------------------------------------------
//                     Menu commands: manual override
// ---------------------------------------------------------------------------------

//
// toggle automatic <-> manual control (state/persistence entirely owned by
// controlLoop)
//
void menuToggleManualCallback() {
  if (ui.toggleMenuChangeStateFlag) {
    controlLoop.setManualMode(!controlLoop.getManualMode());
  }
  ui.toggleMenuStateText = controlLoop.getManualMode() ? "On" : "Off";
}

//
// manual fan speed as a percentage
//
void setManualSpeedCallback(byte op, int value); // forward decl - defined below

void menuCommandSetManualSpeed() {
  ui.displaySlider(0, 100, 5, controlLoop.getManualPct(), "Fan speed %", setManualSpeedCallback);
}
void setManualSpeedCallback(byte op, int value) {
  //
  // preview the speed live while dragging so it can be tested by ear...
  //
  if (op == SLIDER_DISPLAY_VALUE_CHANGED) {
    controlLoop.previewManualDuty((byte) constrain(value, 0, 100));
  }
  //
  // ...and store it when confirmed
  //
  else if (op == SLIDER_DISPLAY_VALUE_SET) {
    controlLoop.setManualPct((byte) constrain(value, 0, 100));
  }
}

// ---------------------------------------------------------------------------------
//                  Menu commands: contrast & backlight timeout
// ---------------------------------------------------------------------------------

void setContrastCallback(byte op, int value); // forward decl - defined below

void menuCommandSetContrast() {
  ui.displaySlider(1, 127, 1, display.getContrast(), "Set contrast", setContrastCallback);
}
void setContrastCallback(byte op, int value) {
  switch (op) {
    case SLIDER_DISPLAY_VALUE_CHANGED:
      display.previewContrast((byte) value);
      break;
    case SLIDER_DISPLAY_VALUE_SET:
      display.setContrast((byte) value);
      break;
    case SLIDER_DISPLAY_CANCELED:
      display.previewContrast(display.getContrast());
      break;
  }
}

void setBacklightTimeoutCallback(byte op, int value); // forward decl - defined below

void menuCommandSetBacklightTimeout() {
  ui.displaySlider(LCD_BACKLIGHT_TIMEOUT_MIN, LCD_BACKLIGHT_TIMEOUT_MAX, LCD_BACKLIGHT_TIMEOUT_STEP,
                   display.getBacklightTimeout(), "Backlight s", setBacklightTimeoutCallback);
}
void setBacklightTimeoutCallback(byte op, int value) {
  if (op == SLIDER_DISPLAY_VALUE_SET) {
    display.setBacklightTimeout((unsigned int) value);
  }
}

// ---------------------------------------------------------------------------------
//                       Menu commands: battery calibration
// ---------------------------------------------------------------------------------

//
// live reading taken when the "Batt calib" slider is opened - held fixed for
// the duration of that one calibration session (battery resting voltage
// doesn't move fast enough for that to matter) so the rescaling in
// setBatteryCalibCallback() is against a single consistent reference, not a
// moving one. This is menu-flow bookkeeping, not battery state, so it lives
// here rather than in battery.h.
//
static float pendingCalibVoltageSnapshot = 0.0;

void setBatteryCalibCallback(byte op, float value); // forward decl - defined below

//
// Battery voltage calibration - no programmer needed. The slider shows the
// current live voltage (what the Climate/battery screens will display),
// seeded from that same reading; drag it until it matches a multimeter on
// the battery terminals, then Set. Since the ADC-to-volts conversion is a
// scalar (see BATTERY_VOLTS_SCALAR_DEFAULT's note in battery.h), Set
// rescales it proportionally rather than adding a fixed offset - see
// setBatteryCalibCallback() below.
//
void menuCommandSetBatteryCalib() {
  battery.read();
  pendingCalibVoltageSnapshot = battery.getVoltage();
  ui.displayFloatSlider(pendingCalibVoltageSnapshot - BATTERY_CALIBRATION_RANGE,
                        pendingCalibVoltageSnapshot + BATTERY_CALIBRATION_RANGE, BATTERY_CALIBRATION_STEP,
                        pendingCalibVoltageSnapshot, "Batt calib V", 1, setBatteryCalibCallback);
}
void setBatteryCalibCallback(byte op, float value) {
  //
  // voltage = adcCount * voltsScalar, and adcCount hasn't changed since the
  // slider opened (same snapshot), so scaling voltsScalar by
  // value/pendingCalibVoltageSnapshot makes that same adcCount read back as
  // exactly `value` next time.
  //
  if (op == SLIDER_DISPLAY_VALUE_SET) {
    battery.setScalar(battery.getScalar() * value / pendingCalibVoltageSnapshot);
  }
}

// ---------------------------------------------------------------------------------
//                       Menu commands: battery charge curve
// ---------------------------------------------------------------------------------
//
// Recalibrate the voltage each of the battery's fixed percent points
// (100/80/60/40/20/10/0%) corresponds to, e.g. against a second, known-good
// SOC source (a smart charger's own estimate, or a rest period + reference
// table for your specific battery chemistry). Each point is its own menu
// command (MENU_ITEM commands can't carry a parameter) but they all share
// one slider callback, keyed by which point was opened.
//
static byte socCurveEditIndex = 0;

void setSocCurveCallback(byte op, float value); // forward decl - defined below

void editSocCurvePoint(byte index) {
  socCurveEditIndex = index;
  char title[16];
  sprintf(title, "%d%% Volts", batteryCharge.getSocPercentLabel(index));
  ui.displayFloatSlider(BATTERY_SOC_VOLTAGE_MIN, BATTERY_SOC_VOLTAGE_MAX, BATTERY_SOC_VOLTAGE_STEP,
                        batteryCharge.getSocVoltage(index), title, 1, setSocCurveCallback);
}
void setSocCurveCallback(byte op, float value) {
  if (op == SLIDER_DISPLAY_VALUE_SET) {
    batteryCharge.setSocVoltage(socCurveEditIndex, value);
  }
}

//
// indices are hardcoded against BATTERY_SOC_TABLE's current 7 entries in
// battery_charge.h - update these alongside any change to that table.
//
static_assert(BATTERY_SOC_TABLE_SIZE == 7, "menuCommandSetSocXX wrappers below assume exactly 7 SOC breakpoints");

void menuCommandSetSoc100() {
  editSocCurvePoint(0);
}
void menuCommandSetSoc80() {
  editSocCurvePoint(1);
}
void menuCommandSetSoc60() {
  editSocCurvePoint(2);
}
void menuCommandSetSoc40() {
  editSocCurvePoint(3);
}
void menuCommandSetSoc20() {
  editSocCurvePoint(4);
}
void menuCommandSetSoc10() {
  editSocCurvePoint(5);
}
void menuCommandSetSoc0() {
  editSocCurvePoint(6);
}

//
// draw a connected line from (x0,y0) to (x1,y1), interpolating y across
// every intermediate column and bridging each column to the previous one's
// y - same technique Graph::draw() (graphs.h) uses for its adjacent data
// points, generalized here since drawSocCurveView() below only has
// BATTERY_SOC_TABLE_SIZE (7) points spread across the whole plot width,
// not one close together per column.
//
void graphConnectPoints(int x0, int y0, int x1, int y1) {
  int prevY = y0;
  graphPlotColumn(x0, y0, y0);
  for (int x = x0 + 1; x <= x1; x++) {
    int y = y0 + (int) (((long) (y1 - y0) * (x - x0)) / (x1 - x0));
    graphPlotColumn(x, min(y, prevY), max(y, prevY));
    prevY = y;
  }
}

//
// Read-only visualization of the calibrated curve itself (not a live
// reading): the BATTERY_SOC_TABLE_SIZE points plotted as percent (y axis,
// fixed 0-100 - that IS the table's range, unlike a History graph's
// autoscaled one) against voltage (x axis, autoscaled to the curve's own
// 0%/100% points rather than the full BATTERY_SOC_VOLTAGE_MIN/MAX slider
// range, for better resolution), connected point-to-point. The live battery
// voltage is marked as a vertical reference line, drawn before the curve so
// the curve stays fully visible where the two cross. Reuses the same plot
// geometry and graphPlotColumn() primitive as the History graphs (graphs.h)
// but isn't a Graph itself - Graph always plots a History time series,
// where this plots BatteryCharge's 7 fixed calibration points instead.
//
void drawSocCurveView() {
  ui.lcdClearDisplay();

  float liveVoltage = battery.getVoltage();
  char  voltBuf[6];
  if (isnan(liveVoltage)) {
    strcpy(voltBuf, "--");
  } else {
    dtostrf(liveVoltage, 1, 1, voltBuf);
    strcat(voltBuf, "V");
  }
  printLabeledValue(0, "SOC curve", voltBuf, 5); // widest: "20.4V"

  float vMin = batteryCharge.getSocVoltage(BATTERY_SOC_TABLE_SIZE - 1); // 0% point
  float vMax = batteryCharge.getSocVoltage(0);                         // 100% point
  if (vMax <= vMin) {
    vMax = vMin + 0.1; // guard a degenerate/miscalibrated curve
  }

  //
  // y-axis: fixed 0-100%
  //
  graphPlotColumn(PLOT_LEFT_X - 2, PLOT_TOP_ROW, PLOT_BOTTOM_ROW);
  ui.lcdSetCursorXY(PLOT_LEFT_X - 4, PLOT_TOP_BANK);
  ui.lcdPrintStringRightJustified("100", 4);
  ui.lcdSetCursorXY(PLOT_LEFT_X - 4, PLOT_BOTTOM_BANK);
  ui.lcdPrintStringRightJustified("0", 4);

  //
  // x-axis: voltage, autoscaled to the curve's own 0%/100% points
  //
  ui.lcdDrawRowOfPixels(PLOT_LEFT_X - 2, PLOT_RIGHT_X, X_AXIS_BANK, 0x01);
  char loLabel[6], hiLabel[6];
  dtostrf(vMin, 1, 1, loLabel);
  dtostrf(vMax, 1, 1, hiLabel);
  ui.lcdSetCursorXY(PLOT_LEFT_X - 2, X_AXIS_BANK);
  ui.lcdPrintString(loLabel);
  ui.lcdSetCursorXY(PLOT_RIGHT_X, X_AXIS_BANK);
  ui.lcdPrintStringRightJustified(hiLabel, 0);

  //
  // live-voltage marker - drawn before the curve so the curve (drawn last,
  // below) is never obscured where the two overlap
  //
  if (!isnan(liveVoltage)) {
    float frac  = constrain((liveVoltage - vMin) / (vMax - vMin), 0.0, 1.0);
    int   markX = PLOT_LEFT_X + (int) (frac * (PLOT_WIDTH - 1) + 0.5);
    graphPlotColumn(markX, PLOT_TOP_ROW, PLOT_BOTTOM_ROW);
  }

  //
  // connect the calibrated points left-to-right (0% -> 100%, lowest voltage
  // to highest)
  //
  int prevX = -1, prevY = 0;
  for (int i = BATTERY_SOC_TABLE_SIZE - 1; i >= 0; i--) {
    float voltage = batteryCharge.getSocVoltage((byte) i);
    byte  percent = batteryCharge.getSocPercentLabel((byte) i);
    int   x       = PLOT_LEFT_X + (int) ((voltage - vMin) / (vMax - vMin) * (PLOT_WIDTH - 1) + 0.5);
    int   y       = PLOT_BOTTOM_ROW - (int) (percent / 100.0 * (PLOT_HEIGHT - 1) + 0.5);
    if (prevX >= 0) {
      graphConnectPoints(prevX, prevY, x, y);
    } else {
      graphPlotColumn(x, y, y);
    }
    prevX = x;
    prevY = y;
  }

  ui.drawButtonBar("", "Back");
  while (ui.getButtonEvent() != BUTTON_ID_BACK + BUTTON_PUSHED_EVENT)
    ;
}

// ---------------------------------------------------------------------------------
//                       Menu commands: per-fan PID gains
// ---------------------------------------------------------------------------------
//
// which gain the currently-open slider is editing - shared by all 6
// menuCommandSetPidXxx() below so they don't each need their own callback.
// Menu-flow bookkeeping, not fan state, so it lives here rather than in
// fans_controller.h.
//
enum PidGainId
{
  PID_GAIN_KP_LEFT,
  PID_GAIN_KI_LEFT,
  PID_GAIN_KD_LEFT,
  PID_GAIN_KP_RIGHT,
  PID_GAIN_KI_RIGHT,
  PID_GAIN_KD_RIGHT
};
static byte pidGainEditId = 0;

void setPidGainCallback(byte op, float value); // forward decl - defined below

void editPidGain(byte id, char *label, float minV, float maxV, float step, float current) {
  pidGainEditId = id;
  ui.displayFloatSlider(minV, maxV, step, current, label, 3, setPidGainCallback);
}
void setPidGainCallback(byte op, float value) {
  if (op != SLIDER_DISPLAY_VALUE_SET) {
    return;
  }

  //
  // whichever gain this is, record the live battery voltage as its new
  // tuning reference - see the note on FanPIDControl::runPid() in
  // fan_pid_control.h.
  //
  battery.read();
  float voltageNow = battery.getVoltage();
  switch (pidGainEditId) {
    case PID_GAIN_KP_LEFT:
      PidFanLeft.setKp(value, voltageNow);
      break;
    case PID_GAIN_KI_LEFT:
      PidFanLeft.setKi(value, voltageNow);
      break;
    case PID_GAIN_KD_LEFT:
      PidFanLeft.setKd(value, voltageNow);
      break;
    case PID_GAIN_KP_RIGHT:
      PidFanRight.setKp(value, voltageNow);
      break;
    case PID_GAIN_KI_RIGHT:
      PidFanRight.setKi(value, voltageNow);
      break;
    case PID_GAIN_KD_RIGHT:
      PidFanRight.setKd(value, voltageNow);
      break;
  }
}

void menuCommandSetPidKpLeft() {
  editPidGain(PID_GAIN_KP_LEFT, "Kp Left", FAN_PID_KP_MIN, FAN_PID_KP_MAX, FAN_PID_KP_STEP, PidFanLeft.getKp());
}
void menuCommandSetPidKiLeft() {
  editPidGain(PID_GAIN_KI_LEFT, "Ki Left", FAN_PID_KI_MIN, FAN_PID_KI_MAX, FAN_PID_KI_STEP, PidFanLeft.getKi());
}
void menuCommandSetPidKdLeft() {
  editPidGain(PID_GAIN_KD_LEFT, "Kd Left", FAN_PID_KD_MIN, FAN_PID_KD_MAX, FAN_PID_KD_STEP, PidFanLeft.getKd());
}
void menuCommandSetPidKpRight() {
  editPidGain(PID_GAIN_KP_RIGHT, "Kp Right", FAN_PID_KP_MIN, FAN_PID_KP_MAX, FAN_PID_KP_STEP, PidFanRight.getKp());
}
void menuCommandSetPidKiRight() {
  editPidGain(PID_GAIN_KI_RIGHT, "Ki Right", FAN_PID_KI_MIN, FAN_PID_KI_MAX, FAN_PID_KI_STEP, PidFanRight.getKi());
}
void menuCommandSetPidKdRight() {
  editPidGain(PID_GAIN_KD_RIGHT, "Kd Right", FAN_PID_KD_MIN, FAN_PID_KD_MAX, FAN_PID_KD_STEP, PidFanRight.getKd());
}

// ---------------------------------------------------------------------------------
//                    Menu commands: PID autotune ("Tune Left"/"Tune Right")
// ---------------------------------------------------------------------------------
//
// Sequences one fan's autotune primitives (fan.h/fan_pid_control.h - which
// know nothing about the LCD or the menu). First sweeps duty downward from
// FAN_AUTOTUNE_LOW_PCT toward 0 until the fan actually stops spinning -
// this both measures rpmLow (the sweep's first step, at
// FAN_AUTOTUNE_LOW_PCT itself) and finds this fan's *real, measured* stall
// RPM, rather than assuming a fixed constant - then tests HIGH and MAX at
// their single fixed duties (MAX giving this fan's real RPM ceiling), shows
// progress between steps, then the resulting gains/stall/max RPM (or a
// failure message) until the user presses OK. Takes the target fan's raw
// Fan (for kickstart()/the sweep) and FanPIDControl (for
// autotuneStep()/gains) by reference, plus its on-screen label, so "Tune
// Left"/"Tune Right" are just two one-line callers below rather than two
// near-duplicate sequences.
//

void autotuneFan(Fan &hwFan, PIDFanControl &pid, const char *label) {
  fanControl.stopAll();

  ui.lcdClearDisplay();
  ui.lcdSetCursorXY(LCD_WIDTH_IN_PIXELS / 2, 0);
  char title[12];
  sprintf(title, "Tune %s", label);
  ui.lcdPrintStringCentered(title, 0);

  hwFan.kickstart();

  //
  // one step at a time, showing which duty is currently under test - each
  // step blocks for FAN_AUTOTUNE_SETTLE_CYCLES * CONTROL_INTERVAL (~8s).
  // Sweep down from FAN_AUTOTUNE_LOW_PCT in fixed 5% decrements until a
  // step reads 0 RPM (the fan has actually stalled) or duty bottoms out;
  // stallRpm ends up as the last non-zero reading, i.e. this fan's own
  // measured stall boundary - not a guessed constant.
  //
  char         stepMsg[17];
  unsigned int rpmLow   = 0;
  unsigned int stallRpm = 0;
  for (int testPct = FAN_AUTOTUNE_LOW_PCT; testPct > 0; testPct -= 5) {
    sprintf(stepMsg, "Testing %d%%...", testPct);
    ui.lcdSetCursorXY(0, 2);
    ui.lcdPrintStringLeftJustified(stepMsg, 16);

    unsigned int rpm = pid.autotuneStep((byte) testPct);
    if (testPct == FAN_AUTOTUNE_LOW_PCT) {
      rpmLow = rpm;
    }
    if (rpm == 0) {
      break;
    }
    stallRpm = rpm;
  }

  sprintf(stepMsg, "Testing %d%%...", FAN_AUTOTUNE_HIGH_PCT);
  ui.lcdSetCursorXY(0, 2);
  ui.lcdPrintStringLeftJustified(stepMsg, 16);
  unsigned int rpmHigh = pid.autotuneStep(FAN_AUTOTUNE_HIGH_PCT);

  sprintf(stepMsg, "Testing %d%%...", FAN_AUTOTUNE_MAX_PCT);
  ui.lcdSetCursorXY(0, 2);
  ui.lcdPrintStringLeftJustified(stepMsg, 16);
  unsigned int maxRpm = pid.autotuneStep(FAN_AUTOTUNE_MAX_PCT);

  fanControl.stopAll();

  float kp, ki, kd;
  bool  ok = PIDFanControl::computeAutotunePID(stallRpm, rpmLow, rpmHigh, kp, ki, kd);

  ui.lcdClearDisplay();
  ui.lcdSetCursorXY(LCD_WIDTH_IN_PIXELS / 2, 0);

  char line1[17], line2[17], line3[17];
  if (ok) {
    //
    // record the live (resting - fans are stopped above) battery voltage as
    // the calibration reference for these gains AND the freshly-measured
    // stallRpm/rpmLow/rpmHigh/maxRpm - see the note on
    // FanPIDControl::isStalled() in fan_pid_control.h.
    //
    battery.read();
    AutotuneResult result = {kp, ki, kd, stallRpm, rpmLow, rpmHigh, maxRpm, battery.getVoltage()};
    pid.applyAutotuneResults(result);

    ui.lcdPrintStringCentered("Tune OK", 0);
    char kpBuf[8], kiBuf[8];
    dtostrf(kp, 1, 3, kpBuf);
    dtostrf(ki, 1, 3, kiBuf);
    sprintf(line1, "Kp%s Ki%s", kpBuf, kiBuf);
    sprintf(line2, "Stall %u RPM", stallRpm);
    sprintf(line3, "Max %u RPM", maxRpm);
  } else {
    ui.lcdPrintStringCentered("Tune FAILED", 0);
    strcpy(line1, "Check fan/wiring");
    line2[0] = '\0';
    line3[0] = '\0';
  }
  ui.lcdSetCursorXY(0, 1);
  ui.lcdPrintStringLeftJustified(line1, 16);
  ui.lcdSetCursorXY(0, 2);
  ui.lcdPrintStringLeftJustified(line2, 16);
  ui.lcdSetCursorXY(0, 3);
  ui.lcdPrintStringLeftJustified(line3, 16);

  ui.drawButtonBar("OK", "");
  while (ui.getButtonEvent() != BUTTON_ID_SELECT + BUTTON_PUSHED_EVENT)
    ;
}

void menuCommandAutotuneLeft() {
  autotuneFan(fanLeft, PidFanLeft, "Left");
}
void menuCommandAutotuneRight() {
  autotuneFan(fanRight, PidFanRight, "Right");
}

// ---------------------------------------------------------------------------------
//                              Menu structure
// ---------------------------------------------------------------------------------
//
// forward declarations for the menu tables (mainMenu/settingsMenu/manualMenu/
// socCurveMenu/pidTuningMenu each reference each other)
//
extern MENU_ITEM mainMenu[];
extern MENU_ITEM settingsMenu[];
extern MENU_ITEM manualMenu[];
extern MENU_ITEM socCurveMenu[];
extern MENU_ITEM pidTuningMenu[];

//
// Main menu.  The 4th field of the header is NULL, which enables the "Back"
// button so the user can leave the menu and return to the live status screen.
//
MENU_ITEM mainMenu[] = {{MENU_ITEM_TYPE_MAIN_MENU_HEADER, "Fan Controller", NULL, NULL},
                        {MENU_ITEM_TYPE_SUB_MENU, "Thresholds", NULL, settingsMenu},
                        {MENU_ITEM_TYPE_SUB_MENU, "Manual / test", NULL, manualMenu},
                        {MENU_ITEM_TYPE_SUB_MENU, "PID tuning", NULL, pidTuningMenu},
                        {MENU_ITEM_TYPE_COMMAND, "Set contrast", menuCommandSetContrast, NULL},
                        {MENU_ITEM_TYPE_COMMAND, "Backlight s", menuCommandSetBacklightTimeout, NULL},
                        {MENU_ITEM_TYPE_COMMAND, "Batt calib", menuCommandSetBatteryCalib, NULL},
                        {MENU_ITEM_TYPE_SUB_MENU, "Charge curve", NULL, socCurveMenu},
                        {MENU_ITEM_TYPE_END_OF_MENU, "", NULL, NULL}};

//
// Thresholds sub-menu - the four numbers that shape the control law
//
MENU_ITEM settingsMenu[] = {{MENU_ITEM_TYPE_SUB_MENU_HEADER, "Thresholds", NULL, mainMenu},
                            {MENU_ITEM_TYPE_COMMAND, "Temp start C", menuCommandSetTempStart, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "Temp high C", menuCommandSetTempHigh, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "Humid start%", menuCommandSetHumStart, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "Humid high %", menuCommandSetHumHigh, NULL},
                            {MENU_ITEM_TYPE_END_OF_MENU, "", NULL, NULL}};

//
// Manual / test sub-menu - override the automatic control for testing/servicing
//
MENU_ITEM manualMenu[] = {{MENU_ITEM_TYPE_SUB_MENU_HEADER, "Manual / test", NULL, mainMenu},
                          {MENU_ITEM_TYPE_TOGGLE, "Manual", menuToggleManualCallback, NULL},
                          {MENU_ITEM_TYPE_COMMAND, "Fan speed %", menuCommandSetManualSpeed, NULL},
                          {MENU_ITEM_TYPE_END_OF_MENU, "", NULL, NULL}};

//
// Charge curve sub-menu - recalibrate the voltage for each of the battery's
// fixed SOC percent points; see editSocCurvePoint() above. "View curve"
// draws the resulting curve instead of editing it - see drawSocCurveView().
//
MENU_ITEM socCurveMenu[] = {{MENU_ITEM_TYPE_SUB_MENU_HEADER, "Charge curve", NULL, mainMenu},
                            {MENU_ITEM_TYPE_COMMAND, "View curve", drawSocCurveView, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "100%", menuCommandSetSoc100, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "80%", menuCommandSetSoc80, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "60%", menuCommandSetSoc60, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "40%", menuCommandSetSoc40, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "20%", menuCommandSetSoc20, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "10%", menuCommandSetSoc10, NULL},
                            {MENU_ITEM_TYPE_COMMAND, "0%", menuCommandSetSoc0, NULL},
                            {MENU_ITEM_TYPE_END_OF_MENU, "", NULL, NULL}};

//
// PID tuning sub-menu - independent Kp/Ki/Kd for each fan, plus a one-shot
// autotune trigger per fan; see autotuneFan() above for what "Tune
// Left"/"Tune Right" actually do.
//
MENU_ITEM pidTuningMenu[] = {{MENU_ITEM_TYPE_SUB_MENU_HEADER, "PID tuning", NULL, mainMenu},
                             {MENU_ITEM_TYPE_COMMAND, "Kp Left", menuCommandSetPidKpLeft, NULL},
                             {MENU_ITEM_TYPE_COMMAND, "Ki Left", menuCommandSetPidKiLeft, NULL},
                             {MENU_ITEM_TYPE_COMMAND, "Kd Left", menuCommandSetPidKdLeft, NULL},
                             {MENU_ITEM_TYPE_COMMAND, "Tune Left", menuCommandAutotuneLeft, NULL},
                             {MENU_ITEM_TYPE_COMMAND, "Kp Right", menuCommandSetPidKpRight, NULL},
                             {MENU_ITEM_TYPE_COMMAND, "Ki Right", menuCommandSetPidKiRight, NULL},
                             {MENU_ITEM_TYPE_COMMAND, "Kd Right", menuCommandSetPidKdRight, NULL},
                             {MENU_ITEM_TYPE_COMMAND, "Tune Right", menuCommandAutotuneRight, NULL},
                             {MENU_ITEM_TYPE_END_OF_MENU, "", NULL, NULL}};

// ---------------------------------------------------------------------------------
//                                    Setup
// ---------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);

  //
  // fans off before anything else, so we never spin up unexpectedly at boot -
  // each fan's own PID (fan_pid_control.h) brings up its own Fan hardware
  // internally (fan.h), including the shared Timer1 PWM config, a one-time
  // singleton that only actually runs on the first of these two calls;
  // fanControl (fans_controller.h) never touches init() itself, since it
  // only ever orchestrates the pair, not their bring-up.
  //
  PidFanLeft.init();
  PidFanRight.init();

  sensor.init();
  ui.connectToPins(LCD_CLOCK_PIN, LCD_DATA_IN_PIN, LCD_DATA_CONTROL_PIN, LCD_CHIP_ENABLE_PIN, LCD_BUTTONS_PIN);
  display.init();
  controlLoop.init();
  battery.init();
  batteryCharge.init();

  //
  // take an initial reading so the first status screen isn't blank
  //
  controlLoop.service();
}

// ---------------------------------------------------------------------------------
//                                  Main loop
// ---------------------------------------------------------------------------------
//
// Like the library's Stopwatch example, the first thing the user sees is the
// application (the live status screen), not a menu.  Pressing "Menu" (the Back
// button) pulls up the configuration menu; pressing Back inside the menu returns
// here.  The fan control law keeps running the whole time the status screen is
// shown.
//
void loop() {
  drawMainScreen(true); // true = full redraw (clear + button bar)

  unsigned long lastDraw = millis();
  while (true) {
    //
    // keep the fans regulated
    //
    controlLoop.service();

    //
    // refresh the readout twice a second
    //
    if (millis() - lastDraw >= 500) {
      lastDraw = millis();
      drawMainScreen(false);
    }

    byte ev = ui.getButtonEvent();
    display.backlightService(ev);

    //
    // the Back button opens the configuration menu, regardless of which
    // main screen is currently showing
    //
    if (ev == BUTTON_ID_BACK + BUTTON_PUSHED_EVENT) {
      ui.displayAndExecuteMenu(mainMenu);
      //
      // button activity while the menu itself is open isn't visible to
      // backlightService() above (same reason the fan control law also
      // pauses while a menu is open - this loop, and everything in it,
      // simply isn't running then) - treat coming back out of the menu as
      // fresh activity so the backlight doesn't potentially go dark the
      // instant the status screen reappears
      //
      display.backlightNoteActivity();
      return; // redraw the status screen from loop()
    }

    //
    // the Select button (bottom-left) cycles to the next main screen
    //
    if (ev == BUTTON_ID_SELECT + BUTTON_PUSHED_EVENT) {
      mainScreenIndex = (mainScreenIndex + 1) % NUM_MAIN_SCREENS;
      drawMainScreen(true);
      lastDraw = millis();
    }
  }
}
