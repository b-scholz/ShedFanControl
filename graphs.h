//      ******************************************************************
//      *                                                                *
//      *   graphs.h  -  Temperature / humidity / battery voltage / charge *
//      *                history graph screens.                            *
//      *                                                                *
//      *   Graph draws a simple auto-scaled 48h line graph, with a         *
//      *   labeled y-axis (min/max value + line) and x-axis (time span +  *
//      *   line), on the Nokia 5110 using only the ArduinoUserInterface    *
//      *   library's public lcdDrawRowOfPixels(x1, x2, bankLine,          *
//      *   byteOfPixels) primitive - calling it with x1==x2 sets one      *
//      *   column's 8-pixel band (bit0 = its top pixel), which is enough  *
//      *   to plot arbitrary points/lines without needing a graphics      *
//      *   library.                                                       *
//      *                                                                 *
//      *   draw(hist) is Graph's entire per-call interface - it takes     *
//      *   only the History (history.h) to plot. Everything that differs  *
//      *   between the four graphs (title, how to format a value into a   *
//      *   y-axis label, and - for the charge graph only - how to turn a  *
//      *   raw stored byte into the value actually plotted) is bound       *
//      *   once, at construction: tempGraph/humidityGraph/batteryGraph/    *
//      *   chargeGraph below are those four specializations, one Graph     *
//      *   instance each.                                                 *
//      *                                                                 *
//      *   Reached only as main (home) screens - see FanController.ino -  *
//      *   there is no standalone menu entry.                             *
//      *                                                                 *
//      ******************************************************************

#ifndef GRAPHS_H
#define GRAPHS_H

#include <Arduino.h>
#include <ArduinoUserInterface.h>
#include "config.h"
#include "history.h"
#include "battery_charge.h"
#include "control_loop.h"

extern ArduinoUserInterface ui;   // defined in the main sketch

//
// Smoothing window (simple moving average, symmetric) applied before
// plotting, on top of - not a substitute for - the bucket averaging each
// point already represents.  Fixed since there's only one time scale now.
//
const uint8_t GRAPH_SMOOTH_WINDOW = 3;

//
// pixel rows 0-7 (bank 0) hold the header text; rows 40-47 (bank 5) are the
// button bar.  Bank 4 is the x-axis (a thin line plus "time ago" labels), so
// the plot itself uses banks 1-3 = rows 8-31 (24 px tall).  Column-wise, the
// left PLOT_LEFT_X pixels are reserved for the y-axis value labels.
//
const int PLOT_TOP_BANK    = 1;
const int PLOT_BOTTOM_BANK = 3;
const int PLOT_TOP_ROW     = PLOT_TOP_BANK * 8;                          // 8
const int PLOT_HEIGHT      = (PLOT_BOTTOM_BANK - PLOT_TOP_BANK + 1) * 8; // 24
const int PLOT_BOTTOM_ROW  = PLOT_TOP_ROW + PLOT_HEIGHT - 1;             // 31
const int X_AXIS_BANK      = PLOT_BOTTOM_BANK + 1;                       // 4

const int PLOT_LEFT_X  = 28;                  // room for a 4-char label (e.g. battery "12.6") + margin
const int PLOT_RIGHT_X = LCD_LAST_COLUMN_X;   // 83
const int PLOT_WIDTH   = PLOT_RIGHT_X - PLOT_LEFT_X + 1;

//
// scratch buffers shared by every Graph instance's draw() - only one graph
// screen is ever shown at a time, so there's no need for each instance to
// have its own copy (that would cost an extra HISTORY_POINTS*2 bytes of RAM
// per graph).
//
static uint8_t graphRawBuf[HISTORY_POINTS];
static uint8_t graphSmoothBuf[HISTORY_POINTS];

//
// value -> y-axis label formatters, and the charge graph's raw-byte ->
// plotted-value transform - free functions so they can be passed as Graph
// constructor arguments (see the four instances below).
//
void graphFormatPlain(uint8_t value, char *buf) { sprintf(buf, "%d", value); }

void graphFormatVolts(uint8_t value, char *buf) { sprintf(buf, "%d.%d", value / 10, value % 10); }

//
// History stores battery voltage as tenths-of-a-volt; the charge graph
// plots state-of-charge instead, so each raw byte is translated through
// BatteryCharge::percentFromVoltage() before smoothing/autoscaling - see
// chargeGraph below. No separate storage - the charge curve can be
// recalibrated at any time (see FanController.ino's "Charge curve" submenu)
// and this graph reflects that on its next redraw.
//
uint8_t graphChargeFromVoltsTenths(uint8_t voltsTenths) {
  return batteryCharge.percentFromVoltage(voltsTenths / 10.0);
}


class Graph {
public:
  typedef void (*LabelFormatter)(uint8_t value, char *buf);
  typedef uint8_t (*ValueTransform)(uint8_t raw);

  //
  // Enter: titleIn     = fixed header text for this graph (e.g. "Temp 48h")
  //        formatterIn = renders one already-transformed value into its
  //                      y-axis label string
  //        transformIn = optional - converts a raw History byte into the
  //                      value to smooth/autoscale/plot (e.g. volts ->
  //                      charge %); defaults to null, meaning "use the raw
  //                      byte directly" (temp/humidity/battery all do this)
  //
  Graph(const char *titleIn, LabelFormatter formatterIn, ValueTransform transformIn = nullptr)
    : title(titleIn), formatter(formatterIn), transform(transformIn) {}

  //
  // draw this graph's content (header, axes, plot) for the given history,
  // without touching the display clear or button bar - called directly by
  // the main-screen dispatch in FanController.ino. hist is the only thing that
  // varies per call; title/formatter/transform were fixed at construction.
  //
  void draw(const History &hist) const
  {
    uint8_t count = hist.count();

    ui.lcdSetCursorXY(0, 0);
    ui.lcdPrintString((char *) title);

    if (count == 0) {
      ui.lcdSetCursorXY(LCD_WIDTH_IN_PIXELS / 2, 3);
      ui.lcdPrintStringCentered("No data yet", 0);
      return;
    }

    //
    // gather the raw series (through the transform, if any), then smooth it
    //
    uint8_t *raw = graphRawBuf;
    uint8_t *smoothed = graphSmoothBuf;
    for (uint8_t i = 0; i < count; i++) {
      uint8_t v = hist.get(i);
      raw[i] = transform ? transform(v) : v;
    }
    graphSmooth(raw, count, GRAPH_SMOOTH_WINDOW, smoothed);

    //
    // autoscale to the smoothed series' min/max
    //
    uint8_t vMin = smoothed[0], vMax = smoothed[0];
    for (uint8_t i = 1; i < count; i++) {
      if (smoothed[i] < vMin) { vMin = smoothed[i]; }
      if (smoothed[i] > vMax) { vMax = smoothed[i]; }
    }
    if (vMax == vMin) {
      vMax = vMin + 1;             // avoid a zero-height range
    }

    //
    // y-axis: max/min value labels + line; x-axis: fixed 48h span + line
    //
    char maxLabel[6], minLabel[6];
    formatter(vMax, maxLabel);
    formatter(vMin, minLabel);
    drawYAxis(maxLabel, minLabel);
    drawXAxis("-48h");

    //
    // plot: map each point to an x column and a y row, connecting
    // consecutive points with a vertical run in the current column (a good
    // approximation of a continuous line when points are packed this
    // closely together)
    //
    int prevY = -1;
    for (uint8_t i = 0; i < count; i++) {
      int x = PLOT_LEFT_X + ((count > 1) ? ((int) i * (PLOT_WIDTH - 1)) / (count - 1) : 0);
      int y = PLOT_BOTTOM_ROW -
              ((int)(smoothed[i] - vMin) * (PLOT_HEIGHT - 1)) / (vMax - vMin);

      int rowTop    = (prevY < 0) ? y : min(y, prevY);
      int rowBottom = (prevY < 0) ? y : max(y, prevY);
      graphPlotColumn(x, rowTop, rowBottom);
      prevY = y;
    }
  }

private:
  const char *title;
  LabelFormatter formatter;
  ValueTransform transform;

  //
  // symmetric moving average with edge clamping; window=1 is a no-op copy
  //
  static void graphSmooth(const uint8_t *raw, uint8_t count, uint8_t window, uint8_t *out)
  {
    int half = window / 2;
    for (uint8_t i = 0; i < count; i++) {
      int sum = 0;
      int n = 0;
      for (int k = -half; k <= half; k++) {
        int idx = (int) i + k;
        if (idx < 0) { idx = 0; }
        if (idx >= count) { idx = count - 1; }
        sum += raw[idx];
        n++;
      }
      out[i] = (uint8_t)(sum / n);
    }
  }

  //
  // set the pixels for one column's vertical run [rowTop..rowBottom]
  // (inclusive, absolute LCD rows), across however many of the 4 plot
  // banks it touches
  //
  static void graphPlotColumn(int x, int rowTop, int rowBottom)
  {
    for (int bank = PLOT_TOP_BANK; bank <= PLOT_BOTTOM_BANK; bank++) {
      int bankTop = bank * 8;
      int bankBottom = bankTop + 7;
      if (rowBottom < bankTop || rowTop > bankBottom) {
        continue;                                  // this bank untouched - stays blank
      }
      int loRow = max(rowTop, bankTop);
      int hiRow = min(rowBottom, bankBottom);
      byte pattern = 0;
      for (int r = loRow; r <= hiRow; r++) {
        pattern |= (byte)(1 << (r - bankTop));
      }
      ui.lcdDrawRowOfPixels(x, x, bank, pattern);
    }
  }

  //
  // vertical y-axis: a thin line at the plot's left edge, with the max
  // value labeled at its top and the min value at its bottom
  // (right-justified into the reserved left margin, so they never collide
  // with the plotted data)
  //
  static void drawYAxis(const char *maxLabel, const char *minLabel)
  {
    graphPlotColumn(PLOT_LEFT_X - 2, PLOT_TOP_ROW, PLOT_BOTTOM_ROW);

    //
    // pad to 4 chars (the widest this ever gets, e.g. battery "20.4") so a
    // shorter label after autoscale changes (e.g. humidity "9") fully
    // blanks out a longer previous one instead of leaving stale digits
    // behind - this exactly fills the PLOT_LEFT_X margin reserved for
    // these labels
    //
    ui.lcdSetCursorXY(PLOT_LEFT_X - 4, PLOT_TOP_BANK);
    ui.lcdPrintStringRightJustified((char *) maxLabel, 4);

    ui.lcdSetCursorXY(PLOT_LEFT_X - 4, PLOT_BOTTOM_BANK);
    ui.lcdPrintStringRightJustified((char *) minLabel, 4);
  }

  //
  // horizontal x-axis: a thin line spanning the plot width, with a "time
  // ago" label at its left end and "now" at its right end, drawn in the
  // bank just below the plot
  //
  static void drawXAxis(const char *leftLabel)
  {
    ui.lcdDrawRowOfPixels(PLOT_LEFT_X - 2, PLOT_RIGHT_X, X_AXIS_BANK, 0x01);   // thin line, top row of the bank

    ui.lcdSetCursorXY(PLOT_LEFT_X - 2, X_AXIS_BANK);
    ui.lcdPrintString((char *) leftLabel);

    ui.lcdSetCursorXY(PLOT_RIGHT_X, X_AXIS_BANK);
    ui.lcdPrintStringRightJustified("now", 0);
  }
};

//
// the four specializations - each just a title + label formatter (+
// transform for charge) - defined in FanController.ino, alongside every
// other subsystem singleton; FanController.ino calls these directly (see
// drawMainScreen()).
//
extern Graph tempGraph;
extern Graph humidityGraph;
extern Graph batteryGraph;
extern Graph chargeGraph;

#endif  // GRAPHS_H
