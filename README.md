# Shed Fan Controller

An Arduino Nano sketch that airs out a shed with **two 4-wire PC fans (left & right)**
whenever it gets **too warm or too damp** inside. A **DHT11** measures temperature and
humidity, a **12 V SLA battery** is monitored through a voltage divider, and a
**Nokia 5110 LCD** with four buttons shows live status plus menus for thresholds,
manual control, and history graphs. Fan speed is regulated smoothly from **LOW → HIGH**.

Running on HIGH is loud and wears the fans out fast, so the controller deliberately
keeps them in a **quiet band (LOW…MED)** for everyday airing and only unlocks HIGH
when the temperature or the humidity actually reaches its *high* threshold.

The display, menus, buttons, sliders and EEPROM settings are all driven by Stan
Reifel's **[ArduinoUserInterface](https://github.com/Stan-Reifel/ArduinoUserInterface)**
library.

---

## Files

Each subsystem is a real C++ **class**, in its own file, owning its own
pins/tuning constants, `private` state, and EEPROM persistence - and,
importantly, **none of it has any menu code of its own**. Every class
exposes only small `public` getter/setter/action methods (`battery.getX()`/
`setX()`, `display.getX()`/`setX()`, `fanLeft.getX()`/`setX()`,
`controlLoop.getX()`/`setX()`), and the menu commands/live status screens in
`FanController.ino` are the *only* code that ever calls
`ui.displaySlider()`/`ui.displayFloatSlider()` or touches a `MENU_ITEM`
table - they talk to every other subsystem exclusively through those public
methods, never their internal state directly (they stay plain free
functions rather than a class, since `MENU_ITEM`'s function-pointer table
needs C-compatible callbacks, which a non-static member function can't
provide). `Battery`/`Fan`/`FanPIDControl`/`FanControl`/
`ControlLoop`/`Display` all inherit a small `Module` base class (`module.h`)
that gives them a shared `protected ui` reference for EEPROM/LCD access, plus a
generalized struct-based EEPROM helper (see "EEPROM layout" below); `Sensor`
never touches EEPROM, so it's a plain standalone class instead. `ControlLoop`
takes its `Sensor`/`Battery`/`FanControl` dependencies as constructor
references rather than reaching for globals - likewise `FanControl` takes
two `Fan&` *and* two `FanPIDControl&` (see `fan.h`/`fan_pid_control.h`
below): it talks to the `Fan` references directly for raw duty/RPM (manual
mode, kick-start, stop, mismatch checks) and to the `FanPIDControl`
references only for the closed-loop auto path - `FanPIDControl` doesn't
re-expose the `Fan` methods it wraps as pass-throughs, see "Files" below.
Each `FanPIDControl` takes its own `Fan&` too (composition, not
inheritance). Every class has exactly one instance (`Fan` and
`FanPIDControl` each have exactly **two** - `fanLeftHw`/`fanRightHw` and
`fanLeft`/`fanRight` respectively - all other per-fan duplication was
collapsed away, see below), created once in
`FanController.ino` and reached everywhere else through an `extern`
declaration in its own header - see the comment block at the top of
`FanController.ino` for the short version.

| File                | Purpose                                                                |
|---------------------|-------------------------------------------------------------------------|
| `FanController.ino` | Instantiates every subsystem object, `setup()`, the main loop, **and** the live status screens + every menu command/tree (free functions, not a class - see above). |
| `module.h`          | `Module` - shared base class giving `Battery`/`BatteryCharge`/`Fan`/`FanPIDControl`/`FanControl`/`ControlLoop`/`Display` a `protected ui` reference, plus the generalized struct-based EEPROM `loadState()`/`saveState()` helpers. |
| `config.h`          | The truly cross-cutting things: `CONTROL_INTERVAL`/`HISTORY_POINTS` (needed by files included before `control_loop.h`, which otherwise owns that cadence/sizing - see the comment at the top of `config.h`) and the EEPROM address book (see "EEPROM layout" below). |
| `history.h`         | `History` class - a generic rolling 48h ring buffer for one byte-valued channel; `ControlLoop` owns three instances (temp/humidity/battery voltage) - see below. |
| `sensor.h`          | `Sensor` class - **Temp/Humid Sensor (DHT11)** - pins/tuning, reading, oversampling, fail detection. |
| `battery.h`         | `Battery` class - **Battery Reading** - pins/divider/tuning, voltage, calibration. |
| `battery_charge.h`  | `BatteryCharge` class - turns a voltage into a state-of-charge estimate via a runtime-editable curve. Split out from `Battery`, which never needs to know charge % exists - see "Battery charge curve" below. |
| `fan.h`             | `Fan` class - **one physical fan's** raw PWM + tach hardware only: pin, Timer1 compare register, `setDuty()`/`getDuty()`, `read()`/`getRpm()`. No PID/EEPROM at all. Instantiated *twice* - `fanLeftHw`/`fanRightHw` - and reached directly (not through `FanPIDControl`) by anything that needs raw duty/RPM access. |
| `fan_pid_control.h` | `FanPIDControl` class - closed-loop PID on top of a `Fan` it holds *by reference* (composition, not inheritance): gains, `autotuneStep()`/`computeAutotunePID()`. Deliberately doesn't re-expose the `Fan` methods it wraps (including `kickstart()`) as pass-throughs. Instantiated *twice* - `fanLeft`/`fanRight` - so there's no Left/Right-suffixed duplication anywhere. |
| `fan_control.h`     | `FanControl` class - the *pair-level* control law over both `Fan`s and `FanPIDControl`s: drives both to the same target RPM/duty every cycle, and cross-checks their measured RPM for a sustained mismatch fault. |
| `control_loop.h`    | `ControlLoop` class - manual/auto mode, climate thresholds, the auto RPM law, the per-cycle orchestration, **and the 48h temp/humidity/battery-voltage history** (`History` instances fed straight from Sensor/Battery's live readings each cycle - see "History storage" below). Takes `Sensor`/`Battery`/`FanControl` by constructor reference; exposes only `public` getter/setter/`service()` methods. |
| `display.h`         | `Display` class - LCD/button pins, contrast, and backlight (on-activity, auto-off). |
| `graphs.h`          | `Graph` class - draws one auto-scaled 48h line graph from a `History` (`draw(hist)` is its whole per-call interface); `tempGraph`/`humidityGraph`/`batteryGraph`/`chargeGraph` are four specializations (title + label formatter, +value transform for charge), configured once at construction. |
| `circuit.svg`       | Wiring diagram.                                                        |

---

## Required libraries

Install these from the Arduino IDE **Library Manager** (Tools → Manage Libraries):

1. **ArduinoUserInterface** by Stan Reifel — display / menu / buttons.
   (or from <https://github.com/Stan-Reifel/ArduinoUserInterface>)
2. **DHT sensor library** by Adafruit — the DHT11.
3. **Adafruit Unified Sensor** by Adafruit — dependency of the DHT library.

Board: Arduino **Nano** (ATmega328). The LCD/button pins match the "Arduino UI
Shield" layout the ArduinoUserInterface library is designed around.

> **Required manual patch — 180° display rotation.** This build's Nokia 5110 is
> physically mounted upside down, and the PCD8544 controller has no
> mirror/rotate register (unlike e.g. SSD1306), so the rotation is patched into
> the **library's own source**, not the sketch: `lcdSetCursorXY()` and
> `lcdWriteData()` in `ArduinoUserInterface.cpp` are modified to address the
> display at its 180°-rotated physical position and bit-reverse each byte
> (search that file for "SHED FAN CONTROLLER PATCH" to find both edits). This
> is required because it's the *only* place that can correctly rotate the
> library's own menu/slider/text rendering, not just this sketch's custom
> graph drawing. **Consequence: reinstalling or updating the ArduinoUserInterface
> library from the Library Manager will silently overwrite this patch and flip
> the display back upside down** — reapply the two edits (or diff against this
> repo's copy) after any library reinstall. If the LCD is ever physically
> remounted right-side up, revert both functions to a plain passthrough of
> `column`/`lineNumber` and remove the two `lcdRotatedWriteColumnX`/
> `lcdRotatedWriteRowY` shadow variables.

---

## About 4-wire (PWM) fans

A 4-wire PC fan has: **GND**, **+12 V**, **TACH** (RPM out) and **PWM** (speed in).
Unlike a 2-wire fan you **do not switch its power to change speed** — the +12 V feeds
the fan continuously and you send a **25 kHz PWM signal on the control (blue) wire**.
This sketch generates that 25 kHz with Timer1 on pins **D9/D10**, and reads the
**TACH** wire to show real RPM and catch a stalled/failed fan. Speed (including a
full stop at 0%) is set entirely on the PWM wire — no power-switching MOSFET needed
or used.

Typical wire colours (verify against your fan's datasheet):

| Wire            | Common colour | Goes to                                   |
|-----------------|---------------|-------------------------------------------|
| GND             | black         | common ground (Arduino + 12 V supply)     |
| +12 V           | yellow        | +12 V supply, direct (no switching)       |
| TACH (sense)    | green         | Arduino D2 / D3 (with pull-up)            |
| PWM (control)   | blue          | Arduino D9 / D10                          |

---

## Wiring

See `circuit.svg` for the full diagram. Summary:

### Nokia 5110 LCD (3.3 V) + 4 buttons

The LCD is a 3.3 V part; its five signal lines (RST/CLK/DIN/DC/CE) are level-shifted
down from the Nano's 5 V logic with simple resistor dividers — **order matters**:
10 kΩ in series from the Nano pin to a tap, then 20 kΩ from that tap to GND, with
the tap (not the raw Nano pin) feeding the LCD. This is one-way, Nano-to-LCD only,
which is all these signals need — putting the 20 kΩ-to-GND on the wrong side of the
10 kΩ (i.e. between the tap and the LCD instead of between the tap and GND) doesn't
divide the voltage at all, since the LCD's high-impedance CMOS inputs draw
negligible current through the 10 kΩ. The four buttons (Up/Down/Select/Back) share
**one analog pin** through a resistor ladder.

| Signal                 | Nano pin | Notes                          |
|------------------------|----------|---------------------------------|
| LCD Clock (CLK)        | A0       | via level shifter               |
| LCD Data In (DIN)      | A1       | via level shifter               |
| LCD Data/Control (DC)  | A2       | via level shifter                |
| LCD Chip Enable (CE)   | A3       | via level shifter                |
| LCD Reset (RST)        | —        | via level shifter (tie high)     |
| Buttons (analog)       | A6       | 10 kΩ pull-up + ladder to GND    |
| Backlight switch       | A4       | base of Q1 (PNP), via ~4.7 kΩ    |

Button ladder (pull-up to +5V, each button to GND through its own resistor):

| Button | Resistor to GND | ADC target |
|--------|------------------|------------|
| SELECT | 0 Ω (direct)     | ~0         |
| BACK   | 4.7 kΩ           | ~312       |
| UP     | 8.2 kΩ           | ~476       |
| DOWN   | 18 kΩ            | ~658       |

`VCC` → 3.3 V; `GND` → common ground.

**Backlight is switched, not hard-wired.** The LED pin still feeds through the
same ~330 Ω resistor, but that now goes to the collector of **Q1**, a small PNP
transistor (e.g. 2N3906) high-side-switching the backlight's +3.3 V feed instead
of tying straight to it: emitter → +3.3 V, collector → the 330 Ω resistor → LCD
`LED` pin, base → a ~4.7 kΩ resistor → Nano **A4**. This is a **negated**
control: pulling A4 **LOW** forward-biases Q1 so it conducts and the backlight
turns **ON**; driving it **HIGH** turns it **OFF** (`LCD_BACKLIGHT_ON`/
`LCD_BACKLIGHT_OFF` in `display.h`). The sketch turns it on for any button press
and off again after a configurable idle timeout — see "Backlight" under
"Using it" below.

### DHT11 sensor

| DHT11 pin | Arduino |
|-----------|---------|
| VCC       | 5 V     |
| DATA      | **D8**  |
| GND       | GND     |

Add a 10 kΩ pull-up between DATA and VCC (many breakout boards already have it).
*(DATA is on D8, not D2, so the hardware-interrupt pins D2/D3 stay free for the tachs.)*

### Fans (4-wire)

| Fan wire         | Left fan | Right fan |
|------------------|----------|-----------|
| PWM (blue)       | **D9**   | **D10**   |
| TACH (green)     | **D2**   | **D3**    |
| +12 V (yellow)   | +12 V    | +12 V     |
| GND (black)      | GND      | GND       |

- **PWM wire → D9/D10 directly.** The Nano's 5 V push-pull drives the fan's PWM
  input fine (per the Intel 4-wire spec, target 25 kHz).
- **TACH wire → D2/D3**, each with a 10 kΩ pull-up to +5 V.
- **Common ground:** tie Nano GND, the 12 V fan supply GND, and the battery GND
  together.

### 12 V SLA battery voltage sense

A resistor divider scales the battery voltage down into the Nano's 0–5 V ADC range,
read on **A7** (analog-input-only, like A6 for the buttons):

```
Battery (+) ──[ R1 100k ]──┬──────── A7
                            │
                          [ R2 33k ]
                            │
Battery (–) / GND ─────────┴──────── GND
```

With 100 kΩ/33 kΩ, max measurable voltage is ~20 V — well above any realistic SLA
charge voltage (typically ≤ 14.8 V), so normal readings stay safely inside range.
Change `BATTERY_R1`/`BATTERY_R2` in `battery.h` if you use different
resistors (the volts-per-ADC-count conversion is derived from them automatically).

---

## How the fan control works

Every 2 seconds the DHT11 is read and the fan speed is recomputed. Temperature and
humidity are treated symmetrically — **whichever is more demanding wins.**

**Oversampling:** the DHT11 itself only has whole-degree/whole-percent resolution
and can't be re-sampled faster than its own ~1-2s minimum interval, so instead of
acting on each raw reading directly, readings are folded into a running sum/count
(no need to keep the individual samples - see `Sensor::read()` in `sensor.h`) and
averaged over a `DHT_OVERSAMPLE_COUNT`-sample (default 8, ~16s at the 2s sampling
rate) window; the sum/count reset once that many samples have accumulated, so
averaging starts fresh for the next window. This smooths out sensor noise/
quantization into a more precise effective value — control-law comparisons, the
graphs, and the status screen (which now shows humidity to 0.1%, matching
temperature's existing 0.1°C) all use this averaged value. The trade-off is a few
seconds of extra lag before the fans react to a genuinely fast change (negligible
against a shed's thermal mass), plus a small periodic step back to just the
newest raw sample right after each window reset, before it refines again. A dead
sensor doesn't feed bad data into the average — see the fail-safe below.

For each quantity there are two thresholds (defaults in `config.h`, editable at
runtime and stored in EEPROM):

| Quantity    | `START` (begin airing) | `HIGH` (full speed) |
|-------------|------------------------|---------------------|
| Temperature | 25 °C                  | 32 °C               |
| Humidity    | 65 %RH                 | 85 %RH              |

Fan speed (**target RPM**, not duty — see "Closed-loop RPM control" below) as a
function of the *more demanding* reading:

```
  RPM
 HIGH ┤                                   ┌────────   ← noisy, only at/above HIGH
      │                                   │
  MED ┤                          ┌────────┘
      │                    ┌─────┘   quiet band: LOW…MED ramps
  LOW ┤          ┌─────────┘        proportionally with the reading
      │          │
  OFF ┼──────────┘
      └──────────┬──────────────────────┬──────────►  temp / humidity
               START                   HIGH
```

- **Below both START thresholds → fans OFF.**
- **Between START and HIGH → fans ramp LOW…MED RPM** (the quiet band), proportional to
  how far past START the stronger reading is.
- **At or above either HIGH threshold → fans go to HIGH RPM.** This is the *only*
  condition that accepts the noise and extra wear.

Targets come from `FanControl::getLowRpm()`/`getMedRpm()`/`getHighRpm()` in
`fan_control.h` — not fixed constants, but **derived from autotuning your
actual fans**: each is the lower of both fans' own autotune-measured RPM at
the LOW/HIGH/MAX duty points, see "PID tuning and autotune" and "Closed-loop
RPM control" below.

### Closed-loop RPM control (PID)

The control law above decides a **target RPM**, not a PWM duty. Each fan is
driven by its own `FanPIDControl` object (`fan_pid_control.h`), which holds a
reference to the `Fan` (`fan.h`) whose hardware it drives, with its own PID
loop (`FanPIDControl::runPid()`);
`FanControl::driveAuto()` (`fan_control.h`, called every cycle from
`ControlLoop::service()` in `control_loop.h`) simply calls it on both
`fanLeft`/`fanRight` with the same target RPM. Each call reads that fan's
*actual* tachometer speed and adjusts its own PWM duty to close the gap —
independently per fan, so a left/right RPM mismatch (fan wear, manufacturing
variance, wiring resistance) is corrected automatically rather than assuming
identical fans.

This matters because PWM-to-RPM is a nonlinear, fan- and voltage-specific curve —
open-loop duty control means "LOW" could be a different actual airflow on the left
fan than the right, or drift over months as bearings wear. Closed-loop RPM control
means "LOW" reliably means the same airflow, on both fans, for the life of the
hardware.

- **Gains**: independent Kp/Ki/Kd **per fan** (each `FanPIDControl` instance owns its own,
  not shared), tightly coupled to your RPM targets — re-tune (or re-autotune)
  after changing them. Editable live from the **PID tuning** menu, or set
  automatically by **autotune** — see "PID tuning and autotune" below.
  `FAN_PID_KP`/`FAN_PID_KI`/`FAN_PID_KD` in `fan_pid_control.h` are only the power-up
  default for a fan before it's been tuned. `FAN_PID_KD` defaults to `0` (a
  PI controller): RPM is only updated once every 2s, and a derivative term
  reacting to that coarse a signal tends to do more harm than good if
  hand-tuned carelessly - autotune computes a small, conservative Kd
  automatically instead (see "PID tuning and autotune" below).
- **Voltage compensation**: the 12V SLA battery this sketch is built around
  isn't actually constant (~10.5V near-empty to ~14.8V on charge), and
  PWM-to-RPM for a fixed duty scales roughly linearly with supply voltage,
  the same way a plain DC motor's speed does - so gains tuned at one voltage
  drift out of tune as the battery charges/discharges. Every time a fan's
  Kp/Ki/Kd are set (by hand or by autotune), the live battery voltage is
  recorded alongside them as `FanPIDControl::tuningVoltage`; `FanPIDControl::runPid()` then
  scales the stored gains by `tuningVoltage ÷ (the live battery voltage)`
  every cycle, so the effective gains track the battery's actual voltage
  rather than only being correct at whatever voltage they happened to be
  tuned at.
- **Anti-windup**: the integral term only accumulates when doing so wouldn't push
  the output past the 0–100% duty range, so a target the fan physically can't reach
  yet (e.g. right after kick-start) can't wind up unboundedly.
- **Manual/test mode bypasses the PID entirely** — it commands duty directly, useful
  for servicing/diagnostics (e.g. checking the RPM a given duty produces by
  hand). Finding your fans' LOW/MED/HIGH RPM targets no longer needs this
  though - "Tune Left"/"Tune Right" measures them automatically as part of
  autotuning, see "PID tuning and autotune" below.

Extra behaviour (unchanged from before, now RPM-driven rather than duty-driven):

- **Hysteresis** (2 °C / 5 %RH): once the fans start, or once HIGH engages, the
  reading must fall back past the threshold by this margin before they step down — no
  chattering on a threshold.
- **Kick-start**: starting from a standstill the fans are pulsed at full duty for
  ~300 ms (open-loop — the PID has no RPM feedback yet at that instant) so a low
  target RPM can still get them spinning, before the PID takes over.
- **Tachometer / fan-fail detection**: real RPM of each fan is shown on the display;
  if a fan is commanded to run but reads below its own *measured* stall RPM
  (from **Tune Left**/**Tune Right** - see "PID tuning and autotune" below,
  not a fixed guess) it shows `STALL`. This is even more meaningful now than
  under open-loop control: a persistent stall means the PID has already wound
  the duty up trying to reach the target and the fan still isn't responding —
  a strong signal of a real mechanical fault. Before a fan has ever been
  autotuned, its stall RPM is uncalibrated and `STALL` never shows for it —
  run **Tune Left**/**Tune Right** once per fan to enable this.
- **Both fans are always commanded to the identical target RPM** (never separate
  per-fan targets — see `FanControl::driveAuto()` in `fan_control.h`), so under normal operation their measured
  speeds converge to match each other, not just the setpoint. If they nonetheless
  disagree by more than `FAN_RPM_MISMATCH_RPM` (default 300) for
  `FAN_MISMATCH_DEBOUNCE_CYCLES` (default 3, i.e. 6s sustained) — a real fault the
  PID couldn't correct — the Fans screen's title swaps to `MISMATCH!`. Not checked
  in Manual mode, where duty (not RPM) is the same on both fans by design and a
  natural RPM difference is expected, not a fault.
- **Sensor fail-safe**: after 5 consecutive failed reads the display shows
  `SENSOR FAIL` and the fans hold a safe, quiet **MED RPM** target (`FanControl::getMedRpm()`)
  so the shed keeps airing.

### PID tuning and autotune

The **PID tuning** menu has independent **Kp**/**Ki**/**Kd** sliders for each fan
(`Kp Left`, `Ki Left`, `Kd Left`, `Kp Right`, `Ki Right`, `Kd Right`), plus a
**Tune Left**/**Tune Right** command that sets Kp/Ki automatically instead of
hand-tuning:

- Stops both fans, then runs the target fan **open-loop** (the other fan stays
  off). First it sweeps duty *downward* from `FAN_AUTOTUNE_LOW_PCT` (40%) in
  5%-point steps, holding each for `FAN_AUTOTUNE_SETTLE_CYCLES` (4) control
  cycles (~8s) to reach steady RPM, until the fan actually stops spinning
  (measured RPM reads 0) or duty bottoms out - the last non-zero reading
  becomes this fan's own **measured stall RPM**, not a fixed guessed constant.
  The very first step of that sweep (at `FAN_AUTOTUNE_LOW_PCT` itself) doubles
  as the "low" measurement below. It then tests `FAN_AUTOTUNE_HIGH_PCT` (90%)
  and `FAN_AUTOTUNE_MAX_PCT` (100%) as two more fixed-duty points, the same
  way. This is "running the fan at different speeds" to characterize it, not
  the normal closed-loop control.
- From the **low**/**high** (duty%, RPM) points it computes the fan's
  **process gain** `K = ΔRPM / Δduty%`, then derives `Kp = 1/K`,
  `Ki = Kp / T`, and `Kd = Kp · T / 4`, where `T` is taken as
  `CONTROL_INTERVAL` (2s) - the fastest this control loop can actually react,
  and comfortably longer than a PC fan's true mechanical settling time, so
  measuring an explicit time constant at 2s sampling resolution wouldn't be
  meaningful anyway. Kd's `T/4` is a conservative fraction of that same
  constant, chosen because a coarse 2s-updated RPM signal amplifies noise if
  the derivative term is too aggressive - see the note on Kd above.
- The **max** point (100% duty) doesn't feed into Kp/Ki/Kd either - it's this
  fan's actual RPM ceiling, feeding `FanControl::getHighRpm()` (the lower of
  both fans' ceilings - see "Control philosophy" above) without a separate
  trip to Manual mode.
- The measured **stall RPM**, **low/high RPM** (feeding `getLowRpm()`/
  `getMedRpm()`), and **max RPM** are all saved per fan (alongside
  Kp/Ki/Kd), along with the live battery voltage the whole run was measured
  at (taken once, right after the fans stop, so it's a resting reading, not
  skewed by load). Since PWM-to-RPM scales roughly with supply voltage, both
  are scaled by `(live voltage) / (that saved voltage)` whenever they're
  actually used later - same idea as the Kp/Ki/Kd voltage compensation above.
- Takes roughly a minute per fan (kick-start + the downward sweep + two more
  ~8s settle steps - longer than before, since the sweep now measures the
  real stall point instead of assuming one), showing which duty is currently
  under test, then the resulting `Kp`/`Ki`/stall RPM/max RPM (or
  `Tune FAILED` if the fan never spun fast enough at the low point relative
  to its own measured stall RPM, or the high point wasn't meaningfully faster
  than the low one - check the wiring before retrying) until you press **OK**.
- Whichever fan you tune, only that fan's gains/stall RPM/max RPM change -
  the other fan's are untouched, so a genuinely different pair of fans
  doesn't need identical gains.
- The result is a reasonable, principled starting point, not guaranteed optimal -
  feel free to hand-tune Kp/Ki/Kd from there with the sliders (see the tuning
  tips above); stall RPM and max RPM aren't hand-editable, only set by autotune.

---

## Using it

On power-up the **status screen** cycles through six home screens with the
bottom-left button (**Select**, labeled `Next`); **Back** (`Menu`) always opens the
menu regardless of which one is showing:

**1. Climate** — temperature, humidity, battery voltage (`LOW!` below
`BATTERY_LOW_VOLTAGE` — SLA batteries are damaged by deep discharge), and battery
charge status (resting-voltage percentage estimate — fan status now lives only on
the Fans screen). The title line reads `Shed airing`, or `SENSOR FAIL` if the DHT11
is down:
```
 Shed airing
Temp:            24.6C
Humid:           58.4%
Batt:            12.6V
Charge:           88%
[Next        Menu]
```

**2. Battery graph** — 48-hour voltage history, with a labeled, auto-scaled y-axis
(min/max voltage) and x-axis (time span):
```
Battery 48h
13.1|    ╭─╮      ╭──
    |    ╯  ╲____╱
12.6|________________
   -48h            now
[Next        Menu]
```

**3. Charge graph** — 48-hour history of the same battery voltage samples as the
Battery graph, but each point translated through the charge curve
(`BatteryCharge::percentFromVoltage()` in `battery_charge.h`) so the y-axis reads directly in
charge % instead of raw voltage. See "Battery charge curve" below:
```
Charge 48h
  92|    ╭─╮      ╭──
    |    ╯  ╲____╱
  78|________________
   -48h            now
[Next        Menu]
```

**4. Temperature graph** and **5. Humidity graph** — 48-hour history, same axis
treatment:
```
Temp 48h
  24|   ___╱‾‾╲___╱‾╲
    |
  18|________________
   -48h             now
[Next        Menu]
```

Both axes autoscale to whatever data is currently in the buffer — the y-axis labels
the min and max of the plotted (smoothed) values. There's only one time scale
(48h) for all four graphs (temperature, humidity, battery voltage, battery charge);
a light fixed smoothing pass is applied on top of the underlying bucket averaging
(see below).

**6. Fans** — fan speed and each fan's RPM individually (`STALL` if a fan is
commanded on but not spinning fast enough). In automatic mode the speed label is the
RPM band the PID is targeting (`STALL` aside, RPM shown is the *actual* measured
speed, which should track close to that target); in manual mode it's the literal
commanded duty since there's no RPM target to show. Manual mode adds a trailing `*`
either way. The title swaps to `MISMATCH!` if the two fans' RPM sustainedly
disagree despite being commanded to the same target — see "Closed-loop RPM control"
above:
```
       Fans
       MED*

Left:            1180
Right:           1210
[Next        Menu]
```

Press **Menu** (the Back button) to open the menu:

- **Thresholds** — sliders for the four numbers above (`start` is always kept below
  `high`).
- **Manual / test** — toggle **Manual** mode and set a **Fan speed %** directly,
  bypassing the PID entirely; while Manual is on the slider previews the speed live
  so you can test the fans by ear / by RPM (Fans home screen) - handy for
  servicing/diagnostics, though finding the LOW/MED/HIGH RPM targets
  themselves is now autotune's job (see below), not something to do here by
  hand. When Manual is on, the Fans screen's speed label gets
  a trailing `*`.
- **PID tuning** — Kp/Ki/Kd sliders for each fan, plus **Tune Left**/**Tune Right**
  to set Kp/Ki/Kd and the fan's LOW/MED/HIGH RPM contribution automatically.
  See "PID tuning and autotune" above.
- **Set contrast** — LCD contrast.
- **Backlight s** — the backlight's idle timeout, in seconds (default 30). See
  "Backlight" below.
- **Batt calib** — the battery voltage calibration slider, see "Battery voltage
  calibration" below.
- **Charge curve** — recalibrate the voltage↔charge% curve, see "Battery charge
  curve" below.

There is no standalone "Graphs" menu item — the temperature, humidity, battery
voltage and battery charge graphs are only reached as home screens 2-5, and the
live battery voltage (with its low-battery warning) is on the Climate home screen.

All threshold/manual/contrast/backlight-timeout settings persist in EEPROM
across power cycles. **History graphs do not persist** — they're RAM-only and
rebuild from zero after every power cycle or reset (see below).

### Backlight

The LCD backlight (see the Q1 switch in "Wiring" above) turns **on** whenever any
button is pressed, and **off** again after `backlightTimeout` seconds of no
further button activity (default 30, editable at runtime via **Backlight s** in
the menu and stored in EEPROM). It's also turned back on whenever the menu is
entered or exited, so it doesn't go dark mid-navigation. The timeout is only
tracked while the live status screen is showing — the same as the fan control
law itself (`ControlLoop::service()`), neither runs while a menu or slider is
open, since that loop is what drives both.

---

## History storage

Temperature, humidity and battery voltage each get their own rolling 48-hour
`History` (`history.h`) - a generic single-channel ring buffer of
`HISTORY_POINTS` (24, in `control_loop.h`) points, a good match for the 84px-
wide display since finer resolution wouldn't be visible anyway. Each point is
a running average of the raw samples that fell in its ~2-hour bucket. All
three are owned by **`ControlLoop`**, not by `Sensor`/`Battery` - `service()`
already reads every value every cycle, so it feeds all three `History`
instances directly from those live readings (no multi-scale cascade — an
earlier version of this sketch had 24h/Week/Month/Year scales for temperature
and humidity, but that was removed in favor of a single 48h view matching the
battery graph):

```
raw samples (every 2s) → average every ~2h → 48h buffer (24 points)
```

Values are stored as single bytes (whole °C, whole %RH, tenths-of-a-volt for
battery), so all three buffers together cost well under 150 bytes of RAM.

This is **RAM-only**: all history resets on power loss or reset (an external
EEPROM or flash chip would be needed to survive power cycles, which this build
doesn't include) - all three `History` instances are cleared in
`ControlLoop::init()`, called from `setup()`. A dead DHT11 pauses temperature/
humidity history rather than polluting it with stale readings; battery
voltage is always sampled regardless.

Graphs apply a light, fixed additional smoothing pass (`GRAPH_SMOOTH_WINDOW` in
`graphs.h`) on top of the bucket averaging.

---

## Battery charge estimate — important caveat

The Climate screen's `Charge:` percentage (and the Charge graph home screen) is a
rough state-of-charge estimate from a resting-voltage curve (`BATTERY_SOC_TABLE` in
`battery_charge.h`, recalibratable at runtime - see "Battery charge curve" below). **It is
only meaningful with no significant charge or load current flowing** — there's no
current sensing, so while a charger is active the voltage reads high (overstating
charge), and under heavy load it reads low (understating it). Treat it as a coarse
gauge, not a fuel-gauge measurement. The `LOW!` warning threshold
(`BATTERY_LOW_VOLTAGE`, default 11.5 V) is set with margin above the ~10.5 V
deep-discharge cutoff that damages SLA batteries.

---

## Battery voltage calibration

Resistor tolerance and the Nano's actual 5 V rail (which isn't exactly 5.000 V)
mean the ADC-derived voltage can be off by a volt or more from the divider math
in `battery.h`. Both of those errors are *multiplicative* (they scale the whole
divider ratio or the ADC reference, not shift the result by a fixed amount), so
calibration works on the ADC-counts-to-volts scalar itself
(`Battery::voltsScalar`, default `BATTERY_VOLTS_SCALAR_DEFAULT`) rather than an
added offset. Rather than needing a programmer to fix this, it's calibrated live
from the device: **Batt calib** in the menu. The slider shows the current live
voltage, seeded from that same reading — put a multimeter on the battery
terminals and drag the slider until it matches, then **Set**, which rescales
`voltsScalar` proportionally (`newScalar = oldScalar × target ÷ reading-when-
opened`) so the same raw ADC reading now yields exactly the target voltage. The
result is stored in EEPROM (`EEPROM_BATTERY_VOLTS_SCALAR_IDX`) and applied to
every subsequent reading: the Climate screen, the battery graph, the `LOW!`
warning and the charge-percent estimate all use the calibrated value. The
slider's range is the current reading ± `BATTERY_CALIBRATION_RANGE` (default 5 V).

---

## Battery charge curve

`BATTERY_SOC_TABLE` in `battery_charge.h` maps voltage to charge % at 7 fixed points
(100/80/60/40/20/10/0%) - but the resting voltage for a given charge level varies
by battery chemistry, age and temperature, so the defaults are only a starting
point. **Charge curve** in the menu opens a sub-menu with one slider per point
(`100%` .. `0%`); each shows that point's *voltage* (range `BATTERY_SOC_VOLTAGE_MIN`
– `BATTERY_SOC_VOLTAGE_MAX`, default 9.0–15.0 V) and lets you recalibrate it - e.g.
against a smart charger's own SOC estimate, or a manufacturer discharge table for
your specific battery. Only the voltage at each point is adjustable; the percent
values themselves stay fixed. Each point persists to EEPROM independently
(`EEPROM_SOC_CURVE_BASE_IDX`) and takes effect immediately - both the Climate
screen's `Charge:` percentage and the Charge graph home screen read the same
live curve, so there's no separate step to "apply" a change.

The menu doesn't enforce that the 7 points stay in descending voltage order as
you edit them (100% should always be set to a higher voltage than 80%, and so on)
- keep them monotonic yourself, the same way you'd fill in a lookup table.

---

## Tuning

Every tunable constant lives with the subsystem it configures now (see
"Files" above) rather than all in one place - each file's own comments
explain what to change and why:

**`fan.h`** (one fan's own raw hardware) - the same constants apply to both
`fanLeftHw` and `fanRightHw`, since it's one `Fan` class instantiated twice:
- Pins `FAN_PWM_LEFT_PIN`/`FAN_PWM_RIGHT_PIN`, `FAN_TACH_LEFT_PIN`/`FAN_TACH_RIGHT_PIN`
  — passed into the `fanLeftHw`/`fanRightHw` `Fan` constructors in `FanController.ino`.
- `FAN_TACH_PULSES_PER_REV`.
- `FAN_KICKSTART_MS` — how long `kickstart()`'s open-loop full-duty pulse lasts.
- `SPEED_OFF_PCT`/`SPEED_HIGH_PCT` — the duty range's named endpoints (0/100).
- No stall-RPM constant here anymore - each fan's stall RPM is measured by
  autotune and stored per-fan in `fan_pid_control.h`'s `FanPIDControl`, not
  a fixed value in this file - see below.

**`fan_pid_control.h`** (one fan's own PID gains and autotune) - the same constants apply to
both `fanLeft` and `fanRight`, since it's one `FanPIDControl` class instantiated twice:
- Default PID gains `FAN_PID_KP`/`FAN_PID_KI`/`FAN_PID_KD` — re-tune (or
  re-autotune) after changing the RPM targets below; the per-fan sliders/
  autotune in the "PID tuning" menu are usually more convenient than editing
  these. `FAN_PID_KP_MIN`/`_MAX`/`_STEP` (and the `_KI_`/`_KD_` equivalents)
  set that menu's slider ranges.
- `FAN_AUTOTUNE_LOW_PCT`/`_HIGH_PCT`/`_MAX_PCT`/`_SETTLE_CYCLES` — the fixed
  test duty points (the downward stall-sweep starts at `_LOW_PCT` and needs
  no separate constant for where it stops - it stops when the fan actually
  stalls) and the per-point settle time autotune uses, see "PID tuning and
  autotune" above.
- `FAN_VOLTAGE_SCALE_MIN`/`_MAX` — the same sanity clamp `runPid()` uses on
  the Kp/Ki/Kd voltage-compensation ratio also bounds the stall/low/high/max
  RPM voltage compensation in `isStalled()`/`getLowRpm()`/`getHighRpm()`/
  `getMaxRpm()`.
- `FAN_LOW_RPM_DEFAULT`/`FAN_HIGH_RPM_DEFAULT`/`FAN_MAX_RPM_DEFAULT` — this
  fan's power-up default RPM at the LOW/HIGH/MAX autotune duty points,
  before it's ever been autotuned - same role as `FAN_PID_KP`/`KI`/`KD`
  above, just for RPM. Match the *old* fixed `FAN_LOW_RPM`/`FAN_MED_RPM`/
  `FAN_HIGH_RPM` values exactly, so an un-autotuned pair of fans behaves the
  same as before.

**`fan_control.h`** (pair-level RPM targets and mismatch detection):
- `getLowRpm()`/`getMedRpm()`/`getHighRpm()` - **not tunable constants
  anymore** - each is the lower of both fans' own autotune-measured RPM (see
  `fan_pid_control.h` above), so run **Tune Left**/**Tune Right** on both
  fans to actually tune these; see "Closed-loop RPM control" above.
- `FAN_RPM_MISMATCH_RPM` / `FAN_MISMATCH_DEBOUNCE_CYCLES` — how far apart (RPM) and
  for how long (control cycles) the two fans' measured speeds may disagree before
  the Fans screen flags `MISMATCH!`, see "Closed-loop RPM control" above.
- `EEPROM_FAN_LEFT_BASE_IDX`/`EEPROM_FAN_RIGHT_BASE_IDX` — see "EEPROM layout" below.

**`battery.h`** (pins, divider, calibration):
- Pin `BATTERY_ADC_PIN`; `BATTERY_R1`/`BATTERY_R2` (divider resistors) feed
  `BATTERY_VOLTS_SCALAR_DEFAULT`, the power-up default for the runtime-calibrated
  `Battery::voltsScalar` — see "Battery voltage calibration" above;
  `BATTERY_LOW_VOLTAGE`.
- `BATTERY_CALIBRATION_RANGE`/`BATTERY_CALIBRATION_STEP` (span and step size of the
  "Batt calib" slider — see "Battery voltage calibration" above).

**`battery_charge.h`** (charge curve):
- `BATTERY_SOC_TABLE` (default voltage→percent curve - recalibrate at runtime via
  the "Charge curve" menu instead of editing this directly, see above),
  `BATTERY_SOC_VOLTAGE_MIN`/`_MAX`/`_STEP` (range/step of the Charge curve sliders).
- `EEPROM_SOC_CURVE_BASE_IDX` — chains off `battery.h`'s
  `EEPROM_BATTERY_VOLTS_SCALAR_IDX`, see "EEPROM layout" below.

**`sensor.h`** (DHT11):
- Pin `DHT_PIN`; `DHT_SENSOR_FAIL_LIMIT`, `DHT_OVERSAMPLE_COUNT` (temp/humidity
  averaging window - higher is smoother but slower to react).

**`display.h`** (LCD/buttons, backlight):
- Pins `LCD_CLOCK_PIN`/`LCD_DATA_IN_PIN`/`LCD_DATA_CONTROL_PIN`/`LCD_CHIP_ENABLE_PIN`/
  `LCD_BUTTONS_PIN`/`LCD_BACKLIGHT_PIN`.
- `LCD_BACKLIGHT_TIMEOUT_DEFAULT`/`_MIN`/`_MAX`/`_STEP` (default and slider range
  for the "Backlight s" menu setting), `LCD_BACKLIGHT_ON`/`LCD_BACKLIGHT_OFF`
  (only change these if you rewire Q1 as a non-negated switch) — see "Backlight"
  above.

**`control_loop.h`** (history span, climate defaults):
- `TEMP_HUMID_HISTORY_HOURS`/`BATTERY_HISTORY_HOURS` (how far back each
  `History` graph reaches - see "History storage" above).
- `DEFAULT_TEMP_START`/`_HIGH`/`DEFAULT_HUM_START`/`_HIGH` (power-up climate
  thresholds - see "Thresholds" in the menu for the runtime-editable copies),
  `TEMP_HYSTERESIS`/`HUM_HYSTERESIS`.
- `EEPROM_TEMP_START_IDX` etc. — see "EEPROM layout" below.

**`config.h`** (cross-cutting/shared):
- `CONTROL_INTERVAL` - the control loop's own cadence; used everywhere (DHT11
  read rate, PID timing, history bucket sizing). `HISTORY_POINTS` (shared
  resolution for all three `History` buffers - see "History storage" above).
  Both live here rather than with `control_loop.h` (which otherwise owns this
  cadence/sizing conceptually) because `fan.h`/`fan_control.h`/`history.h`
  need them and are included *before* `control_loop.h` in `FanController.ino`
  - see the comment at the top of `config.h`.
- The EEPROM address book: `EEPROM_LCD_BASE_IDX`/`EEPROM_CONTROL_BASE_IDX`/
  `EEPROM_BATTERY_BASE_IDX`/`EEPROM_FAN_BASE_IDX` — see "EEPROM layout" below
  before touching these.

### EEPROM layout

Each subsystem file defines its own named `EEPROM_..._IDX` constants (and is
the only file that ever uses them) as offsets from a reserved block base -
`EEPROM_LCD_BASE_IDX` etc. in `config.h`. Those four base addresses are the
one thing that has to stay centralized and coordinated: they're spaced out
generously (16-40 bytes per block, only 7-24 of which are actually used
today) so each subsystem can add settings later without needing to renumber
anything else - which is what guarantees no two subsystems' addresses ever
collide on the Nano's single shared EEPROM. If you add enough per-subsystem
settings to overflow a block, move the *following* bases further out (and
bump `EEPROM_NEXT_FREE_BASE_IDX` if you add a 5th block) rather than
shrinking the gaps.

Most subsystems still persist each setting as its own named field via the
ArduinoUserInterface library's `readConfigurationByte()`/`Int()`/`Long()`
helpers (a 1-byte "never written" flag plus the value, per field). `FanPIDControl`
(`fan_pid_control.h`) instead uses a more generic mechanism, added specifically so two
instances of the same class could each get independent persisted state
without hand-numbering per-field addresses per side: its entire persisted
state - the three PID gains, the battery voltage they (and the stall/max
RPM below) were last calibrated at, and this fan's own measured stall RPM
and max RPM from autotune (see "PID tuning and autotune" above) - is one
plain struct (`PersistedGains`, whose member initializers double as the
power-up defaults), and `Module::loadState()`/`saveState()` (`module.h`)
move that whole struct to/from EEPROM as a single block via
`EEPROM.get()`/`put()`, at a base offset passed into the `FanPIDControl`
constructor - one raw flag byte (the same "0xFF = never written" convention
as the library's own helpers) plus `sizeof(PersistedGains)` (20 bytes: four
floats plus two `unsigned int`s). `fanLeft`/`fanRight` are constructed with
`EEPROM_FAN_LEFT_BASE_IDX`/`EEPROM_FAN_RIGHT_BASE_IDX`
(`fan_control.h`, 24 bytes apart within the `EEPROM_FAN_BASE_IDX` block - a
little headroom over the 21 actually used) and are otherwise identical `FanPIDControl`
objects - the two instances' state never collides simply because they were
constructed with two different offsets. Any subsystem could adopt the same
`loadState()`/`saveState()` pair for its own settings; the others just
haven't needed to.

> **Timer note:** the sketch reprograms **Timer1** for 25 kHz PWM, so `analogWrite()`
> on pins 9 and 10 no longer works — write those fans only through the sketch. Timer0
> (`millis`, `delay`) and Timer2 are untouched.
