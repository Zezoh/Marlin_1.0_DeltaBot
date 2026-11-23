# Marlin 1.0 - Delta Firmware

Marlin 1.0.2-3 firmware reduced to a clean, English-only build for **delta
(Kossel / Rostock style) 3D printers**.

## What is included

- Delta kinematics (`calculate_delta`, tower angle corrections, endstop adjustment, M665/M666)
- English-only serial messages (`Marlin/language.h`)
- RAMPS 1.3 / 1.4 boards only (ATmega2560 / ATmega1280); no LCD or panel support
- FSR Z probe (G30), delta auto-calibration (G33), bed mesh (G29), one button, status LED, filament change (M600) and runout sensor, ported from the DeltaCore project (see below)
- Thermal runaway protection and the hardware watchdog (4 s) enabled by default
- SD card printing enabled by default (FAT16/FAT32 via `Sd2Card` + `SdFat`, M20-M32, M928, autostart `auto0.g`...`auto9.g`)
- Settings storage (M500/M501) always on, including the delta geometry (M665/M666)
- Delta default configuration in `Marlin/Configuration.h` and `Marlin/Configuration_adv.h`

## Printer setup (from DeltaCore)

| | |
|---|---|
| Geometry | rod 210 mm, delta radius 90 mm, home height 225 mm, printable radius 85 mm |
| Drive | 80 steps/mm on the towers (GT2, 20 teeth, 1/16), 98 steps/mm extruder |
| Limits | tower speed 180 mm/s, acceleration 8000 mm/s2 (2500 for printing moves), homing 120 mm/s then 8 mm/s |
| Hotend | TDK 104NT-4 thermistor (table 104), PID 32.48 / 6.4 / 41.25, max 285 degC |

Pins (RAMPS 1.4, the same as DeltaCore): tower endstops on the MAX sockets (D2, D15, D19), hotend heater D10,
fan D9, thermistor A13, **FSR on A14** (the RAMPS bed thermistor input), **button on D3** (X-MIN socket, to GND),
**status LED on D12** (the PS_ON header, so there is no ATX power control: M80 does nothing).
All values are in `Marlin/Configuration.h`, `Marlin/Configuration_adv.h` and `Marlin/pins.h`.

## FSR Z probe, G30 and G33

The nozzle is the probe: a force sensing resistor under the bed reports contact. A probe stroke goes down at 5 mm/s and
the stepper interrupt stops it as soon as the reading moves by `FSR_SENSITIVITY` counts (default 12) from its value
at the start of the stroke. A point is the average of three strokes that agree within 0.05 mm. A noisy sensor raises its
own threshold.

| Code | |
|---|---|
| `G30` | probe at the current X/Y and print the bed height |
| `G29 [S0]` | bed mesh: probes 19 points (12 at 80 mm, 6 at 40 mm, the centre) and, if the bed is within 5 mm of Z 0, switches the mesh on and saves it (S0: do not save). Z is then corrected by the interpolated bed height. Run G33 first. A failed G29 keeps the old mesh. |
| `M420 [S0\|1] [Z<mm>]` | mesh on/off without moving the nozzle; Z = height where the correction has faded out (0 = never) |
| `G33 [F3\|4\|6] [P7\|10] [S0]` | least-squares delta calibration: probes P points, fits F factors (3 = endstops, 4 = + delta radius, **6 = + tower angles, default**), applies the result, re-homes and probes again, up to 6 passes. It ends calibrated (RMS <= 0.05 mm), noise limited (<= 0.25 mm) or uncertified. S0 does not save to EEPROM. A result outside the allowed range is refused and the old settings are restored. |
| `M974 [S<counts>]` | FSR sensitivity |
| `M851 [Z<mm>]` | nozzle height above the bed when the probe fires (default 0) |
| `M665 A B C` | tower angle corrections in degrees (set by G33) |

The endstop trims G33 finds are stored in `M666` (a delta can only lower a carriage after the endstop), the rest of the
height change goes into the home offset (`M206 Z`). Editing M665/M666/M206 by hand clears the "calibrated" flag. A G33 that changes the geometry also drops the mesh (run G29 again). G92 only sets E on a delta; X, Y and Z are the machine's.

## One button

Pressed to GND on D3, the LED on D12 shows the state (blink period: error 0.2 s, filament change working 0.4 s, busy 0.8 s,
paused or waiting for you 1 s, printing 1.2 s, heating 2 s, cooling 3 s, ready 4 s).

| State | Click | Double click | Long press (1.5 s) |
|---|---|---|---|
| idle, not calibrated | G33 (saves) | filament change | heater off |
| idle, calibrated | print the newest G-code file in the SD root (no filament: load it first) | filament change (load only if there is none) | heater off |
| printing | pause | | cancel the print, heater off, home |
| paused | resume (not without filament) | filament change, a click resumes the print | cancel the print |
| filament change waiting | go on (load the new filament / confirm) | | cancel |
| homing / probing / G33 | ignored | ignored | cancel |
| held at power-on (2.5 s) | | | calibrate (G33) once |

The button does nothing while a host (OctoPrint, Pronterface...) is printing, except the long press that turns the heater off.

## Filament change (M600) and the runout sensor

The sensor goes to D14 (the RAMPS Y-MIN socket, input with pullup, HIGH = no filament, see `RUNOUT_*` in
`Configuration_adv.h`). It must read "no filament" for 0.5 s and "filament" for 0.75 s before it counts.

`M600` (or the sensor, or a double click) does this inside one command, like M109:

1. wait for the moves to end, pull back 3 mm, lift the nozzle 5 mm (at least to Z 30) and park it at X0 Y-60
2. heat to 210 degC (or the printing temperature if that is higher) and unload 745 mm
3. wait for you: pull the old filament out, push the new one in, **click** (or `M701`). The click only counts when the
   sensor has seen the old filament leave and sees the new one
4. load 10 mm slowly, 740 mm fast, purge 5 mm, wait for another **click** (or `M24`) to confirm
5. heat to the printing temperature, return to the print, push the 3 mm back, restore the extruder position and go on

A **long press** (or `M108`) cancels: heater off, an SD print is cancelled and the printer homes. If nobody comes within
2 minutes the heater goes off by itself; the nozzle is heated again before the filament moves.
`M412 S0|1` turns the sensor off and on (until the next reset). The runout starts a change only while an SD print or a
host print is running and something was extruded.
M600 needs a homed machine; with nothing printing the double click homes first, goes to the centre (Z 30), and loads only
if the sensor sees no filament.

## Building

Arduino IDE: open `Marlin/Marlin.ino`, select an *Arduino Mega 2560* (or 1280) board.

PlatformIO:

```bash
cd PlatformIOAddons && platformio run
```

Make (avr-gcc + an Arduino core): `cd Marlin && make ARDUINO_INSTALL_DIR=<arduino dir> HARDWARE_MOTHERBOARD=33`

Select your electronics with `MOTHERBOARD` in `Marlin/Configuration.h`
(board ids are listed in `Marlin/boards.h`) and tune the `DELTA_*` values for your printer.

## Tests

`tests/run_tests.sh` runs the kinematics, arc, planner speed, G33 solver, one-button, probe/G33 simulation and FAT tests on the host (see `tests/README.md`).

## License

GPL v3, see `LICENSE`.
