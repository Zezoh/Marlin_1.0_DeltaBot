# Marlin 1.0 - Delta Firmware

Marlin 1.0.2-3 firmware reduced to a clean, English-only build for **delta
(Kossel / Rostock style) 3D printers**.

## What is included

- Delta kinematics (`calculate_delta`, delta calibration, endstop adjustment, M665/M666)
- English-only serial messages (`Marlin/language.h`)
- RAMPS 1.3 / 1.4 boards only (ATmega2560 / ATmega1280); no LCD or panel support
- No bed auto leveling or Z probe code (it was not compatible with delta kinematics)
- Thermal runaway protection and the hardware watchdog (4 s) enabled by default
- SD card printing enabled by default (FAT16/FAT32 via `Sd2Card` + `SdFat`, M20-M32, M928, autostart `auto0.g`...`auto9.g`)
- Settings storage (M500/M501) always on, including the delta geometry (M665/M666)
- Delta default configuration in `Marlin/Configuration.h` and `Marlin/Configuration_adv.h`

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

`tests/run_tests.sh` runs the kinematics, arc, planner speed and FAT tests on the host (see `tests/README.md`).

## License

GPL v3, see `LICENSE`.
