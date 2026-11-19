# Marlin 1.0 - Delta Firmware

Marlin 1.0.2-3 firmware reduced to a clean, English-only build for **delta
(Kossel / Rostock style) 3D printers**.

## What is included

- Delta kinematics (`calculate_delta`, delta calibration, endstop adjustment, M665/M666)
- English-only LCD and serial messages (`Marlin/language.h`)
- ATmega2560 / ATmega1280 boards: RAMPS 1.x, RAMBo, RUMBA, Megatronics, Megatronics v2, Minitronics, Azteeg X3 / X3 Pro, Elefu RA
- Delta default configuration in `Marlin/Configuration.h` and `Marlin/Configuration_adv.h`

## Building

Arduino IDE: open `Marlin/Marlin.ino`, select an *Arduino Mega 2560* (or 1280) board.

PlatformIO:

```bash
cd PlatformIOAddons && platformio run
```

Make (avr-gcc): `cd Marlin && make HARDWARE_MOTHERBOARD=33`

Select your electronics with `MOTHERBOARD` in `Marlin/Configuration.h`
(board ids are listed in `Marlin/boards.h`) and tune the `DELTA_*` values for your printer.

## License

GPL v3, see `LICENSE`.
