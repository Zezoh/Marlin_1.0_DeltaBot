# Host tests

`./tests/run_tests.sh` compiles the real firmware sources with g++ (no AVR, no printer) and checks:

| Test | What it checks |
|---|---|
| `kinematics_test` | `calculate_delta()` against an independent forward solution, reach of the arms, homing geometry |
| `arc_test` | G2/G3 arcs on a delta, `delta_reachable()` against NaN, arcs stopping at the edge of the reach |
| `planner_speed_test` | the nozzle speed equals the feed rate (and shows the error without the Cartesian length) |
| `calibration_test` | the G33 solver recovers the geometry of a simulated machine (3, 4 and 6 factors, 7 and 10 points) |
| `onebutton_test` | the button decoder: click, long press, boot hold, contact bounce, `millis()` wrap |
| `probe_test` | the real `probe.cpp` runs G30/G33 on a simulated delta with a wrong geometry and an FSR that fires at the bed: finds radius, angles and endstops, flat bed afterwards, noisy probe, no bed in reach, result out of range |
| `sdfat_test` | the FAT16/FAT32 code on real disk images created with `mkfs.fat` and `mtools` (and `fsck.fat` on the result) |

The delta functions are extracted from `Marlin/Marlin_main.cpp` and `Marlin/motion_control.cpp` by `extract.py`,
so a change in the firmware is tested without copying code. Needs `g++`, `python3`, and for the FAT test
`dosfstools` and `mtools`.
