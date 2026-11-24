#!/bin/bash
# Host tests: they compile the real firmware sources with g++ against small shims (no AVR needed).
#   kinematics_test     delta kinematics against an independent forward solution, reach, homing geometry
#   arc_test            G2/G3 arcs on a delta and the reach check
#   planner_speed_test  the nozzle speed equals the feed rate
#   calibration_test    the G33 solver recovers a simulated machine's geometry
#   mesh_test           the bed mesh: interpolation, fade, inverse
#   filament_test       M600 and the runout sensor on a simulated machine, user and heater
#   fan_test            the fan: M106/M107, hot-end floor, kick start, cooldown, one fan and two fans
#   onebutton_test      the button decoder: clicks, long press, boot hold, bounce
#   probe_test          G30 / G33 end to end on a simulated delta with a wrong geometry and an FSR probe
#   sdfat_test          the FAT16/FAT32 code on real disk images (needs mkfs.fat and mtools)
# Usage: tests/run_tests.sh      (from anywhere)
set -e
T=$(cd "$(dirname "$0")" && pwd); ROOT=$(dirname "$T"); B=$(mktemp -d); trap 'rm -rf "$B"' EXIT
CXX="g++ -O1 -w -std=gnu++11 -fpermissive"
python3 "$T/extract.py" "$ROOT" "$B"

echo "== kinematics"; $CXX -I"$B" -I"$ROOT/Marlin" "$T/kinematics_test.cpp" "$ROOT/Marlin/mesh.cpp" -o "$B/kin"; "$B/kin"
echo "== arcs";       $CXX -I"$B" -I"$ROOT/Marlin" "$T/arc_test.cpp" "$ROOT/Marlin/mesh.cpp" -o "$B/arc"; "$B/arc"

echo "== G33 solver";  $CXX -I"$ROOT/Marlin" "$T/calibration_test.cpp" "$ROOT/Marlin/delta_calibration.cpp" -o "$B/cal"; "$B/cal"
echo "== bed mesh";    $CXX -I"$ROOT/Marlin" "$T/mesh_test.cpp" "$ROOT/Marlin/mesh.cpp" -o "$B/mesh"; "$B/mesh"
echo "== one button";  $CXX -I"$ROOT/Marlin" "$T/onebutton_test.cpp" -o "$B/ob"; "$B/ob"

echo "== probe and G33 on a simulated delta"
Q="$B/probe"; mkdir "$Q"
for f in probe.cpp probe.h ui.h mesh.cpp mesh.h delta_calibration.cpp delta_calibration.h Configuration.h Configuration_adv.h Default_Version.h thermistortables.h pins.h; do cp "$ROOT/Marlin/$f" "$Q/"; done
cp "$T"/probe_shim/*.h "$Q/"; cp "$T/probe_test.cpp" "$B/"
ZH=$(awk '/#define Z_HOME_POS/{print $3}' "$ROOT/Marlin/Configuration.h")
(cd "$Q" && $CXX -DZ_HOME_POS=$ZH -I. -I"$B" probe.cpp mesh.cpp delta_calibration.cpp "$B/probe_test.cpp" -o probe_test); "$Q/probe_test"

echo "== filament change and runout"
F="$B/filament"; mkdir "$F"
for f in filament.cpp filament.h ui.h Configuration.h Configuration_adv.h Default_Version.h thermistortables.h; do cp "$ROOT/Marlin/$f" "$F/"; done
cp "$T"/filament_shim/*.h "$F/"; cp "$T/filament_test.cpp" "$F/"
(cd "$F" && $CXX -I. filament.cpp filament_test.cpp -o filament_test); "$F/filament_test"

echo "== fan, one and two"
N="$B/fan"; mkdir "$N"
for f in fan.cpp fan.h ui.h Configuration.h Configuration_adv.h Default_Version.h thermistortables.h; do cp "$ROOT/Marlin/$f" "$N/"; done
cp "$T"/filament_shim/*.h "$N/"; cp "$T/fan_test.cpp" "$N/"
for d in 0 1; do (cd "$N" && $CXX -DFAN_DUAL=$d -I. fan.cpp fan_test.cpp -o fan_test$d); "$N/fan_test$d"; done

echo "== planner speed"
P="$B/planner"; mkdir "$P"
for f in planner.cpp planner.h Configuration.h Configuration_adv.h language.h Default_Version.h pins.h thermistortables.h; do cp "$ROOT/Marlin/$f" "$P/"; done
cp "$T"/shim/*.h "$P/"; cp "$T/planner_speed_test.cpp" "$P/"
(cd "$P" && $CXX -I. planner_speed_test.cpp planner.cpp -o speed); "$P/speed"
cp "$T/planner_start_test.cpp" "$P/"
(cd "$P" && $CXX -I. planner_start_test.cpp planner.cpp -o start); "$P/start" 600 12 150 40 6; "$P/start" 2500 20 150 40 6

echo "== FAT file system"
if command -v mkfs.fat >/dev/null && command -v mcopy >/dev/null; then
  S="$B/sd"; mkdir "$S"; cp "$ROOT/Marlin/SdFat.cpp" "$ROOT/Marlin/SdFat.h" "$T"/sd_shim/*.h "$S/"
  (cd "$S" && $CXX -fsanitize=address,undefined -I. "$T/sdfat_test.cpp" SdFat.cpp -o sdfat)
  head -c 300000 /dev/urandom > "$B/big.ref"; echo "G28" > "$B/deep.txt"
  for fat in 16 32; do
    img="$B/img$fat.bin"; truncate -s $((fat==16?64:80))M "$img"; mkfs.fat -F $fat "$img" >/dev/null
    mmd -i "$img" ::dir ::dir/sub; mcopy -i "$img" "$B/big.ref" ::big.g; mcopy -i "$img" "$B/deep.txt" ::dir/sub/deep.g
    echo "-- FAT$fat"; "$S/sdfat" "$img" "$B/big.ref" "$B/out$fat.bin" | tail -2
    if command -v fsck.fat >/dev/null; then fsck.fat -n "$B/out$fat.bin" >/dev/null && echo "fsck: clean"; fi
  done
else
  echo "skipped (install dosfstools and mtools)"
fi
echo "ALL TESTS PASSED"
