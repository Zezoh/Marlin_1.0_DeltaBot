#!/bin/bash
# Host tests: they compile the real firmware sources with g++ against small shims (no AVR needed).
#   kinematics_test     delta kinematics against an independent forward solution, reach, homing geometry
#   arc_test            G2/G3 arcs on a delta and the reach check
#   planner_speed_test  the nozzle speed equals the feed rate
#   sdfat_test          the FAT16/FAT32 code on real disk images (needs mkfs.fat and mtools)
# Usage: tests/run_tests.sh      (from anywhere)
set -e
T=$(cd "$(dirname "$0")" && pwd); ROOT=$(dirname "$T"); B=$(mktemp -d); trap 'rm -rf "$B"' EXIT
CXX="g++ -O1 -w -std=gnu++11 -fpermissive"
python3 "$T/extract.py" "$ROOT" "$B"

echo "== kinematics";  $CXX -I"$B" "$T/kinematics_test.cpp" -o "$B/kin" && "$B/kin"
echo "== arcs";        $CXX -I"$B" "$T/arc_test.cpp" -o "$B/arc" && "$B/arc"

echo "== planner speed"
P="$B/planner"; mkdir "$P"
for f in planner.cpp planner.h Configuration.h Configuration_adv.h language.h boards.h Default_Version.h pins.h thermistortables.h; do cp "$ROOT/Marlin/$f" "$P/"; done
cp "$T"/shim/*.h "$P/"; cp "$T/planner_speed_test.cpp" "$P/"
(cd "$P" && $CXX -I. planner_speed_test.cpp planner.cpp -o speed) && "$P/speed"

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
