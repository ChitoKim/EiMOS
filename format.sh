#!/bin/sh
# tested on clang-format version 19.1.3

# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$SCRIPT_DIR/src"
EX_DIR="$SCRIPT_DIR/examples"

# Define an array of files relative to the script's directory
FILES=("$SRC_DIR/EiMOS.cpp" "$SRC_DIR/EiMOS.h" "$SRC_DIR/MUX.cpp" "$SRC_DIR/MUX.h" "$SRC_DIR/components.h"  "$EX_DIR/ESP32_AMOS_MONSTER/ESP32_AMOS_MONSTER.ino" "$EX_DIR/MEGA2560_CENTURY_GOLD/MEGA2560_CENTURY_GOLD.ino" "$EX_DIR/PICO_AMOS_MONSTER/PICO_AMOS_MONSTER.ino")
# Loop over each file in the array
for FILE in "${FILES[@]}"; do
  clang-format -i -style=file:"$SCRIPT_DIR"/.clang-format "$FILE"
  vim -c '%s/=\n  {\n/= {\r/g' -c 'wq' "$FILE"
  vim -c '%s/=\n  {{/= {\r  {/g' -c 'wq' "$FILE"
  clang-format -i -style=file:"$SCRIPT_DIR"/.clang-format "$FILE"
done