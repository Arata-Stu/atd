#!/usr/bin/env bash
set -euo pipefail
source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
out="${JPBB_PICO_TEST_BUILD_DIR:-$source_dir/build/host-tests}"
board_dir="${JPBB_REV6_BOARD_DIR:-}"
while (($#)); do
  case "$1" in
    --board-dir)
      if (($# < 2)) || [[ -z "$2" ]]; then
        printf 'Usage: %s [--board-dir DIR]\n' "$0" >&2
        exit 2
      fi
      board_dir="$2"
      shift 2
      ;;
    -h|--help)
      printf 'Usage: %s [--board-dir DIR]\n' "$0"
      printf 'GPIO/netlist check also accepts JPBB_REV6_BOARD_DIR; omitted means explicit SKIP.\n'
      exit 0
      ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
mkdir -p "$out"
compiler="${CC:-cc}"
common_flags=(-std=c11 -Wall -Wextra -Werror -g)
if [[ "${JPBB_SANITIZE:-undefined}" != 0 ]]; then
  common_flags+=("-fsanitize=${JPBB_SANITIZE:-undefined}" -fno-sanitize-recover=all -fno-omit-frame-pointer)
fi
for module in core sbus transport; do
  sources=("$source_dir/tests/test_$module.c" "$source_dir/src/rev6_$module.c")
  if [[ "$module" == transport ]]; then
    sources+=("$source_dir/src/jpbb_protocol.c")
  fi
  "$compiler" "${common_flags[@]}" -I"$source_dir/include" \
    "${sources[@]}" -o "$out/test_$module"
  "$out/test_$module"
done
# Different receiver channel order/direction is a supported configuration,
# rather than an edit to the SBUS decoding or host protocol.
"$compiler" "${common_flags[@]}" -I"$source_dir/include" \
  -DREV6_SBUS_STEERING_CHANNEL=4 -DREV6_SBUS_THROTTLE_CHANNEL=7 -DREV6_SBUS_MODE_CHANNEL=9 \
  -DREV6_SBUS_STEERING_REVERSED=1 -DREV6_SBUS_THROTTLE_REVERSED=1 -DREV6_SBUS_MODE_REVERSED=1 \
  "$source_dir/tests/test_core.c" "$source_dir/src/rev6_core.c" -o "$out/test_core_remapped"
"$out/test_core_remapped"
# Portable JPB1 parser tests use the local Pico board adapter, without STM32 HAL.
"$compiler" "${common_flags[@]}" -I"$source_dir/include" \
  "$source_dir/tests/test_protocol.c" "$source_dir/src/jpbb_protocol.c" \
  -o "$out/test_protocol"
"$out/test_protocol"
python3 -B "$source_dir/tests/test_uf2.py"
if [[ -n "$board_dir" ]]; then
  python3 -B "$source_dir/tests/test_pin_contract.py" --board-dir "$board_dir"
else
  printf 'SKIP: GPIO/native-netlist contract (set JPBB_REV6_BOARD_DIR or --board-dir DIR).\n'
fi
