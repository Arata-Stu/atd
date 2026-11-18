#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_dir="$(cd -- "$script_dir/.." && pwd)"
sdk_dir="${PICO_SDK_PATH:-$source_dir/.cache/pico-sdk}"
build_dir="${JPBB_PICO_BUILD_DIR:-$source_dir/build/pico}"
cmake_bin="${CMAKE:-cmake}"
ninja_bin="${NINJA:-ninja}"
sdk_commit=079c6f39023649b154152db30f1d781e884879bc
tinyusb_commit=86ad6e56c1700e85f1c5678607a762cfe3aa2f47

if [[ ! -f "$sdk_dir/pico_sdk_init.cmake" ]]; then
  echo "Pico SDK missing. Run bash $script_dir/bootstrap.sh first." >&2
  exit 1
fi
if [[ "$(git -C "$sdk_dir" rev-parse HEAD)" != "$sdk_commit" ]] ||
   [[ "$(git -C "$sdk_dir/lib/tinyusb" rev-parse HEAD)" != "$tinyusb_commit" ]]; then
  echo "SDK/TinyUSB must match the pinned revisions; run bootstrap.sh." >&2
  exit 1
fi
toolchain_args=(-DPICO_COMPILER=pico_arm_cortex_m0plus_gcc)
if [[ -n "${ARM_GCC_PATH:-}" ]]; then
  toolchain_args+=("-DPICO_TOOLCHAIN_PATH=$ARM_GCC_PATH")
fi
"$cmake_bin" -S "$source_dir" -B "$build_dir" -G Ninja \
  "-DCMAKE_MAKE_PROGRAM=$ninja_bin" "-DPICO_SDK_PATH=$sdk_dir" \
  -DPICO_BOARD=pico -DPICO_NO_PICOTOOL=1 -DCMAKE_BUILD_TYPE=Release \
  "${toolchain_args[@]}"
"$cmake_bin" --build "$build_dir" --parallel "${JPBB_BUILD_JOBS:-4}"
python3 "$script_dir/uf2.py" encode "$build_dir/jpbb_pico_rev6.bin" "$build_dir/jpbb_pico_rev6.uf2"
python3 "$script_dir/uf2.py" verify "$build_dir/jpbb_pico_rev6.bin" "$build_dir/jpbb_pico_rev6.uf2"
echo "Built (not flashed): $build_dir/jpbb_pico_rev6.uf2"
