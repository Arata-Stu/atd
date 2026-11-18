#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_dir="$(cd -- "$script_dir/.." && pwd)"
sdk_dir="${PICO_SDK_PATH:-$source_dir/.cache/pico-sdk}"
sdk_commit=079c6f39023649b154152db30f1d781e884879bc
tinyusb_commit=86ad6e56c1700e85f1c5678607a762cfe3aa2f47

if [[ ! -d "$sdk_dir/.git" ]]; then
  if [[ -e "$sdk_dir" ]]; then
    echo "Refusing to replace existing non-git SDK path: $sdk_dir" >&2
    exit 1
  fi
  mkdir -p -- "$(dirname -- "$sdk_dir")"
  git clone --depth 1 --branch 2.3.1 https://github.com/raspberrypi/pico-sdk.git "$sdk_dir"
fi
if [[ "$(git -C "$sdk_dir" rev-parse HEAD)" != "$sdk_commit" ]]; then
  echo "Expected Pico SDK 2.3.1 at $sdk_commit. Existing SDK was not changed." >&2
  exit 1
fi
git -C "$sdk_dir" submodule update --init --depth 1 lib/tinyusb
if [[ "$(git -C "$sdk_dir/lib/tinyusb" rev-parse HEAD)" != "$tinyusb_commit" ]]; then
  echo "TinyUSB revision mismatch" >&2
  exit 1
fi
echo "Pico SDK 2.3.1: $sdk_commit"
echo "TinyUSB: $tinyusb_commit"
echo "SDK ready: $sdk_dir"
