#!/usr/bin/env bash
# 旧パスとの互換入口。
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/../webui.sh" "$@"
