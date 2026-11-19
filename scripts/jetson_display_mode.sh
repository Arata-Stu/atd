#!/usr/bin/env bash
# Jetson の systemd 起動ターゲットを確認・切替する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方:
  ./scripts/jetson_display_mode.sh status
  ./scripts/jetson_display_mode.sh cui [--now]
  ./scripts/jetson_display_mode.sh gui [--now]

status は次回起動の既定モードを表示します。
cui / gui は次回起動から適用します。--now を付けると現在のセッションにも直ちに適用します。
cui --now で GUI セッションが終了します。
EOF
}
fail() { printf 'ATD Display: %s\n' "$*" >&2; exit 1; }

if (($# == 0)); then usage >&2; exit 1; fi
mode="$1"
shift
case "$mode" in
    -h|--help) (($# == 0)) || fail 'ヘルプには引数を指定できません。'; usage; exit 0 ;;
    status) (($# == 0)) || fail 'status には引数を指定できません。' ;;
    cui) target=multi-user.target ;;
    gui) target=graphical.target ;;
    *) fail "不明なモード: ${mode}（--help を参照）" ;;
esac
if (($# > 0)); then
    [[ $# == 1 && "$1" == --now ]] || fail "不明なオプション: $*"
fi

source "$(dirname -- "${BASH_SOURCE[0]}")/lib/jetson_host.sh"
atd_require_jetson_host
command -v systemctl >/dev/null || fail 'systemctl が見つかりません。'

if [[ "$mode" == status ]]; then
    target="$(systemctl get-default)"
    case "$target" in
        multi-user.target) label=CUI ;;
        graphical.target) label=GUI ;;
        *) label=その他 ;;
    esac
    printf '次回起動の既定モード: %s (%s)\n' "$label" "$target"
    exit 0
fi

atd_run_as_root systemctl set-default "$target"
printf '次回起動の既定ターゲットを %s に設定しました。\n' "$target"
if [[ "${1:-}" == --now ]]; then
    if [[ "$mode" == cui ]]; then printf 'CUI に移行し、実行中の GUI セッションを終了します。\n'; fi
    atd_run_as_root systemctl isolate "$target"
fi
