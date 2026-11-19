#!/usr/bin/env bash
# Jetson ホストの BlueZ で DualSense を接続する。
set -euo pipefail

fail() { printf 'ATD Bluetooth: %s\n' "$*" >&2; exit 1; }
task_mac='4C:B9:9B:E0:EF:24'
task_action=''
while (($#)); do
    case "$1" in
        connect|status)
            [[ -z "$task_action" ]] || fail '操作は一つだけ指定してください'
            task_action="$1"; shift ;;
        --mac)
            (($# >= 2)) || fail '--mac の後に Bluetooth MAC アドレスを指定してください'
            task_mac="$2"; shift 2 ;;
        --help|-h)
            cat <<'EOF'
使い方: ./scripts/bluetooth.sh [connect|status] [--mac MAC]
  引数なし   DualSense（4C:B9:9B:E0:EF:24）を接続。初回はペアリングも行います。
  status    対象の Bluetooth 登録・接続状態を表示します。
Jetson ホストのプロジェクト直下で実行してください。
EOF
            exit 0 ;;
        *) fail "不明な引数: $1（--help を参照）" ;;
    esac
done
[[ "$task_mac" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]] || fail "MAC アドレスの形式が不正です: $task_mac"
task_mac="$(printf '%s' "$task_mac" | tr '[:lower:]' '[:upper:]')"
source "$(dirname -- "${BASH_SOURCE[0]}")/lib/detect_host.sh"
if [[ "$(detect_atd_host)" != jetson ]] || systemd-detect-virt --quiet --container; then
    fail 'Jetson のホスト端末で実行してください。'
fi
command -v bluetoothctl >/dev/null || fail 'bluetoothctl がありません。./scripts/setup.sh --action devices で BlueZ を準備してください。'

# ハードウェア応答と bluetoothd の起動待ちも含め、各操作の待機を制限する。
bt() { LC_ALL=C timeout 30s bluetoothctl "$@"; }
property_is() { grep -Eq "^[[:space:]]*$2: $3[[:space:]]*$" <<< "$1"; }

printf 'DualSense  %s\n' "$task_mac"
task_adapter="$(bt show)" || fail "Bluetooth アダプターを確認できません。bluetooth.service を確認してください。 $task_adapter"
if [[ "$task_action" = status ]]; then
    bt info "$task_mac" || fail '対象の機器情報を取得できません。初回は ./scripts/bluetooth.sh でペアリングしてください。'
    exit 0
fi
if ! property_is "$task_adapter" Powered yes; then
    bt power on || fail 'Bluetooth アダプターを有効にできません。'
fi

task_info="$(bt info "$task_mac")" || task_info=''
if ! property_is "$task_info" Paired yes; then
    printf '初回は USB を抜き、DualSense の電源を切ってから、クリエイトと PS をライトバーが点滅するまで長押ししてください。20 秒間検索します。\n'
    bt --timeout 20 scan on || fail 'Bluetooth の検索に失敗しました。'
    task_info="$(bt info "$task_mac")" || fail "対象の MAC が見つかりません。ペアリングモードで再実行してください。 $task_info"
    if property_is "$task_info" Paired no; then
        bt --agent NoInputNoOutput pair "$task_mac" || fail 'ペアリングに失敗しました。'
        task_info="$(bt info "$task_mac")" || fail 'ペアリング後の状態を取得できません。'
    fi
    property_is "$task_info" Paired yes || fail '対象のペアリングを確認できません。'
fi
if ! property_is "$task_info" Trusted yes; then
    bt trust "$task_mac" || fail '対象を信頼済みとして登録できません。'
fi
if ! property_is "$task_info" Connected yes; then
    printf '接続します。DualSense の電源が切れている場合は PS ボタンを押してください。\n'
    # PS ボタンによる自動再接続と重なった場合も、最後に機器の状態を確認する。
    bt connect "$task_mac" || printf '接続要求が完了しなかったため、接続状態を再確認します。\n' >&2
fi
for task_attempt in 1 2 3; do
    task_info="$(bt info "$task_mac")" || fail '接続後の状態を取得できません。'
    if property_is "$task_info" Connected yes; then
        printf 'Bluetooth 接続済み  %s（Connected: yes）\n' "$task_mac"
        exit 0
    fi
    if ((task_attempt < 3)); then sleep 1; fi
done
fail '接続を確認できません。PS ボタンで起動してから再実行してください。'
