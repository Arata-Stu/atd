#!/usr/bin/env bash
# Jetson ホストで、選択済みの電力モードにおける最大クロックとファン PWM を設定する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: ./scripts/jetson_max_performance.sh [--check]
  引数なし   MAXN / MAXN_SUPER で CPU・GPU・EMC の最大クロックとファン最大 PWM を設定し、確認します。
  --check   現在の電力モード、クロックの設定値、ファン PWM を読み取って検証します。
電力モードの選択は Jetson ホストの /etc/nvpmodel.conf と sudo nvpmodel -q で確認してください。
EOF
}
fail() { printf 'ATD Jetson: %s\n' "$*" >&2; exit 1; }

mode=apply
(($# <= 1)) || { usage >&2; exit 2; }
if (($#)); then
    case "$1" in
        --check) mode=check ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
fi

source "$(dirname -- "${BASH_SOURCE[0]}")/lib/jetson_host.sh"
atd_require_jetson_host
for tool in nvpmodel jetson_clocks; do
    command -v "$tool" >/dev/null || fail "$tool が見つかりません。Jetson の BSP を確認してください。"
done
power_status="$(atd_run_as_root nvpmodel -q 2>&1)" || fail "nvpmodel の照会に失敗しました。 $power_status"
printf '%s\n' "$power_status"
power_name="$(awk '/^[[:space:]]*NV Power Mode:/ { sub(/^[^:]+:[[:space:]]*/, ""); sub(/[[:space:]]+$/, ""); print; exit }' <<< "$power_status")"
case "$power_name" in
    MAXN|MAXN_SUPER) ;;
    *) fail "現在の電力モードは ${power_name:-不明} です。/etc/nvpmodel.conf で MAXN または MAXN_SUPER の ID を確認し、nvpmodel で選択してください。" ;;
esac

if [[ "$mode" = apply ]]; then
    printf '%s で最大クロックと最大ファン PWM を設定します。\n' "$power_name"
    atd_run_as_root jetson_clocks --fan
fi

clock_status="$(atd_run_as_root jetson_clocks --show 2>&1)" || fail "jetson_clocks の照会に失敗しました。 $clock_status"
printf '%s\n' "$clock_status"
LC_ALL=C awk '
    function value(name, i) {
        for (i = 2; i <= NF; i++)
            if (index($i, name "=") == 1) return substr($i, length(name) + 2)
        return ""
    }
    function problem(message) { print message > "/dev/stderr"; failed = 1 }
    $1 ~ /^cpu[0-9]+:$/ || $1 ~ /^(GPU|gpu-(gpc|nvd|pwr)-0|EMC)$/ {
        if ($1 ~ /^cpu/ && value("Online") == "0") next
        if ($1 ~ /^cpu/) seen["CPU"] = 1
        else if ($1 == "EMC") seen["EMC"] = 1
        else seen["GPU"] = 1
        minimum = value("MinFreq")
        maximum = value("MaxFreq")
        fixed = (minimum ~ /^[1-9][0-9]*$/ && minimum == maximum)
        # 旧 BSP の EMC は設定範囲を保持したまま FreqOverride で固定する。
        if ($1 == "EMC" && value("FreqOverride") != "")
            fixed = (value("FreqOverride") == "1" && value("CurrentFreq") == maximum)
        if (maximum !~ /^[1-9][0-9]*$/ || !fixed)
            problem("最大クロック設定を確認できません: " $0)
    }
    $1 == "FAN" {
        if (tolower(value("Control")) != "disabled")
            problem("ファンの手動固定を確認できません: " $0)
        for (i = 2; i <= NF; i++) {
            if ($i !~ /^(hwmon[0-9]+_)?pwm[0-9]+=/) continue
            seen["FAN"] = 1
            if ($i !~ /=255$/) problem("ファン PWM が最大値 255 ではありません: " $i)
        }
    }
    END {
        split("CPU GPU EMC FAN", required)
        for (i in required)
            if (!seen[required[i]]) problem(required[i] " の設定値を取得できません。")
        exit failed
    }
' <<< "$clock_status"
printf 'OK: %s で CPU・GPU・EMC の最大クロック設定とファン PWM 255 を確認しました。\n' "$power_name"
