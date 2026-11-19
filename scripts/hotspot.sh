#!/usr/bin/env bash
# Jetson の NetworkManager で対象インターフェースの AP を操作する。
set -euo pipefail
export LC_ALL=C
# パスワードを扱う区間でシェルトレースや子プロセスの環境へ秘密を渡さない。
set +x
provided_password="${ATD_HOTSPOT_PASSWORD:-}"
unset ATD_HOTSPOT_PASSWORD

usage() {
    cat <<'HELP'
使い方: ./scripts/hotspot.sh [status|on|off] [--ifname IFACE] [--profile 名前またはUUID]
       ./scripts/hotspot.sh on [--band 2.4|5] [--channel チャンネル番号]

引数なしは状態表示。既定のインターフェースは wlP1p1s0。
on（start）は稼働中の AP またはこのインターフェース用の保存済み AP を再利用する。
AP がなければ ATD-Hotspot を作成（5 GHz/ch36、WPA2、10.42.0.1/24）。
作成時のパスワードは非表示で入力する。非対話では ATD_HOTSPOT_PASSWORD で渡せる。
帯域・チャンネルの指定は保存される。2.4 GHz へ変更するときの既定は ch6。
off（stop）は選択した AP のみ停止する。帯域切替・停止でこの Wi-Fi の SSH は切断される。
HELP
}

fail() { printf 'ATD Hotspot: %s\n' "$*" >&2; exit 1; }

action=''
iface=wlP1p1s0
profile=''
band=''
channel=''
while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        on|start|off|stop|status)
            [[ -z "$action" ]] || fail '操作は一つだけ指定してください。'
            action="$1"; shift ;;
        --ifname|--profile|--band|--channel)
            (($# >= 2)) && [[ -n "$2" && "$2" != --* ]] || fail "$1 の値を指定してください。"
            case "$1" in
                --ifname) iface="$2" ;;
                --profile) profile="$2" ;;
                --band) case "$2" in 2.4) band=bg ;; 5) band=a ;; *) fail '--band は 2.4 または 5 を指定してください。' ;; esac ;;
                --channel) channel="$2"; [[ "$channel" =~ ^[1-9][0-9]{0,2}$ ]] || fail 'チャンネル番号が不正です。' ;;
            esac
            shift 2 ;;
        *) fail "不明な引数: $1（--help を参照）" ;;
    esac
done
case "${action:-status}" in start) action=on ;; stop) action=off ;; status) action=status ;; esac
[[ "$action" = on || ( -z "$band" && -z "$channel" ) ]] || fail '帯域・チャンネルは on のときだけ指定できます。'

source "$(dirname -- "${BASH_SOURCE[0]}")/lib/jetson_host.sh"
atd_require_jetson_host
command -v nmcli >/dev/null || fail 'nmcli がありません。Jetson ホストの NetworkManager を確認してください。'
device_field() { nmcli -g "$1" device show "$iface"; }
profile_field() { nmcli --escape no -g "$1" connection show uuid "$2"; }
[[ "$(device_field GENERAL.TYPE)" = wifi ]] || fail "$iface は利用可能な Wi-Fi インターフェースではありません。"
active="$(device_field GENERAL.CON-UUID)"
# nmcli -g は値に応じて空行を返す。プロファイル名に空白やコロンがあっても UUID 単位で照合する。
ids="$(nmcli -g UUID connection show)"
uuid=''
active_ap=''
match_count=0
default_name_taken=false
for id in $ids; do
    name="$(profile_field connection.id "$id")"
    if [[ "$name" = ATD-Hotspot ]]; then default_name_taken=true; fi
    if [[ -n "$profile" && "$profile" != "$id" && "$profile" != "$name" ]]; then continue; fi
    type="$(profile_field connection.type "$id")"
    if [[ "$type" != 802-11-wireless && "$type" != wifi ]]; then continue; fi
    mode="$(profile_field 802-11-wireless.mode "$id")"
    [[ "$mode" = ap ]] || continue
    bound="$(profile_field connection.interface-name "$id")"
    if [[ "$bound" != "$iface" ]]; then
        [[ -z "$bound" ]] || continue
        if [[ "$id" != "$active" ]]; then
            [[ -n "$profile" ]] || continue
            active_ids="$(nmcli -g UUID connection show --active)"
            if grep -Fxq "$id" <<< "$active_ids"; then fail "$name は別のインターフェースで起動中です。"; fi
        fi
    fi
    uuid="$id"
    if [[ "$id" = "$active" ]]; then active_ap="$id"; fi
    match_count=$((match_count + 1))
done
if ((match_count > 1)); then
    [[ -z "$profile" && -n "$active_ap" ]] || fail "$iface の AP が複数一致しました。nmcli connection show で UUID を確認し、--profile UUID を指定してください。"
    uuid="$active_ap"
fi
if [[ -n "$profile" && -z "$uuid" ]]; then fail "$iface に紐付く AP が見つかりません: $profile"; fi

show_status() {
    local current
    nmcli -f GENERAL.DEVICE,GENERAL.STATE,GENERAL.CONNECTION,IP4.ADDRESS device show "$iface"
    if [[ -z "$uuid" ]]; then printf 'このインターフェース用の保存済み AP はありません。\n'; return; fi
    nmcli -f connection.id,connection.uuid,802-11-wireless.ssid,802-11-wireless.band,802-11-wireless.channel connection show uuid "$uuid"
    current="$(device_field GENERAL.CON-UUID)"
    if [[ "$current" = "$uuid" ]]; then
        printf 'Hotspot 起動中  %s\n' "$uuid"
        if command -v iw >/dev/null; then iw dev "$iface" info || true; fi
    else
        printf 'Hotspot 停止中  %s\n' "$uuid"
    fi
}
if [[ "$action" = status ]]; then show_status; exit 0; fi
if [[ "$action" = off ]]; then
    if [[ -n "$uuid" && "$active" = "$uuid" ]]; then
        printf '%s の Hotspot を停止します。この Wi-Fi 経由の SSH は切断されます。\n' "$iface"
        atd_run_as_root nmcli --wait 20 connection down uuid "$uuid"
        active="$(device_field GENERAL.CON-UUID)"
        [[ "$active" != "$uuid" ]] || fail 'Hotspot の停止を確認できません。'
    fi
    if [[ -n "$uuid" ]]; then
        printf '選択した Hotspot は停止しています（%s）。\n' "$uuid"
    else
        printf '%s に停止対象の Hotspot はありません。\n' "$iface"
    fi
    exit 0
fi

[[ "$(device_field WIFI-PROPERTIES.AP)" = yes ]] || fail "$iface の AP 機能を利用できません。"
old_band=''
old_channel=''
if [[ -n "$uuid" ]]; then
    old_band="$(profile_field 802-11-wireless.band "$uuid")"
    old_channel="$(profile_field 802-11-wireless.channel "$uuid")"
fi
new_band="${band:-$old_band}"
if [[ -z "$uuid" ]]; then new_band="${new_band:-a}"; fi
new_channel="${channel:-$old_channel}"
if [[ -z "$channel" && ( -z "$uuid" || "$new_band" != "$old_band" ) ]]; then
    if [[ "$new_band" = bg ]]; then new_channel=6; else new_channel=36; fi
fi
if [[ -n "$channel" ]]; then
    case "$new_band" in
        bg) (( channel <= 14 )) || fail '2.4 GHz のチャンネルは 1〜14 から指定してください。' ;;
        a) (( channel >= 32 && channel <= 196 )) || fail '5 GHz のチャンネル番号が不正です。' ;;
        *) fail 'チャンネル指定には --band 2.4 または --band 5 が必要です。' ;;
    esac
fi
changed=false
if [[ -n "$uuid" && ( "$old_band" != "$new_band" || "$old_channel" != "$new_channel" ) ]]; then
    changed=true
fi
if [[ -n "$uuid" && "$active" = "$uuid" && "$changed" = false ]]; then
    # 編集済み保存値と起動中の周波数が一致するか、再スキャンせずに確認する。
    live_channel="$(nmcli -t -f ACTIVE,CHAN device wifi list ifname "$iface" --rescan no | awk -F: '$1=="yes" {print $2}')"
    if [[ -z "$new_channel" || "$new_channel" = 0 || "$live_channel" = "$new_channel" ||
          ( -z "$live_channel" && -z "$band" && -z "$channel" ) ]]; then
        show_status; exit 0
    fi
fi

# 同一 IF への並行した初回 on でも同じ UUID を使う。NetworkManager は UUID 重複を拒否する。
managed_id="$(python3 -c 'import sys, uuid; print(uuid.uuid5(uuid.NAMESPACE_URL, "urn:atd:hotspot:" + sys.argv[1]))' "$iface")"
password=''
if [[ -z "$uuid" || "$uuid" = "$managed_id" ]]; then
    need_psk=yes
    if [[ -z "$uuid" ]]; then
        [[ "$default_name_taken" = false ]] || fail 'ATD-Hotspot という別の接続があります。対象 AP を --profile UUID で指定してください。'
    else
        # 初回中断や秘密の保存失敗の場合だけパスワードを再入力する。
        need_psk="$(atd_run_as_root nmcli --show-secrets -g 802-11-wireless-security.psk connection show uuid "$uuid" |
            awk 'length { found=1 } END { print found ? "no" : "yes" }')"
    fi
    if [[ "$need_psk" = yes ]]; then
        password="$provided_password"
        if [[ -z "$password" ]]; then
            [[ -t 0 ]] || fail '初回パスワードを端末で入力するか ATD_HOTSPOT_PASSWORD を設定してください。'
            IFS= read -rs -p 'Hotspot の WPA2 パスワード: ' password
            printf '\n'
        fi
        [[ "$password" =~ ^[[:print:]]{8,63}$ || "$password" =~ ^[[:xdigit:]]{64}$ ]] || fail 'パスワードは ASCII 8〜63 文字、または 64 桁の16進数を指定してください。'
        if ((${#password} != 64)); then
            ssid=ATD-Hotspot
            if [[ -n "$uuid" ]]; then ssid="$(profile_field 802-11-wireless.ssid "$uuid")"; fi
            # passwd-file が空白やバックスラッシュを解釈しても元の入力と同じ鍵を使う。
            password="$(printf '%s' "$password" | python3 -c 'import hashlib, os, sys
print(hashlib.pbkdf2_hmac("sha1", sys.stdin.buffer.read(), os.fsencode(sys.argv[1]), 4096, 32).hex())' "$ssid")"
        fi
    fi
fi
unset provided_password
if [[ -z "$uuid" ]]; then
    uuid="$managed_id"
    atd_run_as_root nmcli connection add type wifi ifname "$iface" con-name ATD-Hotspot connection.uuid "$uuid" \
        autoconnect no ssid ATD-Hotspot 802-11-wireless.mode ap 802-11-wireless.band "$new_band" \
        802-11-wireless.channel "$new_channel" 802-11-wireless-security.key-mgmt wpa-psk \
        802-11-wireless-security.proto rsn ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method disabled
elif [[ "$changed" = true ]]; then
    atd_run_as_root nmcli connection modify uuid "$uuid" 802-11-wireless.band "$new_band" 802-11-wireless.channel "$new_channel"
fi

printf '%s を Hotspot に切り替えます。この Wi-Fi 経由の SSH は切断されます。\n' "$iface"
activate=(nmcli --wait 40 connection up uuid "$uuid" ifname "$iface")
if [[ -n "$password" ]]; then
    # ユーザーの秘密を nmcli / sudo のコマンド引数に含めない。
    activate+=(passwd-file /dev/stdin)
fi
activate_ap() {
    if [[ -n "$password" ]]; then
        printf '802-11-wireless-security.psk:%s\n' "$password" | atd_run_as_root "${activate[@]}"
    else
        atd_run_as_root "${activate[@]}"
    fi
}
if ! activate_ap || [[ "$(device_field GENERAL.CON-UUID)" != "$uuid" ]]; then
    if [[ "$changed" = true ]]; then
        atd_run_as_root nmcli connection modify uuid "$uuid" 802-11-wireless.band "$old_band" 802-11-wireless.channel "$old_channel" ||
            fail '起動と保存済み帯域の復元に失敗しました。status で現在値を確認してください。'
    fi
    fail 'Hotspot の起動を確認できません。Wi-Fi の無線状態・対応チャンネルを確認してください。'
fi
show_status
