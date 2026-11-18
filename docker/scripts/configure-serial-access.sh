#!/usr/bin/env bash
# root entrypoint 専用。device の所有者・mode は変更せず、実 GID を通常ユーザーへ追加する。
set -euo pipefail

if [[ "${ATD_SERIAL_ACCESS:-0}" != 1 ]]; then exit 0; fi
if [[ "$(id -u)" != 0 ]]; then
    printf 'ATD serial: root の起動処理で実行してください。\n' >&2
    exit 1
fi
task_user="${USERNAME:-admin}"
task_device_dir="${ATD_SERIAL_DEVICE_DIR:-/dev}"
[[ "$task_device_dir" == /* ]] || {
    printf 'ATD serial: device directory は絶対パスを指定してください。\n' >&2
    exit 1
}
getent passwd "$task_user" >/dev/null || {
    printf 'ATD serial: 通常ユーザーがありません: %s\n' "$task_user" >&2
    exit 1
}

# ホストの dialout GID を未接続時にも登録する。接続済み device は実 GID を優先する。
# これにより通常の udev dialout ルールで作られる ttyACM の後付けにも対応する。
task_gids="${ATD_HOST_SERIAL_GIDS:-}"
if [[ -n "$task_gids" && ! "$task_gids" =~ ^[1-9][0-9]*(,[1-9][0-9]*)*$ ]]; then
    printf 'ATD serial: ATD_HOST_SERIAL_GIDS は 0 以外の数値 GID をカンマ区切りで指定してください。\n' >&2
    exit 1
fi
shopt -s nullglob
for task_device in "$task_device_dir"/ttyACM*; do
    [[ -c "$task_device" ]] || continue
    # 判定直後の hot-unplug は起動失敗にしない。再接続時は host dialout GID を使う。
    task_gid="$(stat -c %g -- "$task_device" 2>/dev/null)" || continue
    # root group への追加は行わない。所有権や mode を広く変更しない。
    if [[ "$task_gid" == 0 ]]; then
        printf 'ATD serial: %s は GID 0 のため group 追加の対象外です。ホストの udev 設定を確認してください。\n' "$task_device" >&2
        continue
    fi
    task_gids+="${task_gids:+,}${task_gid}"
done
[[ -n "$task_gids" ]] || exit 0
task_memberships=" $(id -G "$task_user") "
IFS=, read -r -a task_groups <<< "$task_gids"
for task_gid in "${task_groups[@]}"; do
    [[ -n "$task_gid" ]] || continue
    [[ "$task_memberships" != *" $task_gid "* ]] || continue
    task_group="$(getent group "$task_gid" | cut -d: -f1 || true)"
    if [[ -z "$task_group" ]]; then
        task_group="atdserial${task_gid}"
        groupadd --gid "$task_gid" "$task_group"
    fi
    usermod --append --groups "$task_group" "$task_user"
    task_memberships+="$task_gid "
    printf 'ATD serial: %s に GID %s (%s) を追加しました。\n' "$task_user" "$task_gid" "$task_group"
done
