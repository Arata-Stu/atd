#!/usr/bin/env bash
# Jetson Docker の atd から、同じ Jetson のログインユーザーにホスト操作を渡す。
set -euo pipefail

case "${1:-}" in
    bluetooth|hotspot|display|performance) ;;
    *) printf 'ATD: Jetson ホストへの操作を指定してください。\n' >&2; exit 2 ;;
esac
source "$(dirname -- "${BASH_SOURCE[0]}")/detect_host.sh"
if ! atd_in_container || [[ "$(detect_atd_host)" != jetson ]]; then
    printf 'ATD: この入口は Jetson Docker で使用します。\n' >&2
    exit 2
fi

login="${ATD_JETSON_SSH_USER:-atd}"
if [[ ! "$login" =~ ^[a-zA-Z0-9_][a-zA-Z0-9_.-]*$ ]]; then
    printf 'ATD: ATD_JETSON_SSH_USER にホストのログインユーザー名を指定してください。\n' >&2
    exit 2
fi

# ~/.ssh はホストからの読み取り専用 mount。ホスト側で登録した鍵を照合する。
# 個別 SSH 設定の HostName / ProxyJump が別のマシンを指す場合も接続先を固定する。
options=(-F /dev/null -a -x -o ClearAllForwardings=yes -o StrictHostKeyChecking=yes
    -o UpdateHostKeys=no -o ConnectTimeout=5)
if [[ -t 0 && -t 1 ]]; then
    options+=(-t)
else
    options+=(-T -o BatchMode=yes)
fi
remote='if [ -e /.dockerenv ] || [ -e /run/.containerenv ] || [ "${ATD_RUNTIME_API:-}" = 1 ]; then
    echo "ATD: SSH の接続先がコンテナです。Jetson ホストへの接続を確認してください。" >&2
    exit 1
fi
launcher="$HOME/.local/bin/atd"
if [ ! -x "$launcher" ]; then launcher="$HOME/workspaces/atd/scripts/atd"; fi
if [ ! -x "$launcher" ]; then
    echo "ATD: Jetson ホストの checkout で ./scripts/setup.sh --action launcher を実行してください。" >&2
    exit 127
fi
exec "$launcher"'
# ssh は引数をリモートシェル用のコマンド文字列として送る。
remote+=" $(/usr/bin/python3 -I -c 'import shlex, sys; print(shlex.join(sys.argv[1:]))' "$@")"

if ssh "${options[@]}" -p 22 -l "$login" 127.0.0.1 "$remote"; then
    exit 0
else
    status=$?
fi
if ((status == 255)); then
    printf 'ATD: %s@127.0.0.1 の SSH 接続を確認してください。初回のホスト鍵登録手順は docs/setup/jetson.md にあります。\n' "$login" >&2
    printf 'Jetson ホストから直接実行するコマンド\n  atd' >&2
    printf ' %q' "$@" >&2
    printf '\n' >&2
fi
exit "$status"
