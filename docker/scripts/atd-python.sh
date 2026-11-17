#!/usr/bin/env bash
# コンテナ内の用途別 uv 環境。ROS の apt 基盤と学習依存を分離する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: atd-python [ros|train|vgl-export|web|raceline] [sync|check|shell|run] [-- COMMAND/ARGS]

引数なし: fzf で用途・操作を選択（未導入時は番号選択）
  sync    lock に従って環境を準備。依存の更新はしない
  check   ファイル・venv の設定を確認。作成・同期・通信はしない
  shell   同期後、選択した環境の Bash に入る
  run     同期後、-- に続くコマンドを実行

例: atd-python train run -- python train.py
保存先: $ISAAC_ROS_WS/.atd-python/<architecture>/{ros,train,vgl-export,web,raceline}
通常のコンテナユーザーから実行する。ホストの Python 環境とは共有しない。
EOF
}
fail() { printf 'ATD Python: %s\n' "$*" >&2; exit 1; }

choose() {
    local task_header="$1" task_choices="$2" task_answer task_number task_index=0 task_key task_label
    [[ -t 0 && -t 1 ]] || { usage >&2; fail '対話端末で起動するか、用途と操作を指定してください'; }
    if command -v fzf >/dev/null 2>&1; then
        task_answer="$(printf '%s\n' "$task_choices" | fzf --delimiter=$'\t' --with-nth=2 \
            --height=60% --border --prompt='ATD Python > ' --header="$task_header")" || exit 1
        task_key="${task_answer%%$'\t'*}"
        while IFS=$'\t' read -r task_answer task_label; do
            if [[ "$task_key" = "$task_answer" ]]; then printf '%s\n' "$task_key"; return 0; fi
        done <<< "$task_choices"
        fail '選択した操作がありません'
    fi
    while IFS=$'\t' read -r task_key task_label; do
        task_index=$((task_index + 1))
        printf '%d) %s\n' "$task_index" "$task_label" >&2
    done <<< "$task_choices"
    read -r -p '番号: ' task_number || exit 1
    [[ "$task_number" =~ ^[0-9]+$ && ${#task_number} -le 2 ]] || exit 1
    task_number=$((10#$task_number))
    ((task_number >= 1 && task_number <= task_index)) || exit 1
    printf '%s\n' "$task_choices" | awk -F '\t' -v item="$task_number" 'NR==item {print $1}'
}

if [[ "${1:-}" = --help || "${1:-}" = -h ]]; then usage; exit 0; fi
task_profile="${1:-}"
if (($#)); then shift; else task_profile="$(choose 'Python の用途' $'ros\tROS 2 の追加 Python 依存\ntrain\tAI 学習・解析\nvgl-export\tNotebook の VGL ONNX export\nweb\tNotebook の Web API\nraceline\tNotebook の最小曲率ライン最適化')" || exit 1; fi
case "$task_profile" in ros|train|vgl-export|web|raceline) ;; *) fail '用途は ros、train、vgl-export、web、raceline を指定してください' ;; esac
task_action="${1:-}"
if (($#)); then
    shift
else
    task_choices=$'sync\t環境を準備・同期\ncheck\t設定を確認（変更なし）\nshell\t環境の Bash に入る'
    task_action="$(choose "${task_profile}: 操作を選択" "$task_choices")" || exit 1
fi
case "$task_action" in
    sync|check|shell|run) ;;
    build) fail 'ROS package のビルドは scripts/build.sh または atd-build を使用してください' ;;
    *) fail "不明な操作: $task_action" ;;
esac
if [[ "${1:-}" = -- ]]; then shift; fi
case "$task_action" in
    run) (($#)) || fail 'run の後に実行するコマンドを指定してください' ;;
    *) (($# == 0)) || fail "${task_action} に追加引数は指定できません" ;;
esac

task_script_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ -r "$task_script_dir/lib/python-environment.sh" ]]; then
    source "$task_script_dir/lib/python-environment.sh"
else
    source /opt/atd/scripts/lib/python-environment.sh
fi

atd_python_context "$task_profile"
atd_python_describe
if [[ "$task_action" = check ]]; then
    [[ -d "$task_venv" ]] || fail 'venv は未作成です。sync を実行してください'
    atd_python_check
    printf '[OK] uv / project / lock ファイル / venv 設定（依存の実 import・GPU 演算は別途確認）\n'
    exit 0
fi
atd_python_sync
if [[ "$task_action" = sync ]]; then exit 0; fi
atd_python_activate
case "$task_action" in
    shell) exec /bin/bash --noprofile --norc -i ;;
    run) exec "$@" ;;
esac
