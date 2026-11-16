#!/usr/bin/env bash
# apt 版 Isaac ROS CLI を、ATD のビルド設定と mount 設定で起動する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: scripts/dev.sh [--host notebook|pc|jetson] [--workspace DIR] [--build] [--dry-run]

  --host       自動判定を上書きして、保存先のホスト設定を選択する
  --workspace  保存済みの ROS workspace を単発で上書きする
  --build      追加 Dockerfile をローカル build してから起動する
  --dry-run    設定とコマンドを表示する。CLI / Docker は起動しない
  -h, --help   この説明を表示する

設定: config/storage.env（config/storage.env.example をコピーして編集）
通常は Jetson / Tegra の機種・BSP 情報から自動判定し、それ以外は PC として扱う。
ATD_STORAGE_CONFIG で別の設定ファイルを指定できる。
ROS workspace は config/workspace.env のホスト別設定、未設定なら ~/workspaces/atd。
setup.sh --action workspace で対話設定する。ATD_WORKSPACE_CONFIG で設定ファイルを変更できる。
ATD_ROS_WS は ROS workspace を単発で上書きする。
ATD_RECORD_DIR / ATD_RESULT_DIR は、選択ホストの保存先を単発で上書きする。
保存先ディレクトリは事前に作成する。record / results は毎回、書込み可能で mount する。
rosbag 記録・再生や車両ノードは自動起動しない。
Docker 起動時に追加依存と Fast DDS の loopback 設定を自動適用する。
実行中への再接続は設定の一致を確認し、ATD 専用 shell を開く。
EOF
}

fail() {
    printf 'エラー: %s\n' "$*" >&2
    exit 1
}

task_host=""
task_build=false
task_dry_run=false
task_workspace_override=""
while (($#)); do
    case "$1" in
        --host|--workspace)
            (($# >= 2)) || fail "$1 の値を指定してください"
            [[ -n "$2" ]] || fail "$1 の値が空です"
            if [[ "$1" = --host ]]; then task_host="$2"; else task_workspace_override="$2"; fi
            shift 2
            ;;
        --build) task_build=true; shift ;;
        --dry-run) task_dry_run=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) fail "不明な引数: $1（--help を参照）" ;;
    esac
done
if [[ "$task_host" = notebook ]]; then task_host=pc; fi
case "$task_host" in ""|pc|jetson) ;; *) fail "--host の値は notebook、pc または jetson を指定してください" ;; esac

task_project_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
# 上流 build loader は検索ディレクトリを空白で分割し、設定パスを未 quote で読む。
case "$task_project_dir" in
    *[[:space:]]*|*'$'*|*'`'*|*"'"*|*'"'*|*'\'*|*';'*|*'&'*|*'|'*|*'<'*|*'>'*|*'('*|*')'*|*'{'*|*'}'*|*'['*|*']'*|*'*'*|*'?'*|*'!'*)
        fail "上流 CLI の制約によりプロジェクトパスに空白や shell 特殊文字は使えません（データの保存先は空白可）"
        ;;
esac
task_host_selection="manual"
if [[ -z "$task_host" ]]; then
    source "${task_project_dir}/scripts/detect_host.sh"
    task_host="$(detect_atd_host)"
    task_host_selection="auto"
fi
source "${task_project_dir}/scripts/lib/workspace.sh"
atd_load_workspace "$task_project_dir" "$task_host" || exit 1
task_workspace="${task_workspace_override:-$ATD_RESOLVED_ROS_WS}"
case "$task_workspace" in '~/'*) task_workspace="${HOME}/${task_workspace:2}" ;; esac
atd_validate_workspace_path "$task_workspace" || exit 1
if [[ -d "$task_workspace" ]]; then
    task_workspace="$(CDPATH= cd -- "$task_workspace" && pwd -P)"
    atd_validate_workspace_path "$task_workspace" || exit 1
elif ! $task_dry_run; then
    fail "ROS workspace がありません。setup.sh --action workspace を実行してください: $task_workspace"
fi
task_storage_config="${ATD_STORAGE_CONFIG:-${task_project_dir}/config/storage.env}"
if [[ -f "$task_storage_config" ]]; then
    [[ -r "$task_storage_config" ]] || fail "設定ファイルを読めません: $task_storage_config"
    # ユーザー管理のローカル Bash 設定。秘密情報を出力しない。
    source "$task_storage_config"
elif [[ -n "${ATD_STORAGE_CONFIG:-}" ]]; then
    fail "指定した設定ファイルがありません: $task_storage_config"
fi

case "$task_host" in
    pc)
        task_record_dir="${ATD_RECORD_DIR:-${ATD_PC_RECORD_DIR:-}}"
        task_result_dir="${ATD_RESULT_DIR:-${ATD_PC_RESULT_DIR:-}}"
        ;;
    jetson)
        task_record_dir="${ATD_RECORD_DIR:-${ATD_JETSON_RECORD_DIR:-}}"
        task_result_dir="${ATD_RESULT_DIR:-${ATD_JETSON_RESULT_DIR:-}}"
        ;;
esac

validate_directory() {
    local task_path="$1" task_label="$2"
    [[ -n "$task_path" ]] || fail "$task_label が未設定です。config/storage.env.example を config/storage.env にコピーして編集してください"
    [[ "$task_path" = /* ]] || fail "$task_label は絶対パスを指定してください: $task_path"
    # 上流 loader の変数展開と shell=True、および --mount の CSV 構文に合わせる。
    case "$task_path" in
        *[\$\`\"\\,]*|*$'\n'*|*$'\r'*) fail "$task_label に使用できない文字が含まれています（\$, バッククォート, 二重引用符, バックスラッシュ, カンマ, 改行）" ;;
    esac
    if [[ ! -d "$task_path" ]]; then
        printf 'エラー: %s のディレクトリがありません: %s\n作成する場合: mkdir -p -- ' "$task_label" "$task_path" >&2
        printf '%q\n' "$task_path" >&2
        exit 1
    fi
    [[ -r "$task_path" && -x "$task_path" ]] || fail "$task_label を読み取れません: $task_path"
}
validate_directory "$task_record_dir" "record 保存先"
validate_directory "$task_result_dir" "results 保存先"
task_record_dir="$(CDPATH= cd -- "$task_record_dir" && pwd -P)"
task_result_dir="$(CDPATH= cd -- "$task_result_dir" && pwd -P)"
# symlink 解決後にも上流 loader で扱えるパスであることを確認する。
validate_directory "$task_record_dir" "record 保存先"
validate_directory "$task_result_dir" "results 保存先"
[[ -w "$task_result_dir" ]] || fail "results 保存先に書き込めません: $task_result_dir"
[[ -w "$task_record_dir" ]] || fail "record 保存先に書き込めません: $task_record_dir"

export ATD_PROJECT_DIR="$task_project_dir"
export ISAAC_ROS_WS="$task_workspace"
export ATD_RECORD_DIR="$task_record_dir"
export ATD_RESULT_DIR="$task_result_dir"
command -v python3 >/dev/null 2>&1 || fail "python3 が必要です"
task_source_layout="$(python3 - "$task_project_dir" "$ISAAC_ROS_WS" <<'PY'
from pathlib import Path
import sys
print('native' if (Path(sys.argv[1]) / 'ros2_ws/src').resolve()
      == (Path(sys.argv[2]) / 'src').resolve() else 'external')
PY
)" || exit 1
# 起動済みコンテナへ古い依存・DDS 設定のまま再接続しない。
source "${task_project_dir}/scripts/lib/runtime.sh"
task_runtime_revision="$(atd_runtime_revision "$task_project_dir")" || exit 1
export ATD_RUNTIME_REV="$task_runtime_revision"
# 上流 CLI が COPY 入力を image hash に含めない場合にも、更新した sh 等を反映する。
# 生成物は hash ごとの独立ディレクトリに置き、build context は元の docker/ を使う。
export ATD_GENERATED_DOCKER_DIR="${task_project_dir}/.cache/docker/${ATD_RUNTIME_REV}"
export DOCKER_ARGS_FILE="${ATD_GENERATED_DOCKER_DIR}/dev.${task_host}.dockerargs"
task_container_name="atd_${task_host}_dev"
task_cli_args=(activate --config "docker.run.container_name=${task_container_name}")
if $task_build; then task_cli_args+=(--build-local); fi
printf 'host: %s (%s)\ncontainer: %s\nworkspace: %s\n' "$task_host" "$task_host_selection" "$task_container_name" "$ISAAC_ROS_WS"
if [[ "$task_source_layout" = native ]]; then
    printf 'ROS source: %s/ros2_ws/src -> /workspaces/isaac_ros-dev/src (workspace mount / read-write)\n' "$task_project_dir"
else
    printf 'ROS source: %s/ros2_ws/src -> /workspaces/isaac_ros-dev/src/atd (read-write)\n' "$task_project_dir"
fi
printf 'record: %s -> /data/record (read-write)\nresults: %s -> /data/results (read-write)\n' "$ATD_RECORD_DIR" "$ATD_RESULT_DIR"
printf 'DDS: Fast DDS / UDPv4 loopback\nruntime: %s\n' "$ATD_RUNTIME_REV"
printf 'docker_args: %s\ncommand: isaac-ros' "$DOCKER_ARGS_FILE"
printf ' %q' "${task_cli_args[@]}"
printf '\n'
if $task_dry_run; then exit 0; fi

# 任意の workspace でも公式 CLI が ATD 設定を発見できるようにする。
# 既存の別設定は上書きせず停止する。ユーザーの bashrc は変更しない。
atd_prepare_workspace "$task_project_dir" "$ISAAC_ROS_WS" || exit 1

command -v isaac-ros >/dev/null 2>&1 || fail "apt 版 isaac-ros-cli をインストールしてください"
task_status="$(isaac-ros status --output json)" || fail "CLI の状態を取得できません。ホストで sudo isaac-ros init docker を実行してください"
task_cli_mode="$(python3 -c 'import json, sys; print(json.loads(sys.argv[1])["mode"])' "$task_status")" || fail "CLI の status JSON を解釈できません"
[[ "$task_cli_mode" = docker ]] || fail "CLI が Docker モードではありません。ホストで sudo isaac-ros init docker を実行してください"

# 同名の起動済みコンテナへ違う保存先で再接続することを防ぐ。停止・削除は自動実行しない。
task_attach=false
if command -v docker >/dev/null 2>&1; then
    if task_inspect="$(docker container inspect "$task_container_name" 2>/dev/null)"; then
        task_container_state="$(printf '%s' "$task_inspect" | python3 -c '
import json, os, sys
container = json.load(sys.stdin)[0]
if not container.get("State", {}).get("Running"):
    print("stopped")
    sys.exit(0)
if sys.argv[2] == "true":
    sys.exit("エラー: build 前に同名の実行中コンテナを終了してください: " + sys.argv[1])
expected = {
    "/workspaces/isaac_ros-dev": (os.environ["ISAAC_ROS_WS"], True),
    "/workspaces/atd": (os.environ["ATD_PROJECT_DIR"], False),
    "/data/record": (os.environ["ATD_RECORD_DIR"], True),
    "/data/results": (os.environ["ATD_RESULT_DIR"], True),
}
mounts = {m["Destination"]: m for m in container.get("Mounts", [])}
source_root = os.path.join(os.environ["ATD_PROJECT_DIR"], "ros2_ws", "src")
source_destination = "/workspaces/isaac_ros-dev/src/atd"
if os.path.realpath(source_root) == os.path.realpath(os.path.join(os.environ["ISAAC_ROS_WS"], "src")):
    if source_destination in mounts:
        sys.exit("エラー: 起動済みコンテナに不要な入れ子の source mount があります。コンテナを再作成してください: " + sys.argv[1])
else:
    expected[source_destination] = (source_root, True)
for destination, (source, writable) in expected.items():
    mount = mounts.get(destination)
    if (not mount or mount.get("Type") != "bind"
            or os.path.realpath(mount.get("Source", "")) != os.path.realpath(source)
            or mount.get("RW") != writable):
        sys.exit("エラー: 起動済みコンテナの mount が設定と異なります。記録等を終了してコンテナを再作成してください: " + sys.argv[1])
environment = dict(item.split("=", 1) for item in container.get("Config", {}).get("Env", []) if "=" in item)
required = {
    "ATD_RUNTIME_API": "1",
    "ATD_RUNTIME_REV": os.environ["ATD_RUNTIME_REV"],
    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    "FASTDDS_DEFAULT_PROFILES_FILE": "/etc/atd/fastdds.xml",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "SYSTEM_DEFAULT",
    "ROS_STATIC_PEERS": "",
    "RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
}
if any(environment.get(key) != value for key, value in required.items()):
    sys.exit("エラー: 起動済みコンテナの依存・DDS 設定が古いか異なります。処理を終了し、必要なら --build で再構築してコンテナを再作成してください: " + sys.argv[1])
print("running")
' "$task_container_name" "$task_build")" || exit 1
        if [[ "$task_container_state" = running ]]; then task_attach=true; fi
    fi
fi

if $task_attach; then
    task_ready="$(docker exec "$task_container_name" cat /etc/atd/runtime.ready)" \
        || fail "コンテナの初期設定が完了していません。起動ログを確認してください"
    [[ "$task_ready" = "$ATD_RUNTIME_REV" ]] || fail "コンテナの初期設定が未完了、または現在の設定と異なります"
    task_exec_args=(exec -i)
    if [[ -t 0 && -t 1 ]]; then task_exec_args+=(-t); fi
    printf '初期設定済みのコンテナに再接続します: %s\n' "$task_container_name"
    exec docker "${task_exec_args[@]}" -u admin --workdir /workspaces/isaac_ros-dev \
        "$task_container_name" /usr/local/bin/atd-shell
fi

python3 - "$task_project_dir" "$ATD_GENERATED_DOCKER_DIR" "$ATD_RUNTIME_REV" "$DOCKER_ARGS_FILE" "$task_source_layout" <<'PY'
import os
from pathlib import Path
import sys
import tempfile
root, destination, revision = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
source = (root / 'docker/Dockerfile.atd').read_bytes()
dockerfile = source + ('\n# ATD runtime source revision: ' + revision + '\n').encode()
arguments = (root / 'docker/dev.dockerargs').read_text().splitlines(keepends=True)
if sys.argv[5] == 'native':
    source_mount = '--mount type=bind,src="${ATD_PROJECT_DIR}/ros2_ws/src",dst=/workspaces/isaac_ros-dev/src/atd'
    if sum(line.rstrip('\r\n') == source_mount for line in arguments) != 1:
        sys.exit('エラー: source mount のテンプレートを識別できません')
    arguments = [line for line in arguments if line.rstrip('\r\n') != source_mount]
destination.mkdir(parents=True, exist_ok=True)
for output, content in ((destination / 'Dockerfile.atd', dockerfile),
                        (Path(sys.argv[4]), ''.join(arguments).encode())):
    if not output.exists() or output.read_bytes() != content:
        with tempfile.NamedTemporaryFile(dir=destination, delete=False) as stream:
            temporary = Path(stream.name)
            try:
                stream.write(content)
                stream.flush()
                os.fchmod(stream.fileno(), 0o644)
                os.replace(temporary, output)
            finally:
                temporary.unlink(missing_ok=True)
PY

exec isaac-ros "${task_cli_args[@]}"
