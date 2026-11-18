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

保存先: setup.sh --action storage で対話設定し、config/storage.env に保存。
手動設定は config/storage.env.example をコピーし、対象行のコメントを外して実パスを記入する。
通常は Jetson / Tegra の機種・BSP 情報から自動判定し、それ以外は PC として扱う。
ATD_STORAGE_CONFIG で別の設定ファイルを指定できる。
ROS workspace は config/workspace.env のホスト別設定、未設定なら clone 先の ros2_ws/。
setup.sh --action workspace で対話設定する。ATD_WORKSPACE_CONFIG で設定ファイルを変更できる。
ATD_ROS_WS は ROS workspace を単発で上書きする。
ATD_RECORD_DIR / ATD_RESULT_DIR は、選択ホストの保存先を単発で上書きする。
Notebook の --build は一時ファイルを $HOME/.cache/atd/build-tmp に作る。
TMPDIR を指定すると、既存の書込み可能なディレクトリを使う。
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

# 上流 CLI は設定ファイルや一時 bake ファイルのパスを未 quote で扱う。
validate_cli_shell_path() {
    case "$1" in
        *[[:space:]]*|*'$'*|*'`'*|*"'"*|*'"'*|*'\'*|*';'*|*'&'*|*'|'*|*'<'*|*'>'*|*'('*|*')'*|*'{'*|*'}'*|*'['*|*']'*|*'*'*|*'?'*|*'!'*)
            fail "上流 CLI の制約により${2}に空白や shell 特殊文字は使えません"
            ;;
    esac
}
task_project_dir="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
validate_cli_shell_path "$task_project_dir" "プロジェクトパス"
task_host_selection="manual"
if [[ -z "$task_host" ]]; then
    source "${task_project_dir}/scripts/lib/detect_host.sh"
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

# ホストの鍵・known_hostsを正本とし、内容のコピーや権限変更をしない。
export ATD_HOST_SSH_DIR=''
export ATD_HOST_SSH_AGENT=''
if [[ -d "$HOME/.ssh" ]]; then
    validate_directory "$HOME/.ssh" "SSH directory"
    ATD_HOST_SSH_DIR="$(CDPATH= cd -- "$HOME/.ssh" && pwd -P)"
    validate_directory "$ATD_HOST_SSH_DIR" "SSH directory"
fi
# Jetson の agent は上流 CLI が /ssh-agent へ共有する。PCのみ不足分を補う。
if [[ "$task_host" = pc && -n "${SSH_AUTH_SOCK:-}" && -S "$SSH_AUTH_SOCK" ]]; then
    case "$SSH_AUTH_SOCK" in
        /*) ;;
        *) fail 'SSH_AUTH_SOCK は絶対パスが必要です' ;;
    esac
    case "$SSH_AUTH_SOCK" in
        *[\$\`\"\\,]*|*$'\n'*|*$'\r'*) fail 'SSH_AUTH_SOCK に使用できない文字が含まれています' ;;
    esac
    ATD_HOST_SSH_AGENT="$SSH_AUTH_SOCK"
fi

export ATD_PROJECT_DIR="$task_project_dir"
export ISAAC_ROS_WS="$task_workspace"
export ATD_RECORD_DIR="$task_record_dir"
export ATD_RESULT_DIR="$task_result_dir"
command -v python3 >/dev/null 2>&1 || fail "python3 が必要です"
# 未接続起動でも後付け ttyACM の group を先に追加できるよう、ホストの実 GID を渡す。
# device 個別の --device 指定は再接続で inode が変わるため使わない。
export ATD_SERIAL_ACCESS=0
export ATD_HOST_SERIAL_GIDS=''
if [[ "$task_host" == jetson ]]; then
    ATD_SERIAL_ACCESS=1
    ATD_HOST_SERIAL_GIDS="$(python3 - <<'PY'
import grp
from pathlib import Path
import stat

gids = set()
try:
    gids.add(grp.getgrnam('dialout').gr_gid)
except KeyError:
    pass
for path in Path('/dev').glob('ttyACM*'):
    try:
        info = path.stat()
    except FileNotFoundError:  # hot-unplug during startup
        continue
    if stat.S_ISCHR(info.st_mode):
        gids.add(info.st_gid)
print(','.join(str(gid) for gid in sorted(gids) if gid > 0))
PY
)" || fail 'ホストの USB serial GID を取得できません'
fi
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
# 起動引数は起動構成ごとに生成する。イメージタグには build 用の入力だけを反映する。
export ATD_GENERATED_DOCKER_DIR="${task_project_dir}/.cache/docker/${ATD_RUNTIME_REV}"
export DOCKER_ARGS_FILE="${ATD_GENERATED_DOCKER_DIR}/dev.${task_host}.dockerargs"
task_container_name="atd_${task_host}_dev"
task_cli_args=(activate --config "docker.run.container_name=${task_container_name}")
if $task_build; then task_cli_args+=(--build-local); fi
printf 'host: %s (%s)\ncontainer: %s\nworkspace: %s\n' "$task_host" "$task_host_selection" "$task_container_name" "$ISAAC_ROS_WS"
printf 'Git checkout: %s -> /workspaces/atd (read-write)\n' "$task_project_dir"
task_build_tmpdir=""
if [[ "$task_host" = pc ]] && $task_build; then
    task_build_tmpdir="${TMPDIR:-$HOME/.cache/atd/build-tmp}"
    [[ "$task_build_tmpdir" = /* ]] || fail "TMPDIR は絶対パスで指定してください"
    validate_cli_shell_path "$task_build_tmpdir" "TMPDIR"
    printf 'build TMPDIR: %s\n' "$task_build_tmpdir"
fi
if [[ "$task_source_layout" = native ]]; then
    printf 'ROS source: %s/ros2_ws/src -> /workspaces/isaac_ros-dev/src (workspace mount / read-write)\n' "$task_project_dir"
else
    printf 'ROS source: %s/ros2_ws/src -> /workspaces/isaac_ros-dev/src/atd (read-write)\n' "$task_project_dir"
fi
printf 'record: %s -> /data/record (read-write)\nresults: %s -> /data/results (read-write)\n' "$ATD_RECORD_DIR" "$ATD_RESULT_DIR"
printf 'DDS: Fast DDS / UDPv4 loopback\nruntime: %s\n' "$ATD_RUNTIME_REV"
if [[ -n "$ATD_HOST_SSH_DIR" ]]; then printf 'SSH: %s -> /home/admin/.ssh (read-only)\n' "$ATD_HOST_SSH_DIR"; fi
if [[ -n "$ATD_HOST_SSH_AGENT" ]]; then printf 'SSH agent: existing socket -> /run/atd/ssh-agent\n'; fi
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
    "/workspaces/atd": (os.environ["ATD_PROJECT_DIR"], True),
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
if os.environ.get("ATD_SERIAL_ACCESS") == "1":
    expected["/dev"] = ("/dev", True)
    if not container.get("HostConfig", {}).get("Privileged"):
        sys.exit("エラー: Jetson の USB hotplug には既定の privileged コンテナが必要です。コンテナを再作成してください")
for destination, (source, writable) in expected.items():
    mount = mounts.get(destination)
    if (not mount or mount.get("Type") != "bind"
            or os.path.realpath(mount.get("Source", "")) != os.path.realpath(source)
            or mount.get("RW") != writable):
        sys.exit("エラー: 起動済みコンテナの mount が設定と異なります。記録等を終了してコンテナを再作成してください: " + sys.argv[1])
environment = dict(item.split("=", 1) for item in container.get("Config", {}).get("Env", []) if "=" in item)
ssh_source = os.environ.get("ATD_HOST_SSH_DIR", "")
ssh_agent = os.environ.get("ATD_HOST_SSH_AGENT", "")
if ssh_source:
    expected["/home/admin/.ssh"] = (ssh_source, False)
elif environment.get("ATD_HOST_SSH_SHARED") == "1":
    sys.exit("エラー: SSH共有設定が変わりました。コンテナを再作成してください")
if ssh_agent:
    expected["/run/atd/ssh-agent"] = (ssh_agent, False)
    if environment.get("SSH_AUTH_SOCK") != "/run/atd/ssh-agent":
        sys.exit("エラー: SSH agent 設定が変わりました。コンテナを再作成してください")
elif environment.get("ATD_HOST_SSH_AGENT_SHARED") == "1":
    sys.exit("エラー: SSH agent 共有設定が変わりました。コンテナを再作成してください")
for destination in ("/home/admin/.ssh", "/run/atd/ssh-agent"):
    if destination not in expected:
        continue
    source, writable = expected[destination]
    mount = mounts.get(destination)
    if (not mount or mount.get("Type") != "bind"
            or os.path.realpath(mount.get("Source", "")) != os.path.realpath(source)
            or mount.get("RW") != writable):
        sys.exit("エラー: SSH共有 mount が設定と異なります。コンテナを再作成してください")
required = {
    "ATD_RUNTIME_API": "1",
    "ATD_RUNTIME_REV": os.environ["ATD_RUNTIME_REV"],
    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    "FASTDDS_DEFAULT_PROFILES_FILE": "/etc/atd/fastdds.xml",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "SYSTEM_DEFAULT",
    "ROS_STATIC_PEERS": "",
    "RMW_FASTRTPS_USE_QOS_FROM_XML": "0",
}
if os.environ.get("ATD_SERIAL_ACCESS") == "1":
    required["ATD_SERIAL_ACCESS"] = "1"
    required["ATD_HOST_SERIAL_GIDS"] = os.environ["ATD_HOST_SERIAL_GIDS"]
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

source "${task_project_dir}/scripts/lib/isaac_cli.sh"
atd_check_isaac_cli || exit 1

if [[ -n "$task_build_tmpdir" ]]; then
    if [[ -z "${TMPDIR:-}" ]]; then
        mkdir -p -m 700 -- "$task_build_tmpdir" || fail "ビルド用の一時ディレクトリを作成できません: $task_build_tmpdir"
    fi
    [[ -d "$task_build_tmpdir" && -w "$task_build_tmpdir" && -x "$task_build_tmpdir" ]] \
        || fail "TMPDIR を書込み可能なディレクトリにしてください: $task_build_tmpdir"
    export TMPDIR="$task_build_tmpdir"
fi

task_image_revision="$(atd_image_revision "$task_project_dir")" || exit 1
python3 - "$task_project_dir" "$ATD_GENERATED_DOCKER_DIR" "$task_image_revision" "$DOCKER_ARGS_FILE" "$task_source_layout" <<'PY'
import os
from pathlib import Path
import re
import shlex
import sys
import tempfile
root, destination, revision = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
source = (root / 'docker/Dockerfile.atd').read_bytes()
dockerfile = source + ('\n# ATD image source revision: ' + revision + '\n').encode()
arguments = (root / 'docker/dev.dockerargs').read_text().splitlines(keepends=True)
if os.environ['ATD_SERIAL_ACCESS'] == '1':
    # 上流 CLI は既定で privileged。devtmpfs を共有して起動後の ttyACM 追加も見せる。
    arguments.extend(['--mount type=bind,src=/dev,dst=/dev\n',
                      '-e ATD_SERIAL_ACCESS=1\n',
                      '-e ATD_HOST_SERIAL_GIDS=' + os.environ['ATD_HOST_SERIAL_GIDS'] + '\n'])

def add_ssh_mount(source, destination):
    # 上流の追加引数も加算される。標準のユーザー設定に同じ宛先がある場合は再利用する。
    candidates = [Path.home() / '.isaac_ros_dev-dockerargs']
    cli_root = Path(os.environ.get('ATD_SETUP_ROOT', '/'))
    adjacent_args = cli_root / 'usr/lib/isaac-ros-cli/.isaac_ros_dev-dockerargs'
    workspace_args = Path(os.environ['ISAAC_ROS_WS']) / 'scripts/.isaac_ros_dev-dockerargs'
    candidates.append(next((path for path in (adjacent_args, workspace_args,
        cli_root / 'etc/isaac-ros-cli/.isaac_ros_dev-dockerargs') if path.is_file()),
        cli_root / 'etc/isaac-ros-cli/.isaac_ros_dev-dockerargs'))
    lines = arguments + [line for path in candidates if path.is_file() for line in path.read_text().splitlines()]
    found = []
    for line in lines:
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        if destination not in os.path.expandvars(line):
            continue
        line = line.strip()
        if len(line) > 1 and line[0] == line[-1] and line[0] in ('"', "'"):
            line = line[1:-1]
        # 公式テンプレートの `realpath ~/.ssh` を shlex より先に解決する。
        # 任意の shell 式は実行せず、単一パスの正規化だけを行う。
        def realpath(match):
            path = match.group(1).strip()
            if not path or path.startswith('-') or any(c in path for c in '$;|&<>\n\r'):
                sys.exit('エラー: SSH mount の realpath 指定を解釈できません: ' + destination)
            return shlex.quote(str(Path(path).expanduser().resolve()))
        line = re.sub(r'`realpath ([^`]+)`', realpath, line)
        line = os.path.expandvars(line)
        if '`' in line or '$(' in line:
            sys.exit('エラー: SSH mount の未対応の shell 式は実行しません: ' + destination)
        tokens = []
        for token in shlex.split(line):
            if token.startswith(('--mount=', '--volume=')):
                tokens.extend(token.split('=', 1))
            elif token.startswith('-v') and len(token) > 2:
                tokens.extend(['-v', token[2:]])
            else:
                tokens.append(token)
        for index, token in enumerate(tokens):
            if token in ('--mount',) and index + 1 < len(tokens):
                # Docker は大文字小文字を区別せず、同義属性も記載順で上書きする。
                aliases = {'source': 'src', 'destination': 'dst', 'target': 'dst', 'readonly': 'ro'}
                parts = {}
                for item in tokens[index+1].split(','):
                    key, _, value = item.partition('=')
                    key = aliases.get(key.lower(), key.lower())
                    parts[key] = value
                target = parts.get('dst', '')
                if target.rstrip('/') == destination:
                    readonly = 'ro' in parts and parts['ro'].lower() in ('', 'true', '1')
                    found.append((parts.get('src', ''), readonly))
            if token in ('-v', '--volume') and index + 1 < len(tokens):
                parts = tokens[index+1].split(':')
                if len(parts) >= 2 and parts[1].rstrip('/') == destination:
                    found.append((os.path.expanduser(parts[0]), len(parts) >= 3 and 'ro' in parts[2].split(',')))
    if found:
        if len(found) != 1 or os.path.realpath(found[0][0]) != os.path.realpath(source) or not found[0][1]:
            sys.exit('エラー: 既存の追加Docker引数のSSH mountが異なるか書込可能です。同じ宛先への重複を避けてください: ' + destination)
        return
    arguments.append('--mount type=bind,src="' + source + '",dst=' + destination + ',readonly\n')

ssh_directory, ssh_agent = os.environ.get('ATD_HOST_SSH_DIR'), os.environ.get('ATD_HOST_SSH_AGENT')
if ssh_directory:
    add_ssh_mount(ssh_directory, '/home/admin/.ssh')
    arguments.append('-e ATD_HOST_SSH_SHARED=1\n')
if ssh_agent:
    add_ssh_mount(ssh_agent, '/run/atd/ssh-agent')
    arguments.extend(['-e SSH_AUTH_SOCK=/run/atd/ssh-agent\n', '-e ATD_HOST_SSH_AGENT_SHARED=1\n'])
if sys.argv[5] == 'native':
    source_mount = '--mount type=bind,src="${ATD_PROJECT_DIR}/ros2_ws/src",dst=/workspaces/isaac_ros-dev/src/atd'
    if sum(line.rstrip('\r\n') == source_mount for line in arguments) != 1:
        sys.exit('エラー: source mount のテンプレートを識別できません')
    arguments = [line for line in arguments if line.rstrip('\r\n') != source_mount]

# 上流の -v ~/.gitconfig:/home/admin/.gitconfig は bind 元がないとホストに
# ディレクトリを作る。新規コンテナ用に、未作成のときだけ通常ファイルを用意する。
gitconfig = Path.home() / '.gitconfig'
try:
    fd = os.open(gitconfig, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
except FileExistsError:
    if gitconfig.is_dir():
        sys.exit(f'エラー: ホストの {gitconfig} はディレクトリです。内容を確認し、ディレクトリを退避してから再実行してください。')
    if not gitconfig.is_file() or not os.access(gitconfig, os.R_OK):
        sys.exit(f'エラー: ホストの {gitconfig} を読み取れる通常ファイルにしてください。')
except OSError as error:
    sys.exit(f'エラー: ホストの Git 設定ファイルを作成できません: {error}')
else:
    os.close(fd)

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
