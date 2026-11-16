#!/usr/bin/env bash
# setup / dev / docker 共通のホスト別 ROS workspace 設定。source 時には操作しない。

atd_validate_workspace_path() {
    local task_path="$1"
    [[ "$task_path" = /* && "$task_path" != / ]] || {
        printf 'エラー: ROS workspace は / 以外の絶対パスで指定してください: %s\n' "$task_path" >&2
        return 1
    }
    # 公式 CLI の shell config reader / shell=True / Docker --mount の制約。
    case "$task_path" in
        *[[:space:]]*|*[\$\`\"\\,]*|*"'"*|*';'*|*'&'*|*'|'*|*'<'*|*'>'*|*'('*|*')'*|*'{'*|*'}'*|*'['*|*']'*|*'*'*|*'?'*|*'!'*)
            printf 'エラー: CLI の制約により ROS workspace に空白や shell 特殊文字は使えません: %s\n' "$task_path" >&2
            return 1 ;;
    esac
}

atd_load_workspace() {
    local task_project="$1" task_profile="$2" task_config
    task_config="${ATD_WORKSPACE_CONFIG:-${task_project}/config/workspace.env}"
    if [[ -f "$task_config" ]]; then
        [[ -r "$task_config" ]] || { printf 'エラー: workspace 設定を読めません: %s\n' "$task_config" >&2; return 1; }
        # storage.env と同様に、ユーザー管理の信頼するローカル Bash 設定。
        source "$task_config" || return $?
    fi
    case "$task_profile" in
        pc) ATD_RESOLVED_ROS_WS="${ATD_ROS_WS:-${ATD_PC_ROS_WS:-${HOME}/workspaces/atd}}" ;;
        jetson) ATD_RESOLVED_ROS_WS="${ATD_ROS_WS:-${ATD_JETSON_ROS_WS:-${HOME}/workspaces/atd}}" ;;
        *) printf 'エラー: 不明な workspace ホスト: %s\n' "$task_profile" >&2; return 1 ;;
    esac
    case "$ATD_RESOLVED_ROS_WS" in '~/'*) ATD_RESOLVED_ROS_WS="${HOME}/${ATD_RESOLVED_ROS_WS:2}" ;; esac
}

atd_save_workspace() {
    local task_project="$1" task_profile="$2" task_workspace="$3"
    atd_validate_workspace_path "$task_workspace" || return $?
    task_workspace="$(python3 -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).resolve())' "$task_workspace")" || return $?
    atd_validate_workspace_path "$task_workspace" || return $?
    python3 - "${ATD_WORKSPACE_CONFIG:-${task_project}/config/workspace.env}" \
        "${task_project}/config/workspace.env.example" "$task_profile" "$task_workspace" <<'PY'
import os
from pathlib import Path
import re
import shlex
import sys
import tempfile

config, example = Path(sys.argv[1]), Path(sys.argv[2])
key = 'ATD_' + sys.argv[3].upper() + '_ROS_WS'
value = Path(sys.argv[4]).resolve()
content = config.read_text() if config.exists() else example.read_text()
pattern = r'^(?:export\s+)?' + key + r'\s*=.*$'
replacement = key + '=' + shlex.quote(str(value))
if re.search(pattern, content, re.MULTILINE):
    content = re.sub(pattern, lambda match: replacement, content, flags=re.MULTILINE)
else:
    content = content.rstrip('\n') + '\n' + replacement + '\n'
config.parent.mkdir(parents=True, exist_ok=True)
with tempfile.NamedTemporaryFile(dir=config.parent, delete=False) as stream:
    temporary = Path(stream.name)
    try:
        stream.write((content.rstrip('\n') + '\n').encode())
        stream.flush()
        os.fchmod(stream.fileno(), 0o600)
        os.replace(temporary, config)
    finally:
        temporary.unlink(missing_ok=True)
print('workspace 設定: ' + str(config))
print(key + '=' + str(value))
PY
}

atd_prepare_workspace() {
    local task_project="$1" task_workspace="$2"
    atd_validate_workspace_path "$task_project" || return $?
    atd_validate_workspace_path "$task_workspace" || return $?
    task_workspace="$(python3 -c 'from pathlib import Path; import sys; print(Path(sys.argv[1]).resolve())' "$task_workspace")" || return $?
    atd_validate_workspace_path "$task_workspace" || return $?
    python3 - "$task_project" "$task_workspace" <<'PY'
import os
from pathlib import Path
import sys
import uuid

project, workspace = (Path(value).resolve() for value in sys.argv[1:])
# 公式 CLI は layer YAML を workspace の親/scripts から探す。
# この checkout の旧テンプレートへの symlink だけを移行し、別設定は上書きしない。
links = [
    (project / 'docker/isaac_ros/build_image_layers.yaml', workspace.parent / 'scripts/.build_image_layers.yaml',
     project / 'scripts/.build_image_layers.yaml'),
    (project / 'docker/isaac_ros/common-config.sh', workspace / 'scripts/.isaac_ros_common-config',
     project / 'ros2_ws/scripts/.isaac_ros_common-config'),
    (project / 'docker/isaac_ros/config.yaml', workspace / '.isaac-ros-cli/config.yaml',
     project / 'ros2_ws/.isaac-ros-cli/config.yaml'),
]
for source, _, _ in links:
    if not source.is_file():
        sys.exit('エラー: workspace の設定がありません: ' + str(source))

source_root = project / 'ros2_ws/src'
if source_root.is_symlink() or not source_root.is_dir():
    sys.exit('エラー: ROS source の配置先がありません: ' + str(source_root))
source_root = source_root.resolve()
source_parent = workspace / 'src'
mountpoint = source_parent / 'atd'
resolved_mountpoint = mountpoint.resolve()
if os.path.lexists(source_parent) and (source_parent.is_symlink() or not source_parent.is_dir()):
    sys.exit('エラー: workspace の src に既存の symlink またはファイルがあります（上書きしません）: ' + str(source_parent))
native_source = source_parent.resolve() == source_root
if not native_source:
    if (source_root == resolved_mountpoint or source_root.is_relative_to(resolved_mountpoint)
            or resolved_mountpoint.is_relative_to(source_root)):
        sys.exit('エラー: ROS source と workspace の mount 先が重複します: ' + str(mountpoint))
    if os.path.lexists(mountpoint) and (
            mountpoint.is_symlink() or not mountpoint.is_dir() or any(mountpoint.iterdir())):
        sys.exit('エラー: workspace の src/atd は空の通常ディレクトリが必要です（既存データを上書きしません）: ' + str(mountpoint))


def link_action(source, destination, legacy):
    if not os.path.lexists(destination):
        return 'create'
    if destination.is_symlink():
        target = Path(os.readlink(destination))
        if not target.is_absolute():
            target = destination.parent / target
        # 旧 target が既に存在しなくても、所有する既知のパスを識別できる。
        target = Path(os.path.abspath(target))
        if target == legacy:
            return 'migrate'
        if target == source and destination.is_file():
            return 'keep'
        sys.exit('エラー: 既存の CLI symlink と衝突します（上書きしません）: ' + str(destination))
    if not destination.is_file() or destination.read_bytes() != source.read_bytes():
        sys.exit('エラー: 既存の CLI 設定と衝突します（上書きしません）: ' + str(destination))
    return 'keep'


changes = [(source, destination, link_action(source, destination, legacy))
           for source, destination, legacy in links]
# 親に shell config があると workspace 内の config より優先される。
parent_shell = workspace.parent / 'scripts/.isaac_ros_common-config'
if os.path.lexists(parent_shell):
    source, _, legacy = links[1]
    changes.append((source, parent_shell, link_action(source, parent_shell, legacy)))
# ここまでの事前確認に失敗した場合は既存ファイル・リンク・データを変更しない。
workspace.mkdir(parents=True, exist_ok=True)
if not os.access(workspace, os.R_OK | os.W_OK | os.X_OK):
    sys.exit('エラー: ROS workspace を読み書きできません: ' + str(workspace))
for source, destination, action in changes:
    if action != 'keep':
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_name(destination.name + '.atd-' + uuid.uuid4().hex)
        try:
            temporary.symlink_to(source)
            os.replace(temporary, destination)
        finally:
            temporary.unlink(missing_ok=True)
    print('[設定あり] ' + str(destination))
if not native_source:
    mountpoint.mkdir(parents=True, exist_ok=True)
print('ROS workspace を準備しました: ' + str(workspace))
container_source = '/workspaces/isaac_ros-dev/src' if native_source else '/workspaces/isaac_ros-dev/src/atd'
print('ROS source: ' + str(source_root) + ' -> ' + container_source + ' (Docker 内)')
PY
}
