#!/usr/bin/env bash
# ローカル Docker の使用量を確認し、選択した範囲のデータを整理する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: ./scripts/tools/clean-docker-cache.sh [--dry-run | --old-images]
        ./scripts/tools/clean-docker-cache.sh --reset-all [--dry-run]
  引数なし      領域別の使用量を表示し、確認後に default builder の未使用キャッシュを削除
  --dry-run     使用量と ATD イメージの削除候補を表示して終了
  --old-images  ATD の完成イメージを一覧表示し、選択した未使用イメージのタグを削除
  --reset-all   全プロジェクトのコンテナを停止・削除し、イメージ・ボリューム・
                追加ネットワーク・ビルドキャッシュを削除（RESET の入力が必要）
                --dry-run 併用で対象の表示のみ
EOF
}

invalid_args() { usage >&2; exit 2; }
action=cache
dry_run=false
for option in "$@"; do
    case "$option" in
        --old-images|--reset-all)
            [[ "$action" == cache ]] || invalid_args
            action="$option" ;;
        --dry-run)
            if $dry_run; then invalid_args; fi
            dry_run=true ;;
        --help|-h) (($# == 1)) || invalid_args; usage; exit 0 ;;
        *) invalid_args ;;
    esac
done
if $dry_run && [[ "$action" == --old-images ]]; then invalid_args; fi

context="$(docker context show)"
export DOCKER_CONTEXT="$context"
endpoint="$(docker context inspect "$context" --format '{{.Endpoints.docker.Host}}')"
printf 'Docker context: %s (%s)\n' "$context" "$endpoint"
if [[ "$endpoint" != unix:///* && ! "$endpoint" =~ ^tcp://(localhost|127\.0\.0\.1|\[::1\]):[0-9]+$ ]]; then
    printf 'ローカルの Docker を選び、容量を空けたいホストで実行してください。\n' >&2
    exit 1
fi

docker system df
if [[ "$action" == --reset-all ]]; then
    # Swarm manager の network prune はクラスタにも作用するため、単体の Engine に限定する。
    daemon_format='{{.ID}} {{.Swarm.LocalNodeState}}'
    daemon="$(docker info --format "$daemon_format")"
    if [[ ! "$daemon" =~ ^[^[:space:]]+[[:space:]]inactive$ ]]; then
        printf '全削除は単体の Docker Engine で実行してください（Engine ID / Swarm: %s）。\n' "$daemon" >&2
        exit 1
    fi
    printf '\n全プロジェクトの削除対象（起動中のコンテナも停止します）\n'
    docker container ls --all
    docker image ls --all
    docker volume ls
    docker network ls --filter type=custom
    printf '\nUbuntu・Isaac ROS を含む全イメージ、コンテナの書き込み層、名前付き・匿名のローカルボリューム内のデータも削除します。\n'
    printf 'ホストから bind mount した checkout・bag・地図・モデルはホスト側に残ります。\n'
    if $dry_run; then exit 0; fi

    # 確認中に起動した新しいコンテナまで停止しない。
    running="$(docker container ls --quiet --no-trunc)"
    printf '上記を削除するには RESET と入力して Enter（それ以外は中止）: '
    if ! IFS= read -r answer || [[ "$answer" != RESET ]]; then printf '中止しました。\n'; exit 0; fi
    current_daemon="$(docker info --format "$daemon_format")"
    if [[ "$current_daemon" != "$daemon" ]]; then
        printf '対象の Engine または Swarm の状態が変わりました。再度確認してください。\n' >&2
        exit 1
    fi
    if [[ -n "$running" ]]; then
        while IFS= read -r id; do docker container stop "$id" </dev/null; done <<< "$running"
    fi
    docker system prune --all --force
    # system prune --volumes が削除するのは匿名ボリュームのみ。
    docker volume prune --all --force

    printf '\n整理後の Docker 使用量\n種類\t個数\tサイズ\n'
    remaining="$(docker system df --format $'{{.Type}}\t{{.TotalCount}}\t{{.Size}}')"
    printf '%s\n' "$remaining"
    volumes="$(docker volume ls --quiet)"
    networks="$(docker network ls --filter type=custom --quiet)"
    if [[ -n "$volumes$networks" ]] || ! awk -F '\t' '$2 != "0" {left=1} END {exit left}' <<< "$remaining"; then
        printf 'リソースが残っています。--reset-all --dry-run で使用中のデータや外部ドライバのボリュームを確認してください。\n' >&2
        exit 1
    fi
    printf 'ローカル Docker のデータを削除しました。\n'
    exit 0
fi

image_tool="$(dirname -- "${BASH_SOURCE[0]}")/../lib/docker_images.py"
if [[ "$action" == --old-images ]]; then
    exec python3 "$image_tool" --remove
fi

printf '\nBuildx ビルドキャッシュ (builder: default)\n'
docker buildx du --builder default | awk '/^(Reclaimable|Total):/ { print }'
if $dry_run; then
    exec python3 "$image_tool"
fi

printf 'Images 欄の古い ATD イメージは %s --old-images で選択できます。\n' "$0"
printf 'ここでは全プロジェクトの未使用ビルドキャッシュを削除します。確認は y + Enter、完了時は Total が表示されます。\n'
exec docker buildx prune --builder default --all
