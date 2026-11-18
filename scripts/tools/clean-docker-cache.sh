#!/usr/bin/env bash
# ATD のローカルビルドに使う default builder のキャッシュを整理する。
set -euo pipefail

usage() {
    cat <<'EOF'
使い方: ./scripts/tools/clean-docker-cache.sh [--dry-run]
  引数なし   使用量を表示し、Docker の確認後に回収可能なビルドキャッシュを全削除
  --dry-run  使用量を表示して終了
EOF
}

if (($# > 1)); then usage >&2; exit 2; fi
case "${1:-}" in
    ''|--dry-run) ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac

context="$(docker context show)"
export DOCKER_CONTEXT="$context"
endpoint="$(docker context inspect "$context" --format '{{.Endpoints.docker.Host}}')"
printf 'Docker context: %s (%s)\nBuildx builder: default\n' "$context" "$endpoint"
if [[ "$endpoint" != unix:///* && ! "$endpoint" =~ ^tcp://(localhost|127\.0\.0\.1|\[::1\]):[0-9]+$ ]]; then
    printf 'ローカルの Docker を選び、容量を空けたいホストで実行してください。\n' >&2
    exit 1
fi

docker buildx du --builder default | awk '/^(Reclaimable|Total):/ { print }'
if [[ "${1:-}" == --dry-run ]]; then exit 0; fi

printf 'この builder を使う全プロジェクトが対象です。削除後の再ビルドではキャッシュを作り直します。\n'
exec docker buildx prune --builder default --all
