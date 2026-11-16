# workspace が参照するテンプレート。dev.sh が設定する ATD ルートを使用する。
# 上流 CLI の配列 reader は空白で分割するため、プロジェクトパスの空白は非対応。
# ローカルの realsense455 / atd と、公式の isaac_ros 層を探索する。
# atd は COPY 入力の revision を含む生成 Dockerfile を優先し、古い image の再利用を防ぐ。
: "${ATD_PROJECT_DIR:?scripts/dev.sh から起動してください}"
: "${ATD_GENERATED_DOCKER_DIR:?scripts/dev.sh から起動してください}"
CONFIG_DOCKER_SEARCH_DIRS=(
    "${ATD_GENERATED_DOCKER_DIR}"
    "${ATD_PROJECT_DIR}/docker"
    "/etc/isaac-ros-cli/docker"
)
