"""ローカル Docker の ATD 完成イメージを ID 単位で表示し、選択したタグを整理する。"""

import argparse
from dataclasses import dataclass, field
import json
import re
import subprocess
import sys


REGISTRY = "nvcr.io/nvidia/isaac/ros"
CACHED = "cached_isaac_run_dev_image_local:latest"
ATD_KEY = re.compile(r"isaac_ros-realsense455-atd_[0-9a-f]{32}-(amd64|arm64-jetpack)")


def atd_platform(ref):
    repository, _, tag = ref.rpartition(":")
    if repository == REGISTRY:
        key = tag
    elif repository.startswith(REGISTRY + "/") and tag == "latest":
        key = repository[len(REGISTRY) + 1:]
    else:
        return None
    match = ATD_KEY.fullmatch(key)
    return match.group(1) if match else None


def docker(*args):
    return subprocess.check_output(["docker", *args], text=True)


def used_by_container(image_id):
    return bool(docker("container", "ls", "--all", "--quiet", "--filter",
                       "ancestor=sha256:" + image_id).strip())


@dataclass
class Image:
    id: str
    size: str
    tags: set[str] = field(default_factory=set)
    platforms: set[str] = field(default_factory=set)
    keep: str = ""


def inventory():
    listing = docker("image", "ls", "--all", "--no-trunc", "--format",
                     "{{.ID}}\t{{.Repository}}:{{.Tag}}\t{{.Size}}")
    images = {}
    for row in listing.splitlines():
        image_id, tag, size = row.split("\t")
        image_id = image_id.removeprefix("sha256:")
        if not re.fullmatch("[0-9a-f]{64}", image_id):
            raise ValueError(f"Docker のイメージ ID を読み取れません: {image_id}")
        image = images.setdefault(image_id, Image(image_id, size))
        image.tags.add(tag)
        if platform := atd_platform(tag):
            image.platforms.add(platform)
    result = {key: image for key, image in images.items() if image.platforms}
    for image in result.values():
        if CACHED in image.tags:
            image.keep = "Isaac ROS CLI のキャッシュ別名"
        elif any(atd_platform(tag) is None for tag in image.tags):
            image.keep = "別用途のタグあり"
        elif used_by_container(image.id):
            image.keep = "コンテナから参照あり（停止中を含む）"
    return result


def choose(images, answer):
    if not answer or answer.lower() == "n":
        return []
    candidates = [image for image in images.values() if not image.keep]
    if answer == "all":
        cached_platforms = {platform for image in images.values() if CACHED in image.tags
                            for platform in image.platforms}
        if not all(image.platforms <= cached_platforms for image in candidates):
            raise ValueError("CLI のキャッシュ別名で保持する ATD イメージを各機種で確認できません。削除する ID を個別に指定してください。")
        return candidates
    selected = []
    for token in answer.split():
        prefix = token.removeprefix("sha256:")
        matches = [image for image in images.values() if image.id.startswith(prefix)]
        if not re.fullmatch("[0-9a-f]{1,64}", prefix) or len(matches) != 1 or matches[0].keep:
            raise ValueError(f"削除候補を一意に指定してください: {token}")
        if matches[0] not in selected:
            selected.append(matches[0])
    return selected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--remove", action="store_true")
    args = parser.parse_args()
    images = inventory()
    print(f"ATD 完成イメージ: {len(images)} ID（サイズは共有レイヤーを含む）")
    for image in images.values():
        status = "保持: " + image.keep if image.keep else "削除候補"
        print(f"  {image.id}  {image.size}  [{status}]")
        for tag in sorted(image.tags):
            print(f"    {tag}")
    if not args.remove or not any(not image.keep for image in images.values()):
        return

    try:
        answer = input("削除する ID の先頭を空白区切りで入力（候補を全選択: all / 中止: Enter）: ").strip()
    except EOFError:
        answer = ""
    selected = choose(images, answer)
    if not selected:
        print("中止しました。")
        return
    if answer == "all":
        cached_id = docker("image", "inspect", "--format", "{{.Id}}", CACHED).strip().removeprefix("sha256:")
        if cached_id not in images or CACHED not in images[cached_id].tags:
            raise ValueError("CLI のキャッシュ別名が変わりました。再実行して候補を確認してください。")
    for image in selected:
        # 確認を待つ間に進んだ build やコンテナ起動を検出する。別 ID に移ったタグも除外する。
        current = json.loads(docker("image", "inspect", "--format", "{{json .RepoTags}}",
                                    "sha256:" + image.id))
        if set(current or []) != image.tags or used_by_container(image.id):
            raise ValueError(f"{image.id[:12]} のタグまたは使用状態が変わりました。再実行して候補を確認してください。")
        subprocess.run(["docker", "image", "rm", "--no-prune", *sorted(image.tags)], check=True)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        sys.exit(f"イメージの整理を終了しました: {error}")
    except KeyboardInterrupt:
        sys.exit(130)
