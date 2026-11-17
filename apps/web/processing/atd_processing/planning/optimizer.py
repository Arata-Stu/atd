"""Optional helper optimizer in a separate, cancellable Python environment."""

import json
import math
import os
import subprocess
import tempfile
import time
from pathlib import Path


def optimize(document, parameters, check):
    if not document["closed"]:
        raise ValueError(
            "helper最適化は閉路に対応しています。開路へは対応していません（別方式へ fallback しません）"
        )
    try:
        timeout = float(os.environ.get("ATD_RACELINE_TIMEOUT_SECONDS", "0"))
        if not math.isfinite(timeout) or timeout < 0:
            raise ValueError
    except ValueError:
        raise ValueError("ATD_RACELINE_TIMEOUT_SECONDSは0以上の有限の秒数で指定してください（0は時間制限なし）") from None
    python = os.environ.get("ATD_RACELINE_PYTHON")
    if not python or not Path(python).is_file():
        raise ValueError("Raceline 専用環境がありません。atd-python raceline sync を実行し ATD_RACELINE_PYTHON を設定してください。別方式へ fallback しません")
    with tempfile.TemporaryDirectory(prefix="atd-raceline-") as tmp:
        root = Path(tmp)
        source, output = root / "input.json", root / "output.json"
        source.write_text(json.dumps(dict(document=document, parameters=parameters)))
        env = dict(
            os.environ,
            PYTHONPATH=str(Path(__file__).resolve().parents[2]),
            MPLCONFIGDIR=str(root / "mpl"),
            MPLBACKEND="Agg",
        )
        with (root / "log.txt").open("w") as log:
            process = subprocess.Popen(
                [
                    python,
                    "-m",
                    "atd_processing.planning.optimizer_worker",
                    str(source),
                    str(output),
                ],
                stdout=log,
                stderr=log,
                env=env,
            )
            offset = 0
            def relay():
                nonlocal offset
                with (root / "log.txt").open() as reader:
                    reader.seek(offset)
                    chunk = reader.read()
                    offset = reader.tell()
                if chunk:
                    print(chunk, end="", flush=True)
            try:
                deadline = time.monotonic() + timeout if timeout else None
                while process.poll() is None:
                    check()
                    relay()
                    if deadline is not None and time.monotonic() > deadline:
                        detail = (root / "log.txt").read_text()[-1600:]
                        raise ValueError(
                            f"最適化は設定した時間上限（{timeout:g}秒）に達したため中断しました。"
                            "計算の異常や未収束を示すものではありません。"
                            "ATD_RACELINE_TIMEOUT_SECONDSを延長するか、0で時間制限を解除できます。"
                            + ("\n最終処理:\n" + detail if detail else "")
                        )
                    time.sleep(0.1)
                check()
                relay()
                if process.returncode:
                    if output.is_file():
                        failure = json.loads(output.read_text())
                        if failure.get("error"):
                            error = ValueError(failure["error"])
                            if failure.get("diagnostic") is not None:
                                error.diagnostic = failure["diagnostic"]
                            raise error
                    detail = (root / "log.txt").read_text()[-2400:]
                    raise ValueError(
                        "helper実行プロセスが異常終了しました。"
                        "以下の詳細を確認してください。"
                        "依存の読み込みエラーの場合はATD_RACELINE_PYTHONと依存を確認してください。\n"
                        + detail
                    )
                return json.loads(output.read_text())
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
