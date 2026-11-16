"""Notebook 上で固定 ALIKED-n16 checkpoint を ONNX へ変換する。

TensorRT engine は生成しない。ALIKED-TensorRT の upstream ソースは実行時に
固定 commit / SHA256 で取得し、BSD-3-Clause の LICENSE とともに保持する。
export の接続方法は Arata-Stu/kart の tools/vgl/lab.py を参考にしている。
"""

from contextlib import contextmanager
import hashlib
import importlib
import inspect
import json
import os
from pathlib import Path
import platform
import sys
import tempfile
import types
import urllib.request


UPSTREAM_REPOSITORY = "https://github.com/ajuric/aliked-tensorrt"
UPSTREAM_COMMIT = "96f1f9e7a9932a5da4f0d1aa62017c0fd48141ce"
SOURCE_SHA256 = {
    "LICENSE": "4a039828625f985f2c94463f6a4c8eacd483c71c01d336c428830437f065639d",
    "nets/aliked.py": "59f3e1827045ec42b0e5d704eadf3531056cb3dd3bc5e4c1219863dee50f211f",
    "nets/blocks.py": "552b3517716acf8db6f6ddffb33ab181aa6f5ed3b4b2a091e25468d8d72e75b4",
    "nets/padder.py": "b8f2fedaa1f17e691d268432f1b00229840f73ace0792315615cea804afc6fe2",
    "nets/soft_detect.py": "6fd31b91f4c26d5ccc7051a18785b82a52df647730cc19837611708611e4a290",
    "deform_conv2d_onnx_exporter.py": "83097f2545154f0bd642580c77bce11a8fe84267b92d8bbe4d975385a021082c",
    "models/aliked-n16.pth": "5be8704840ed662d9d8c561bf7279c222092674e7eb05fd0feab94899e9d82f2",
}
TOP_K = 2048
OPSET = 17
OUTPUT_NAMES = ["keypoints", "descriptors", "scores"]


def sha256(path: Path) -> str:
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def input_shape(width: int, height: int) -> list[int]:
    if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
        raise ValueError("width / height は正の整数が必要です")
    if width * height < TOP_K:
        raise ValueError(f"入力の画素数は TopK={TOP_K} 以上が必要です")
    return [1, 3, height, width]


def _download(relative: str, destination: Path, expected: str) -> None:
    """検証できたファイルだけを公開する。重みを含め import 時には呼ばない。"""
    destination.parent.mkdir(parents=True, exist_ok=True)
    url = f"https://raw.githubusercontent.com/ajuric/aliked-tensorrt/{UPSTREAM_COMMIT}/{relative}"
    request = urllib.request.Request(url, headers={"User-Agent": "ATD-VGL-export"})
    temporary = None
    try:
        with urllib.request.urlopen(request, timeout=60) as response:
            with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as stream:
                temporary = Path(stream.name)
                digest = hashlib.sha256()
                while chunk := response.read(1024 * 1024):
                    digest.update(chunk)
                    stream.write(chunk)
                stream.flush()
                os.fsync(stream.fileno())
                os.fchmod(stream.fileno(), 0o644)
        if digest.hexdigest() != expected:
            raise ValueError(f"upstream の SHA256 が一致しません: {relative}")
        # 同時実行でも、先に置かれた別ファイルを置換しない。
        try:
            os.link(temporary, destination)
        except FileExistsError:
            if destination.is_symlink() or not destination.is_file() or sha256(destination) != expected:
                raise ValueError(f"既存の upstream ファイルと衝突しました: {destination}")
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def prepare_source(source_dir: Path) -> dict:
    source_dir = Path(source_dir)
    if source_dir.is_symlink() or (source_dir.exists() and not source_dir.is_dir()):
        raise ValueError(f"source は通常のディレクトリを指定してください: {source_dir}")
    provenance = {
        "repository": UPSTREAM_REPOSITORY,
        "commit": UPSTREAM_COMMIT,
        "license": "BSD-3-Clause",
        "files": dict(SOURCE_SHA256),
    }
    record = source_dir / "atd-source.json"
    content = json.dumps(provenance, ensure_ascii=False, indent=2) + "\n"
    if record.is_symlink() or (record.exists() and (not record.is_file() or record.read_text() != content)):
        raise ValueError(f"既存の source 管理情報と衝突しました: {record}")
    # 既存内容が異なる場合は、追加取得より前に停止する。
    for relative, expected in SOURCE_SHA256.items():
        path = source_dir / relative
        for parent in path.parents:
            if parent == source_dir:
                break
            if os.path.lexists(parent) and (parent.is_symlink() or not parent.is_dir()):
                raise ValueError(f"upstream 内のディレクトリと衝突しました: {parent}")
        if os.path.lexists(path) and (
                path.is_symlink() or not path.is_file() or sha256(path) != expected):
            raise ValueError(f"既存の upstream ファイルの SHA256 が一致しません: {path}")
    for relative, expected in SOURCE_SHA256.items():
        path = source_dir / relative
        if not path.exists():
            print(f"ALIKED source を準備: {relative}", flush=True)
            _download(relative, path, expected)
    if not record.exists():
        record.write_text(content)
    return provenance


def inspect_onnx_contract(proto, shape: list[int]) -> dict:
    """checker 後の公開 I/O 契約を検査する。GPU や重い module は不要。"""
    inputs = list(proto.graph.input)
    if len(inputs) != 1 or inputs[0].name != "image":
        raise ValueError("ALIKED ONNX の入力は image 一つである必要があります")
    tensor = inputs[0].type.tensor_type
    actual = [dimension.dim_value for dimension in tensor.shape.dim]
    if actual != shape or tensor.elem_type != 1:  # TensorProto.FLOAT
        raise ValueError(f"ALIKED ONNX の FP32 / NCHW 入力契約が異なります: {actual}")
    outputs = [value.name for value in proto.graph.output]
    if outputs != OUTPUT_NAMES:
        raise ValueError(f"ALIKED ONNX の出力名が異なります: {outputs}")
    if not any(item.domain in ("", "ai.onnx") and item.version == OPSET
               for item in proto.opset_import):
        raise ValueError(f"ALIKED ONNX に opset {OPSET} がありません")
    if any(item.data_location == 1 for item in proto.graph.initializer):
        raise ValueError("この exporter は単一ファイルの ONNX を生成する必要があります")
    return {"input_name": "image", "input_shape": actual, "input_dtype": "float32",
            "output_names": outputs, "opset": OPSET}


@contextmanager
def _vendor_imports(source_dir: Path):
    """upstream の絶対 nets import を他のアプリの module と混在させない。"""
    if any(name == "nets" or name.startswith("nets.") or name == "deform_conv2d_onnx_exporter"
           for name in sys.modules):
        raise RuntimeError("ALIKED export は nets module を読み込んでいない独立プロセスで実行してください")
    package = types.ModuleType("nets")
    package.__path__ = [str(source_dir / "nets")]
    previous_path = list(sys.path)
    previous_bytecode = sys.dont_write_bytecode
    sys.modules["nets"] = package
    sys.path.insert(0, str(source_dir))
    sys.dont_write_bytecode = True
    try:
        yield (importlib.import_module("nets.aliked").ALIKED,
               importlib.import_module("deform_conv2d_onnx_exporter"))
    finally:
        sys.path[:] = previous_path
        sys.dont_write_bytecode = previous_bytecode
        for name in list(sys.modules):
            if name == "nets" or name.startswith("nets.") or name == "deform_conv2d_onnx_exporter":
                sys.modules.pop(name, None)


def _configure_symbolic_types(exporter) -> str:
    if getattr(exporter, "JitScalarType", None) is not None:
        return "upstream"
    name = "torch.onnx._internal.torchscript_exporter._type_utils"
    try:
        scalar_type = importlib.import_module(name).JitScalarType
    except (ImportError, AttributeError) as error:
        raise RuntimeError("固定 ALIKED exporter に必要な PyTorch symbolic 型がありません") from error
    for method in ("from_value", "from_dtype", "onnx_type", "dtype"):
        if not callable(getattr(scalar_type, method, None)):
            raise RuntimeError(f"未対応の PyTorch symbolic 型です: {method}")
    exporter.JitScalarType = scalar_type
    return name


def export_aliked(output_onnx: Path, source_dir: Path, width: int, height: int) -> dict:
    shape = input_shape(width, height)
    output_onnx, source_dir = Path(output_onnx), Path(source_dir)
    if os.path.lexists(output_onnx):
        raise FileExistsError(f"既存 ONNX は上書きしません: {output_onnx}")
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        raise RuntimeError("ALIKED の元重みからの export は Notebook の Linux x86_64 Docker で実行してください")
    # Jetson 側の engine build / 検査ではこれらを import しない。
    try:
        torch = importlib.import_module("torch")
        torchvision = importlib.import_module("torchvision")
        onnx = importlib.import_module("onnx")
        numpy = importlib.import_module("numpy")
    except (ImportError, OSError, RuntimeError) as error:
        raise RuntimeError("vgl-export の uv 環境を同期してください") from error
    if not torch.cuda.is_available():
        raise RuntimeError("ALIKED export には CUDA 対応 PyTorch と利用可能な GPU が必要です")
    try:
        with torch.inference_mode():
            torchvision.ops.deform_conv2d(
                torch.zeros(1, 1, 4, 4, device="cuda"),
                torch.zeros(1, 18, 2, 2, device="cuda"),
                torch.ones(1, 1, 3, 3, device="cuda"),
            )
            torch.cuda.synchronize()
    except (RuntimeError, OSError, AttributeError) as error:
        raise RuntimeError("torchvision の CUDA deform_conv2d を実行できません") from error
    provenance = prepare_source(source_dir)
    output_onnx.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(suffix=".onnx", dir=output_onnx.parent, delete=False) as stream:
            temporary = Path(stream.name)
        with _vendor_imports(source_dir.resolve()) as (model_type, exporter):
            symbolic = _configure_symbolic_types(exporter)
            exporter.register_deform_conv2d_onnx_op()
            torch.manual_seed(0)
            model = model_type(model_name="aliked-n16", device="cuda", top_k=TOP_K).eval().float()
            tensor = torch.from_numpy(numpy.random.default_rng(0).random(shape, dtype=numpy.float32)).cuda()
            options = dict(opset_version=OPSET, input_names=["image"], output_names=OUTPUT_NAMES,
                           export_params=True, do_constant_folding=True)
            parameters = inspect.signature(torch.onnx.export).parameters
            if "dynamo" in parameters:
                options["dynamo"] = False
            if "external_data" in parameters:
                options["external_data"] = False
            with torch.inference_mode():
                torch.onnx.export(model, tensor, str(temporary), **options)
        proto = onnx.load(str(temporary))
        onnx.checker.check_model(proto)
        contract = inspect_onnx_contract(proto, shape)
        metadata = {
            "model": "aliked-n16", "feature_type": "aliked", "top_k": TOP_K,
            "weight_sha256": SOURCE_SHA256["models/aliked-n16.pth"],
            "onnx_sha256": sha256(temporary), "source": provenance, **contract,
            "export": {
                "torch_version": torch.__version__, "torchvision_version": torchvision.__version__,
                "onnx_version": onnx.__version__, "numpy_version": numpy.__version__,
                "python_version": platform.python_version(), "cuda_runtime": torch.version.cuda,
                "gpu": torch.cuda.get_device_name(0), "symbolic_types": symbolic,
                "input": "synthetic_seed_0_export_only",
                "preprocessing_contract": "RGB float32 [0,1] NCHW",
            },
        }
        # 完成した一つの ONNX だけを公開し、別処理の出力を置換しない。
        os.link(temporary, output_onnx)
        print(f"ALIKED ONNX export 完了: {output_onnx}", flush=True)
        return metadata
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
