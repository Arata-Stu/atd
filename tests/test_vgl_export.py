"""ALIKED export の入力・取得整合・公開境界を GPU やモデル取得なしで検証する。"""

from contextlib import nullcontext
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "tools/vgl/export_model.py"
SPEC = importlib.util.spec_from_file_location("atd_vgl_export_model", SCRIPT)
exporter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(exporter)


def graph(shape=(1, 3, 240, 424), *, dtype=1, outputs=None):
    tensor = NS(elem_type=dtype, shape=NS(dim=[NS(dim_value=value) for value in shape]))
    return NS(graph=NS(input=[NS(name="image", type=NS(tensor_type=tensor))],
                       output=[NS(name=name) for name in (outputs or exporter.OUTPUT_NAMES)],
                       initializer=[]), opset_import=[NS(domain="", version=17)])


class VglExportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="atd-vgl-export-test-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def test_import_does_not_load_frameworks_or_make_network_calls(self):
        code = (
            "import runpy,sys,urllib.request; "
            "urllib.request.urlopen=lambda *a,**k: (_ for _ in ()).throw(RuntimeError('network')); "
            "runpy.run_path(sys.argv[1]); "
            "assert not any(name in sys.modules for name in ('torch','torchvision','numpy','onnx'))"
        )
        result = subprocess.run([sys.executable, "-I", "-c", code, str(SCRIPT)],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_invalid_shape_or_existing_output_stops_before_framework_import(self):
        with patch.object(exporter.importlib, "import_module") as imported:
            for width, height in ((0, 240), (424, -1), (True, 240), (20, 20)):
                with self.subTest(width=width, height=height), self.assertRaises(ValueError):
                    exporter.export_aliked(self.root / "model.onnx", self.root / "source", width, height)
            output = self.root / "existing.onnx"
            output.write_bytes(b"do not replace")
            with self.assertRaises(FileExistsError):
                exporter.export_aliked(output, self.root / "source", 424, 240)
            self.assertEqual(output.read_bytes(), b"do not replace")
            imported.assert_not_called()

    def test_prepare_pins_each_source_hash_retains_license_and_reuses_verified_files(self):
        files = {"LICENSE": b"BSD fixture\n", "nets/aliked.py": b"source fixture\n",
                 "models/aliked-n16.pth": b"synthetic weight fixture\n"}
        pins = {name: hashlib.sha256(content).hexdigest() for name, content in files.items()}

        def response(request, timeout):
            self.assertIn(exporter.UPSTREAM_COMMIT, request.full_url)
            self.assertEqual(timeout, 60)
            relative = request.full_url.split(exporter.UPSTREAM_COMMIT + "/", 1)[1]
            return io.BytesIO(files[relative])

        source = self.root / "source"
        with patch.object(exporter, "SOURCE_SHA256", pins), patch.object(exporter.urllib.request, "urlopen", side_effect=response) as fetched:
            metadata = exporter.prepare_source(source)
            self.assertEqual(fetched.call_count, len(files))
            fetched.reset_mock()
            exporter.prepare_source(source)
            fetched.assert_not_called()
        self.assertEqual((source / "LICENSE").read_bytes(), files["LICENSE"])
        self.assertEqual(metadata["files"], pins)
        self.assertEqual(json.loads((source / "atd-source.json").read_text())["commit"], exporter.UPSTREAM_COMMIT)

    def test_wrong_download_hash_never_publishes_file_and_existing_mismatch_stops_before_fetch(self):
        source = self.root / "source"
        pins = {"LICENSE": hashlib.sha256(b"expected").hexdigest()}
        with patch.object(exporter, "SOURCE_SHA256", pins), patch.object(exporter.urllib.request, "urlopen", return_value=io.BytesIO(b"wrong")):
            with self.assertRaisesRegex(ValueError, "SHA256"):
                exporter.prepare_source(source)
        self.assertFalse((source / "LICENSE").exists())
        self.assertEqual(list(source.iterdir()), [])
        (source / "LICENSE").write_text("user content")
        with patch.object(exporter, "SOURCE_SHA256", pins), patch.object(exporter.urllib.request, "urlopen") as fetched:
            with self.assertRaisesRegex(ValueError, "SHA256"):
                exporter.prepare_source(source)
            fetched.assert_not_called()
        self.assertEqual((source / "LICENSE").read_text(), "user content")

    def test_source_symlink_or_conflicting_provenance_is_preserved_without_fetch(self):
        source = self.root / "source"
        source.mkdir()
        other = self.root / "other"
        other.mkdir()
        (source / "nets").symlink_to(other, target_is_directory=True)
        with patch.object(exporter.urllib.request, "urlopen") as fetched:
            with self.assertRaisesRegex(ValueError, "ディレクトリ"):
                exporter.prepare_source(source)
            fetched.assert_not_called()
        self.assertEqual(list(other.iterdir()), [])
        (source / "nets").unlink()
        record = source / "atd-source.json"
        record.write_text('{"different": true}\n')
        with patch.object(exporter.urllib.request, "urlopen") as fetched:
            with self.assertRaisesRegex(ValueError, "管理情報"):
                exporter.prepare_source(source)
            fetched.assert_not_called()
        self.assertEqual(record.read_text(), '{"different": true}\n')

    def test_contract_rejects_wrong_shape_dtype_output_or_external_data(self):
        expected = [1, 3, 240, 424]
        self.assertEqual(exporter.inspect_onnx_contract(graph(), expected)["input_shape"], expected)
        cases = [graph((1, 3, 480, 640)), graph(dtype=10), graph(outputs=["different"])]
        external = graph()
        external.graph.initializer.append(NS(data_location=1))
        cases.append(external)
        opset = graph()
        opset.opset_import[0].version = 18
        cases.append(opset)
        for proto in cases:
            with self.subTest(proto=proto), self.assertRaises(ValueError):
                exporter.inspect_onnx_contract(proto, expected)

    def test_unavailable_cuda_stops_before_source_download(self):
        fake_torch = NS(cuda=NS(is_available=lambda: False))
        with patch.object(exporter.platform, "system", return_value="Linux"), \
             patch.object(exporter.platform, "machine", return_value="x86_64"), \
             patch.object(exporter.importlib, "import_module", side_effect=[fake_torch, NS(), NS(), NS()]), \
             patch.object(exporter, "prepare_source") as prepare:
            with self.assertRaisesRegex(RuntimeError, "CUDA"):
                exporter.export_aliked(self.root / "model.onnx", self.root / "source", 424, 240)
            prepare.assert_not_called()
        self.assertEqual(list(self.root.iterdir()), [])

    def test_export_metadata_and_atomic_publish_use_checked_graph_without_real_gpu(self):
        for invalid in (False, True):
            with self.subTest(invalid=invalid):
                output = self.root / ("invalid.onnx" if invalid else "model.onnx")
                options = []

                def write_model(model, tensor, destination, *, dynamo=True, external_data=True, **kwargs):
                    options.append({"dynamo": dynamo, "external_data": external_data, **kwargs})
                    Path(destination).write_bytes(b"synthetic ONNX fixture")

                torch = NS(cuda=NS(is_available=lambda: True, synchronize=lambda: None,
                                  get_device_name=lambda index: "fake GPU"),
                           inference_mode=nullcontext, zeros=lambda *a, **k: None,
                           ones=lambda *a, **k: None, manual_seed=lambda seed: None,
                           from_numpy=lambda value: NS(cuda=lambda: "tensor"),
                           onnx=NS(export=write_model), __version__="test", version=NS(cuda="test"))
                torchvision = NS(ops=NS(deform_conv2d=lambda *args: None), __version__="test")
                proto = graph((1, 3, 241, 424) if invalid else (1, 3, 240, 424))
                onnx = NS(load=lambda path: proto, checker=NS(check_model=lambda value: None), __version__="test")
                numpy = NS(random=NS(default_rng=lambda seed: NS(random=lambda shape, dtype: None)),
                           float32="float32", __version__="test")
                model = NS(eval=lambda: NS(float=lambda: "model"))
                symbolic = NS(JitScalarType=object(), register_deform_conv2d_onnx_op=lambda: None)
                with patch.object(exporter.platform, "system", return_value="Linux"), \
                     patch.object(exporter.platform, "machine", return_value="x86_64"), \
                     patch.object(exporter.importlib, "import_module", side_effect=[torch, torchvision, onnx, numpy]), \
                     patch.object(exporter, "prepare_source", return_value={"commit": exporter.UPSTREAM_COMMIT}), \
                     patch.object(exporter, "_vendor_imports", return_value=nullcontext((lambda **kwargs: model, symbolic))):
                    if invalid:
                        with self.assertRaisesRegex(ValueError, "入力契約"):
                            exporter.export_aliked(output, self.root / "source", 424, 240)
                        self.assertFalse(output.exists())
                    else:
                        result = exporter.export_aliked(output, self.root / "source", 424, 240)
                        self.assertEqual(result["onnx_sha256"], hashlib.sha256(output.read_bytes()).hexdigest())
                        self.assertEqual(result["input_shape"], [1, 3, 240, 424])
                        self.assertEqual(result["weight_sha256"], exporter.SOURCE_SHA256["models/aliked-n16.pth"])
                        self.assertEqual(result["source"]["commit"], exporter.UPSTREAM_COMMIT)
                self.assertFalse(options[0]["dynamo"])
                self.assertFalse(options[0]["external_data"])
                self.assertEqual(options[0]["opset_version"], 17)
        self.assertEqual([path.name for path in self.root.iterdir()], ["model.onnx"])


if __name__ == "__main__":
    unittest.main()
