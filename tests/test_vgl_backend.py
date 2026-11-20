"""VGL bundle・export・Jetson build の境界を CPU fixture / builder mocks で確認する。"""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
from copy import deepcopy
import io
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from tools.vgl import bundle, build_engine, cli, export_model


CONFIG_PREFIX = '''# aliked_detector { ignored comment }
unrelated { model_name: "aliked_detector { min: 999 }" }
aliked_detector {
  max_key_points_for_tensorrt: 4000
  tensorrt_config {
    model_name: "aliked"
    use_fp16: true
    input_dimension {'''
CONFIG_PROFILE = '''
      # min: 999 and max: 999 are comments, not dimensions.
      min: 1 min: 3 min: 100 min: 100
      opt: 1 opt: 3 opt: 1200 opt: 1920
      max: 1 max: 3 max: 2400 max: 4000
    '''
CONFIG_SUFFIX = '''}
  }
}
super_point_detector {
  tensorrt_config { input_dimension { min: 1 min: 1 min: 100 min: 100 } }
}
'''
CONFIG = CONFIG_PREFIX + CONFIG_PROFILE + CONFIG_SUFFIX
MODEL = {'family': 'aliked_lightglue', 'mode': 'aliked',
         'width': 424, 'height': 240, 'top_k': 2048,
         'input_shape': [1, 3, 240, 424], 'input_dtype': 'float32',
         'output_names': ['keypoints', 'descriptors', 'scores']}


class VglBackendTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='atd-vgl-backend-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        self.results = self.root / 'results'
        self.results.mkdir()
        self.share = self.root / 'installed/share'
        self.prefix = self.root / 'installed'
        self.write_inputs(self.share)
        for name in ('export_extractor_engine', 'export_lightglue_engine'):
            path = self.prefix / 'bin/visual_mapping' / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#!/bin/sh\nexit 99\n')
            path.chmod(0o755)
        self.hardware = {'architecture': 'aarch64', 'gpu_uuid': 'fixture-gpu',
                         'gpu_name': 'fixture Jetson GPU', 'tensorrt_version': 'fixture',
                         'atd_runtime_revision': 'fixture-revision'}

    @staticmethod
    def write_inputs(root):
        inputs = {
            'models/aliked_lightglue/aliked.onnx': b'official ONNX fixture',
            'models/aliked_lightglue/lightglue_aliked.onnx': b'matcher ONNX fixture',
            'models/aliked_lightglue/lightglue_aliked_b16.onnx': b'batched matcher fixture',
            'configs/isaac/keypoint_creation_config.pb.txt': CONFIG.encode(),
            'configs/isaac/matching_task_worker_config.pb.txt': b'matching_params { fixture: true }\n',
        }
        for relative, content in inputs.items():
            path = root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)

    def make_bundle(self, export_id='export1'):
        path = bundle.export_path(self.results, 'asset1', export_id)
        self.write_inputs(path)
        bundle.make_manifest(path, 'asset1', export_id, deepcopy(MODEL), {'origin': 'CPU fixture'})
        return path

    @staticmethod
    def change_manifest(path, change):
        data = json.loads((path / 'manifest.json').read_text())
        change(data)
        bundle.write_json(path / 'manifest.json', data)

    def cli_args(self, action, *extra):
        return [action, '--result-dir', str(self.results), '--asset-id', 'asset1',
                '--export-id', 'export1', *extra]

    def run_cli(self, arguments):
        stdout, stderr = io.StringIO(), io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            status = cli.main(arguments)
        return status, stdout.getvalue(), stderr.getvalue()

    def only_run(self):
        runs = list((self.results / 'runs').iterdir())
        self.assertEqual(len(runs), 1)
        return runs[0]

    def test_valid_bundle_hashes_exactly_the_portable_file_set(self):
        path = self.make_bundle()
        manifest = bundle.validate_bundle(path)
        self.assertEqual(manifest['files'], bundle.inventory(path))
        self.assertEqual(manifest['asset_id'], 'asset1')
        self.assertNotIn('manifest.json', manifest['files'])
        self.assertEqual(manifest['files']['models/aliked_lightglue/aliked.onnx']['sha256'],
                         bundle.sha256(path / 'models/aliked_lightglue/aliked.onnx'))

    def test_changed_missing_and_unlisted_files_each_fail_bundle_validation(self):
        for case in ('changed', 'missing', 'extra'):
            with self.subTest(case=case):
                path = self.make_bundle(case)
                model = path / 'models/aliked_lightglue/aliked.onnx'
                if case == 'changed':
                    model.write_bytes(b'changed content')
                elif case == 'missing':
                    model.unlink()
                else:
                    (path / 'models/unlisted.onnx').write_bytes(b'unlisted')
                with self.assertRaisesRegex(ValueError, '検証失敗'):
                    bundle.validate_bundle(path)

    def test_bundle_rejects_symlink_file_and_directory_even_with_same_bytes(self):
        path = self.make_bundle()
        model = path / 'models/aliked_lightglue/aliked.onnx'
        content = model.read_bytes()
        outside = self.root / 'outside.onnx'
        outside.write_bytes(content)
        model.unlink()
        model.symlink_to(outside)
        with self.assertRaises(ValueError):
            bundle.validate_bundle(path)
        model.unlink()
        model.write_bytes(content)
        original = path / 'models/aliked_lightglue'
        moved = self.root / 'outside-models'
        original.rename(moved)
        original.symlink_to(moved, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            bundle.validate_bundle(path)
        self.assertEqual((moved / 'aliked.onnx').read_bytes(), content)

    def test_manifest_traversal_paths_fail_before_inventory_reads(self):
        path = self.make_bundle()
        original = json.loads((path / 'manifest.json').read_text())
        bad_paths = ('../outside', '/tmp/outside', 'models/../outside', 'models//file',
                     'models/./file', 'models\\file', 'weights/file', 'models/\x00bad')
        for name in bad_paths:
            with self.subTest(name=name):
                data = deepcopy(original)
                data['files'][name] = {'sha256': '0' * 64, 'size': 1}
                bundle.write_json(path / 'manifest.json', data)
                with patch.object(bundle, 'inventory') as inventory, self.assertRaises(ValueError):
                    bundle.validate_bundle(path)
                inventory.assert_not_called()

    def test_schema_ids_sizes_and_model_types_are_strict(self):
        path = self.make_bundle()
        original = json.loads((path / 'manifest.json').read_text())
        changes = [lambda item, value=value: item.update(schema_version=value)
                   for value in (True, '1', 2, None)]
        changes.extend([
            lambda item: item.update(kind='other.bundle'),
            lambda item: item.update(asset_id='../other'),
            lambda item: item.update(export_id='has space'),
            lambda item: item.update(asset_id='x' * 97),
            lambda item: item['model'].update(width=True),
            lambda item: item['model'].update(top_k=4096),
            lambda item: item['files']['models/aliked_lightglue/aliked.onnx'].update(size=True),
            lambda item: item['files']['models/aliked_lightglue/aliked.onnx'].update(sha256='invalid'),
        ])
        for index, change in enumerate(changes):
            with self.subTest(case=index):
                data = deepcopy(original)
                change(data)
                bundle.write_json(path / 'manifest.json', data)
                with self.assertRaises(ValueError):
                    bundle.validate_bundle(path)

    def test_copy_inputs_omits_gpu_caches_and_checkpoints_but_keeps_external_data(self):
        models = self.share / 'models/aliked_lightglue'
        for suffix in ('.engine', '.plan', '.so', '.a', '.pt', '.pth', '.pyc'):
            (models / ('ignored' + suffix)).write_bytes(b'excluded')
        (models / 'aliked.onnx.data').write_bytes(b'external tensor data')
        destination = self.root / 'copied'
        bundle.copy_inputs(self.share, destination)
        self.assertEqual((destination / 'models/aliked_lightglue/aliked.onnx.data').read_bytes(), b'external tensor data')
        self.assertEqual([path for path in destination.rglob('ignored*')], [])
        self.assertFalse(any(path.is_symlink() for path in destination.rglob('*')))

    def test_fixed_extractor_profile_changes_only_the_nested_aliked_dimensions(self):
        result = build_engine.fixed_extractor_config(CONFIG, 848, 480)
        self.assertTrue(result.startswith(CONFIG_PREFIX))
        self.assertTrue(result.endswith(CONFIG_SUFFIX))
        changed_profile = result[len(CONFIG_PREFIX):-len(CONFIG_SUFFIX)]
        self.assertIn('# min: 999 and max: 999 are comments', changed_profile)
        numeric_profile = re.sub(r'#[^\n]*|//[^\n]*', '', changed_profile)
        for key in ('min', 'opt', 'max'):
            self.assertEqual([int(value) for value in re.findall(r'\b' + key + r':\s*(\d+)', numeric_profile)],
                             [1, 3, 480, 848])
        with self.assertRaises(ValueError):
            build_engine.fixed_extractor_config(CONFIG + '\naliked_detector {}\n', 848, 480)
        with self.assertRaises(ValueError):
            build_engine.fixed_extractor_config(CONFIG.replace('min: 1 min: 3', 'min: 1 min: 1'), 848, 480)

    @staticmethod
    def tensor_value(name, shape, dtype=1):
        return NS(name=name, type=NS(tensor_type=NS(
            elem_type=dtype, shape=NS(dim=[NS(dim_value=value) for value in shape]))))

    def model_outputs(self, *, count=2048):
        return [self.tensor_value('keypoints', [count, 2]),
                self.tensor_value('descriptors', [count, 128]),
                self.tensor_value('scores', [count])]

    def test_package_contract_requires_static_top_k_and_correct_descriptor_width(self):
        self.assertEqual(cli.aliked_output_contract(self.model_outputs(), require_static=True)
                         ['top_k_validation'], 'onnx_static')
        for count in (1024, 4096, 0):
            with self.subTest(count=count), self.assertRaises(ValueError):
                cli.aliked_output_contract(self.model_outputs(count=count), require_static=True)
        outputs = self.model_outputs()
        outputs[1] = self.tensor_value('descriptors', [2048, 256])
        with self.assertRaises(ValueError):
            cli.aliked_output_contract(outputs, require_static=True)

    def test_true_export_accepts_symbolic_top_k_but_records_the_source_of_its_contract(self):
        result = cli.aliked_output_contract(self.model_outputs(count=0), require_static=False)
        self.assertEqual(result['top_k_validation'], 'pinned_exporter')
        self.assertEqual(result['output_shapes']['keypoints'], [0, 2])
        with self.assertRaises(ValueError):
            cli.aliked_output_contract(self.model_outputs(count=4096), require_static=False)

    def test_dynamic_onnx_height_does_not_hide_a_conflicting_static_width(self):
        graph = NS(graph=NS(input=[self.tensor_value('image', [1, 3, 0, 848])],
                            output=self.model_outputs()))
        fake_onnx = NS(load=lambda *args, **kwargs: graph, checker=NS(check_model=lambda value: None),
                       TensorProto=NS(FLOAT=1))
        with patch.dict(sys.modules, {'onnx': fake_onnx}):
            with self.assertRaisesRegex(ValueError, 'width'):
                cli.onnx_contract(self.root / 'model.onnx', 640, 480)
            model = cli.onnx_contract(self.root / 'model.onnx', None, 480, require_static_top_k=True)
        self.assertEqual(model['input_shape'], [1, 3, 480, 848])
        self.assertEqual(model['top_k'], 2048)

    def test_onnx_extra_input_and_wrong_output_dtype_are_rejected(self):
        graph = NS(graph=NS(input=[self.tensor_value('image', [1, 3, 480, 848]),
                                  self.tensor_value('other', [1])], output=self.model_outputs()))
        fake_onnx = NS(load=lambda *args, **kwargs: graph, checker=NS(check_model=lambda value: None),
                       TensorProto=NS(FLOAT=1))
        with patch.dict(sys.modules, {'onnx': fake_onnx}):
            with self.assertRaisesRegex(ValueError, 'input'):
                cli.onnx_contract(self.root / 'model.onnx', None, None)
        outputs = self.model_outputs()
        outputs[0] = self.tensor_value('keypoints', [2048, 2], dtype=7)
        with self.assertRaisesRegex(ValueError, 'FP32'):
            cli.aliked_output_contract(outputs, require_static=True)

    def test_true_export_replaces_only_staging_onnx_and_publishes_license_and_provenance(self):
        official = self.share / 'models/aliked_lightglue/aliked.onnx'
        original = official.read_bytes()

        def export(output, source, width, height):
            self.assertFalse(output.exists())
            self.assertEqual((width, height), (424, 240))
            source.mkdir(parents=True)
            (source / 'LICENSE').write_text('BSD fixture license\n')
            output.write_bytes(b'custom exported ONNX')
            print('mock true export completed')
            return {'weight_sha256': 'w' * 64, 'onnx_sha256': bundle.sha256(output),
                    'source': {'commit': 'pinned-test-commit'}}

        with patch.object(cli.platform, 'system', return_value='Linux'), \
             patch.object(cli.platform, 'machine', return_value='x86_64'), \
             patch.object(export_model, 'export_aliked', side_effect=export) as converted, \
             patch.object(cli, 'onnx_contract', return_value=deepcopy(MODEL)):
            status, stdout, stderr = self.run_cli(self.cli_args(
                'export', '--host', 'pc', '--width', '424', '--height', '240', '--source-dir', str(self.share)))
        self.assertEqual(status, 0, stdout + stderr)
        converted.assert_called_once()
        self.assertEqual(official.read_bytes(), original)
        destination = bundle.export_path(self.results, 'asset1', 'export1')
        manifest = bundle.validate_bundle(destination)
        self.assertEqual((destination / 'models/aliked_lightglue/aliked.onnx').read_bytes(), b'custom exported ONNX')
        self.assertEqual((destination / 'models/aliked_lightglue/LICENSE.aliked').read_text(), 'BSD fixture license\n')
        self.assertEqual(manifest['source']['export']['source']['commit'], 'pinned-test-commit')
        self.assertFalse(list(destination.parent.glob('.partial-*')))
        run = self.only_run()
        self.assertEqual(json.loads((run / 'result.json').read_text())['status'], 'complete')
        self.assertIn('mock true export completed', (run / 'job.log').read_text())

    def test_failed_export_keeps_diagnostics_and_never_promotes_a_completed_bundle(self):
        def failed(output, source, width, height):
            output.write_bytes(b'partial export')
            print('model exporter diagnostic')
            raise RuntimeError('fixture export failure')

        with patch.object(cli.platform, 'system', return_value='Linux'), \
             patch.object(cli.platform, 'machine', return_value='x86_64'), \
             patch.object(export_model, 'export_aliked', side_effect=failed):
            status, stdout, stderr = self.run_cli(self.cli_args('export', '--host', 'pc', '--source-dir', str(self.share)))
        self.assertEqual(status, 1, stdout + stderr)
        self.assertFalse(bundle.export_path(self.results, 'asset1', 'export1').exists())
        run = self.only_run()
        self.assertEqual(json.loads((run / 'result.json').read_text())['status'], 'failed')
        self.assertIn('model exporter diagnostic', (run / 'job.log').read_text())
        self.assertIn('fixture export failure', (run / 'job.log').read_text())
        self.assertEqual((run / 'failed-export/models/aliked_lightglue/aliked.onnx').read_bytes(), b'partial export')

    def test_package_export_copies_portable_model_without_calling_checkpoint_exporter(self):
        with patch.object(cli.platform, 'system', return_value='Linux'), \
             patch.object(cli.platform, 'machine', return_value='x86_64'), \
             patch.object(export_model, 'export_aliked') as converted, \
             patch.object(cli, 'onnx_contract', return_value=deepcopy(MODEL)):
            status, stdout, stderr = self.run_cli(self.cli_args('export', '--host', 'pc', '--mode', 'package', '--source-dir', str(self.share)))
        self.assertEqual(status, 0, stdout + stderr)
        converted.assert_not_called()
        destination = bundle.export_path(self.results, 'asset1', 'export1')
        self.assertEqual(bundle.validate_bundle(destination)['model']['mode'], 'package')
        self.assertEqual((destination / 'models/aliked_lightglue/aliked.onnx').read_bytes(),
                         (self.share / 'models/aliked_lightglue/aliked.onnx').read_bytes())

    def test_cli_dry_run_does_not_create_data_or_call_export(self):
        missing = self.root / 'not-created'
        with patch.object(cli, 'prepare_export') as prepare:
            status, stdout, stderr = self.run_cli([
                'export', '--result-dir', str(missing), '--asset-id', 'asset1', '--export-id', 'export1', '--dry-run'])
        self.assertEqual(status, 0, stdout + stderr)
        self.assertFalse(missing.exists())
        prepare.assert_not_called()

    def test_sigterm_export_records_failed_job_and_preserves_partial_diagnostics(self):
        # signal は独立した CPU subprocess のみに送り、test runner を巻き込まない。
        code = '''
import os
import signal
import sys
from tools.vgl import cli, export_model
cli.platform.system = lambda: 'Linux'
cli.platform.machine = lambda: 'x86_64'
def interrupted(output, source, width, height):
    print('SIGTERM fixture exporter', flush=True)
    os.kill(os.getpid(), signal.SIGTERM)
export_model.export_aliked = interrupted
raise SystemExit(cli.main(sys.argv[1:]))
'''
        result = subprocess.run(
            [sys.executable, '-c', code, *self.cli_args('export', '--host', 'pc', '--source-dir', str(self.share))],
            cwd=PROJECT, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 130, result.stdout + result.stderr)
        self.assertFalse(bundle.export_path(self.results, 'asset1', 'export1').exists())
        run = self.only_run()
        self.assertEqual(json.loads((run / 'result.json').read_text())['status'], 'failed')
        self.assertIn('SIGTERM fixture exporter', (run / 'job.log').read_text())
        self.assertTrue((run / 'failed-export').is_dir())

    def builder_stack(self, *, incomplete=False, changed_hardware=None):
        stack = ExitStack()
        stack.enter_context(patch.object(build_engine, 'hardware_info', return_value=deepcopy(changed_hardware or self.hardware)))
        stack.enter_context(patch.object(build_engine, 'package_paths', return_value=(self.prefix, self.share)))

        def inspect(path, expected_image_shape=None):
            return {'sha256': bundle.sha256(path), 'size': path.stat().st_size,
                    'tensors': [{'name': 'image', 'shape': expected_image_shape}] if expected_image_shape else []}

        stack.enter_context(patch.object(build_engine, 'inspect_engine', side_effect=inspect))

        def run(command):
            output = Path(command[command.index('--output_model_dir') + 1])
            models = Path(command[command.index('--model_dir') + 1])
            source = bundle.export_path(self.results, 'asset1', 'export1')
            model = models / 'aliked_lightglue/aliked.onnx'
            config = Path(command[2])
            self.assertEqual(model.read_bytes(), (source / 'models/aliked_lightglue/aliked.onnx').read_bytes())
            self.assertFalse(model.samefile(source / 'models/aliked_lightglue/aliked.onnx'))
            self.assertEqual(config.read_bytes(), (source / 'configs/isaac' / config.name).read_bytes())
            target = output / 'aliked_lightglue'
            target.mkdir(parents=True, exist_ok=True)
            name = Path(command[0]).name
            print('fixture builder: ' + name)
            if name == 'export_extractor_engine':
                engine = 'aliked_fixture.engine'
            else:
                engine = 'lightglue_aliked_fixture.engine'
            # The SDK writes a cache next to its input ONNX before exporting the engine.
            cached = models / 'aliked_lightglue' / engine
            cached.write_bytes(engine.encode())
            if not incomplete or name == 'export_extractor_engine':
                shutil.copyfile(cached, target / engine)

        called = stack.enter_context(patch.object(build_engine, 'run_builder', side_effect=run))
        return stack, called

    def build_fixture(self, *, incomplete=False):
        source = self.make_bundle()
        logs = self.root / 'build-log'
        logs.mkdir()
        stack, called = self.builder_stack(incomplete=incomplete)
        with stack, redirect_stdout(io.StringIO()):
            output = build_engine.build(self.results, 'asset1', 'export1', 'jetson', 'build1', logs)
        return source, output, logs, called

    def test_build_uses_official_commands_and_reuses_only_verified_matching_cache(self):
        source, output, logs, called = self.build_fixture()
        bundle.validate_bundle(source)
        self.assertEqual(output, self.results / 'models/vgl/asset1/engines/jetson/build1')
        commands = [call.args[0] for call in called.call_args_list]
        self.assertEqual(len(commands), 2)
        for command, name, flag, config in zip(commands,
                ('export_extractor_engine', 'export_lightglue_engine'),
                ('--configure_file', '--worker_config_file'),
                ('keypoint_creation_config.pb.txt', 'matching_task_worker_config.pb.txt')):
            self.assertEqual(command[:2], [str(self.prefix / 'bin/visual_mapping' / name), flag])
            self.assertEqual(Path(command[2]).name, config)
            self.assertEqual(command[3], '--model_dir')
            self.assertFalse(Path(command[4]).exists())
            self.assertEqual(command[5], '--output_model_dir')
        self.assertEqual(commands[0][-1], commands[1][-1])
        self.assertEqual(len(list(output.rglob('*.engine'))), 2)
        self.assertEqual(list(output.rglob('*.onnx')), [output / 'aliked_lightglue/aliked.onnx'])
        report = json.loads((output / 'build.json').read_text())
        self.assertEqual(report['status'], 'complete')
        self.assertEqual(report['bundle_sha256'], bundle.sha256(source / 'manifest.json'))
        self.assertEqual(report['hardware'], self.hardware)
        stack, again = self.builder_stack()
        with stack, redirect_stdout(io.StringIO()):
            reused = build_engine.build(self.results, 'asset1', 'export1', 'jetson', 'build1', logs)
        self.assertEqual(reused, output)
        again.assert_not_called()

    def test_changed_gpu_or_bundle_manifest_refuses_cache_without_executing_builder(self):
        source, output, logs, _ = self.build_fixture()
        stack, called = self.builder_stack(changed_hardware={**self.hardware, 'gpu_uuid': 'different-gpu'})
        with stack, self.assertRaisesRegex(ValueError, '一致しません'):
            build_engine.build(self.results, 'asset1', 'export1', 'jetson', 'build1', logs)
        called.assert_not_called()
        self.change_manifest(source, lambda item: item['source'].update(note='changed provenance'))
        stack, called = self.builder_stack()
        with stack, self.assertRaisesRegex(ValueError, '一致しません'):
            build_engine.build(self.results, 'asset1', 'export1', 'jetson', 'build1', logs)
        called.assert_not_called()
        self.assertEqual(json.loads((output / 'build.json').read_text())['status'], 'complete')

    def test_gpu_preflight_failure_happens_before_builder_or_output_creation(self):
        self.make_bundle()
        logs = self.root / 'logs'
        logs.mkdir()
        with patch.object(build_engine, 'hardware_info', side_effect=ValueError('no CUDA device')), \
             patch.object(build_engine, 'run_builder') as run, patch.object(build_engine, 'package_paths') as paths:
            with self.assertRaisesRegex(ValueError, 'CUDA'):
                build_engine.build(self.results, 'asset1', 'export1', 'jetson', 'build1', logs)
        run.assert_not_called()
        paths.assert_not_called()
        self.assertFalse((self.results / 'models/vgl/asset1/engines').exists())

    def test_incomplete_engine_output_is_not_promoted_and_failure_logs_keep_artifacts(self):
        source = self.make_bundle()
        original = bundle.validate_bundle(source)
        stack, called = self.builder_stack(incomplete=True)
        with stack:
            status, stdout, stderr = self.run_cli(self.cli_args(
                'build', '--host', 'jetson', '--target-id', 'jetson', '--build-id', 'build1'))
        self.assertEqual(status, 1, stdout + stderr)
        self.assertEqual(called.call_count, 2)
        self.assertFalse((self.results / 'models/vgl/asset1/engines/jetson/build1').exists())
        logs = self.only_run()
        self.assertEqual(json.loads((logs / 'build-failed.json').read_text())['status'], 'failed')
        self.assertEqual(json.loads((logs / 'result.json').read_text())['status'], 'failed')
        self.assertTrue((logs / 'build-plan.json').is_file())
        self.assertTrue((logs / 'failed-output/aliked_lightglue/aliked_fixture.engine').is_file())
        self.assertEqual(bundle.validate_bundle(source), original)
        self.assertFalse(list((logs / 'failed-output').rglob('*.onnx')))
        for call in called.call_args_list:
            command = call.args[0]
            self.assertFalse(Path(command[command.index('--model_dir') + 1]).exists())
        self.assertIn('fixture builder: export_lightglue_engine', (logs / 'job.log').read_text())

    def test_changed_or_symlinked_engine_and_model_are_rejected_during_verify(self):
        source, output, logs, _ = self.build_fixture()
        engine = output / 'aliked_lightglue/aliked_fixture.engine'
        original = engine.read_bytes()
        engine.write_bytes(b'tampered engine')
        stack, _ = self.builder_stack()
        with stack, self.assertRaisesRegex(ValueError, '変更'):
            build_engine.verify_build(output, source, self.hardware)
        engine.write_bytes(original)
        outside = self.root / 'outside-engine'
        outside.write_bytes(original)
        engine.unlink()
        engine.symlink_to(outside)
        stack, _ = self.builder_stack()
        with stack, self.assertRaises(ValueError):
            build_engine.verify_build(output, source, self.hardware)
        engine.unlink()
        engine.write_bytes(original)
        onnx = output / 'aliked_lightglue/aliked.onnx'
        onnx.unlink()
        onnx.symlink_to(source / 'models/aliked_lightglue/aliked.onnx')
        stack, _ = self.builder_stack()
        with stack, self.assertRaises(ValueError):
            build_engine.verify_build(output, source, self.hardware)

    def test_builder_process_output_is_visible_to_job_log_capture(self):
        logs = io.StringIO()
        with redirect_stdout(logs):
            build_engine.run_builder([sys.executable, '-c',
                                      'import sys; print("builder stdout"); print("builder stderr",file=sys.stderr)'])
        self.assertIn('builder stdout', logs.getvalue())
        self.assertIn('builder stderr', logs.getvalue())

    def test_builder_interrupt_terminates_only_the_created_process_group(self):
        process = NS(stdout=io.StringIO('builder running\n'), pid=123456,
                     poll=lambda: None, wait=unittest.mock.Mock(return_value=-15))
        with patch.object(build_engine.subprocess, 'Popen', return_value=process) as opened, \
             patch.object(build_engine, 'print', side_effect=KeyboardInterrupt, create=True), \
             patch.object(build_engine.os, 'killpg') as stopped:
            with self.assertRaises(KeyboardInterrupt):
                build_engine.run_builder(['fixture-builder', '--input', 'argument with spaces'])
        self.assertTrue(opened.call_args.kwargs['start_new_session'])
        self.assertEqual(opened.call_args.args[0], ['fixture-builder', '--input', 'argument with spaces'])
        stopped.assert_called_once_with(process.pid, signal.SIGTERM)
        process.wait.assert_called_once_with(timeout=10)


if __name__ == '__main__':
    unittest.main()
