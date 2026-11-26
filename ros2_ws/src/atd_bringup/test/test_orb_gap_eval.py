"""収録元と地図を照合し、同じ run で繰り返し評価できることを確認する。"""
import hashlib
import json
from unittest.mock import patch

import pytest

from atd_bringup import orb_gap_eval


@pytest.fixture
def run(tmp_path):
    bag = tmp_path / 'moved-bag'
    folder = tmp_path / 'runs/orbslam3/first'
    replay = tmp_path / 'runs/replay/paired'
    for directory in (folder, replay, bag):
        directory.mkdir(parents=True)
    metadata = b'fixture metadata'
    (bag / 'metadata.yaml').write_bytes(metadata)
    common = dict(backend='orbslam3', mode='localization', bag='/data/record/rosbags/original',
                  map_id='odaiba', rate=1., start_offset_s=0., status='complete')
    (folder / 'summary.json').write_text(json.dumps(dict(
        common, gt_replay_id='paired', metadata_sha256=hashlib.sha256(metadata).hexdigest(), trajectory_samples=3)))
    (replay / 'summary.json').write_text(json.dumps(dict(common, id='paired', alignment_source='saved_map')))
    stamps = [10_000_000, 110_000_000, 410_000_000]
    (folder / 'trajectory.csv').write_text('image_stamp_ns\n' + ''.join(f'{s}\n' for s in stamps))
    (folder / 'status.csv').write_text('stamp_ns,state,epoch\n' + ''.join(
        f'{s},tracking,{int(s == stamps[-1])}\n' for s in stamps))
    with (replay / 'samples.jsonl').open('w') as file:
        for stamp in stamps:
            sample = dict(stamp_ns=str(stamp), est_x_m=stamp/1e9, gt_x_m=stamp/1e9,
                          est_y_m=0., gt_y_m=0., est_yaw_rad=0., gt_yaw_rad=0.)
            file.write(json.dumps(sample) + '\n')
    return bag, folder, replay


def test_entry_writes_paired_comparison_and_preserves_report_on_failed_retry(tmp_path, run, capsys):
    bag, folder, _ = run
    gyro_stamps = list(range(0, 420_000_000, 10_000_000))
    args = ['first', '--bag', str(bag), '--results-root', str(tmp_path)]
    with patch.object(orb_gap_eval, 'read_bag_gyro', return_value=(gyro_stamps, [0.]*len(gyro_stamps), {})) as reader:
        assert orb_gap_eval.main(args) == 0
        assert '最長の途絶 0.300 秒' in capsys.readouterr().out
        reader.assert_called_once_with(bag)
        report = folder / 'imu_gap_report.json'
        original = report.read_bytes()
        result = json.loads(original)
        assert result['paired_count'] == 1
        assert result['paired_metrics']['gyro']['median_position_error_m'] == pytest.approx(0.)
        (bag / 'metadata.yaml').write_bytes(b'different bag')
        assert orb_gap_eval.main(args) == 1
        assert '収録元と一致しません' in capsys.readouterr().err
        assert report.read_bytes() == original
        assert reader.call_count == 1


def test_refuses_an_unrelated_gt_run(tmp_path, run):
    bag, _, replay = run
    path = replay / 'summary.json'
    path.write_text(json.dumps(dict(json.loads(path.read_text()), map_id='other')))
    with pytest.raises(ValueError, match='一致しません'):
        orb_gap_eval.evaluate_run(tmp_path, 'first', bag)
