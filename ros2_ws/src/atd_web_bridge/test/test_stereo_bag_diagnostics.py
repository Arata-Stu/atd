import struct

from atd_web_bridge.stereo_bag_diagnostics import image_stamp, pair_stamps, summarize


def test_image_stamp_reads_cdr_header_without_decoding_pixels():
    for encoding, endian in [(0, '>'), (1, '<')]:
        header = bytes([0, encoding, 0, 0]) + struct.pack(endian + 'iI', 4, 10)
        assert image_stamp(header) == 4_000_000_010
    assert image_stamp(b'') is None
    assert image_stamp(bytes(12)) is None


def test_gap_and_stereo_pair_estimates():
    ms = 1_000_000
    left = [10 * ms, 30 * ms, 30 * ms, 20 * ms, 50 * ms]
    ordered = sorted(set(left))
    summary = summarize(left, ordered, 10 * ms, 10 * ms)
    assert summary['estimated_missing'] == 1
    assert summary['duplicates'] == summary['backwards'] == 1
    assert summary['gaps'] == [dict(after_s=.02, interval_ms=20, estimated_missing=1)]
    assert summarize([], [], 0, 10 * ms)['estimated_missing'] is None
    assert pair_stamps(ordered, [11 * ms, 20 * ms, 40 * ms, 50 * ms], 2.5 * ms) == dict(
        paired=3, left_only=1, right_only=1, tolerance_ms=2.5)
