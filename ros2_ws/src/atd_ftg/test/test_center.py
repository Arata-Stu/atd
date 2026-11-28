from array import array
import math

from atd_ftg.planner import CORRIDOR_WIDTH_HOLD_NS, DepthProjection, GapPlanner, RecentCorridorWidth


def corridor_depth(camera, lateral_offset):
    """平坦路面と高さ10 cm、幅1.4 mの左右境界を前方視野で撮影する。"""
    image = array("f")
    for v in range(camera.height):
        vertical = (camera.cy - v) / camera.fy
        floor = -.12 / vertical if vertical < 0. else math.inf
        for u in range(camera.width):
            lateral = (camera.cx - u) / camera.fx
            wall = (math.copysign(.7, lateral) - lateral_offset) / lateral if lateral else math.inf
            if math.isfinite(wall) and 0. <= .12 + wall * vertical <= .10:
                image.append(min(wall, floor))
            else:
                image.append(floor)
    return image


def test_center_steering_from_forward_only_depth():
    camera = DepthProjection.simulated(424, 240, math.radians(75.), .18, .12, 0.)
    planner = GapPlanner(.257, math.radians(28.), .5, .5, 1.8)
    for lateral_offset in (-.3, 0., .3):
        image = corridor_depth(camera, lateral_offset)
        angle, speed, report = planner.plan(
            camera.obstacles(image), lambda *xyz: camera.visible(image, *xyz), .3, 0.)
        assert speed > 0.
        assert abs(report["center_target_m"][1] + lateral_offset) < .04
        assert report["center_error_m"] < .04
        assert angle * lateral_offset < 0. if lateral_offset else angle == 0.


def test_one_wall_reuses_recent_width_and_expiry_keeps_wall_clearance():
    planner = GapPlanner(.257, math.radians(28.), .5, .5, 1.8)
    visible = lambda *xyz: True
    for side in (-1, 1):
        near = [(step * .025, side * .28, .075) for step in range(24, 70)]
        far = [(step * .025, -side * 1.12, .075) for step in range(24, 70)]
        _, _, both = planner.plan(near + far, visible, .2, 0.)
        width = RecentCorridorWidth()
        width.observe(10, both["observed_corridor_width_m"])
        angle, _, estimate = planner.plan(near, visible, .2, 0., recent_width=width.get(100))
        assert angle * side < 0.
        assert estimate["center_mode"] == "recent_width"
        assert abs(estimate["center_target_m"][1] + side * .42) < .03
        expired = width.get(11 + CORRIDOR_WIDTH_HOLD_NS)
        assert expired is None
        margin_angle, _, margin = planner.plan(near, visible, .2, 0., recent_width=expired)
        assert margin_angle * side < 0.
        assert margin["center_mode"] == "wall_margin"
        assert "center_target_m" not in margin


def test_center_moves_nearer_when_the_far_target_is_unreachable():
    planner = GapPlanner(.257, math.radians(28.), .5, .5, 1.8)
    left = [(step * .025, .4, .075) for step in range(24, 70)]
    right = [(step * .025, -1., .075) for step in range(24, 70)]
    _, _, near = planner.plan(left + right, lambda x, y, z: x < .98, .2, 0.)
    assert near["center_mode"] == "both_walls"
    assert near["center_target_m"][0] < .95
    _, speed, fallback = planner.plan(left + right, lambda x, y, z: x < .72, .2, 0.)
    assert speed > 0.
    assert fallback["center_mode"] == "gap_only"
