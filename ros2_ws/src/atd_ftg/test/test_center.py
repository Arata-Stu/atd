from array import array
import math

from atd_ftg.planner import DepthProjection, GapPlanner


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


def test_one_wall_or_unreachable_center_preserves_ftg_fallback():
    planner = GapPlanner(.257, math.radians(28.), .5, .5, 1.8)
    left = [(step * .025, .4, .075) for step in range(24, 70)]
    right = [(step * .025, -1., .075) for step in range(24, 70)]
    visible_prefix = lambda x, y, z: x < .85
    assert planner.corridor_target(left + right) is not None
    for obstacles in (left, left + right):
        angle, speed, report = planner.plan(obstacles, visible_prefix, .2, 0.)
        assert speed > 0.
        assert abs(angle) <= planner.steering_limit
        assert "center_target_m" not in report
