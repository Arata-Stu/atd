from array import array
import math
import struct
from types import SimpleNamespace

from atd_ftg.planner import DepthProjection, GapPlanner
from atd_ftg.ros_input import depth_metres, projection


def test_depth_in_mm_with_big_endian_rows_and_padding():
    data = struct.pack(">HHxxHHxx", 500, 0, 2500, 1000)
    message = SimpleNamespace(encoding="16UC1", width=2, height=2, step=6, is_bigendian=True, data=data)
    assert list(depth_metres(message)) == [.5, 0., 2.5, 1.]


def test_rectified_projector_matches_simulated_mount_and_unknown_is_not_free():
    sim = DepthProjection.simulated(80, 40, 1.4, .18, .12, 0.)
    camera = projection(80, 40, (sim.fx, sim.fy, sim.cx, sim.cy), (.18, 0., .0875),
                        (-.5, .5, -.5, .5), .0325, .17)
    image = array("f", [2.]) * (80 * 40)
    for x, y, z in ((1., 0., .08), (1., .2, .15), (4., 0., .08)):
        assert camera.visible(image, x, y, z) == sim.visible(image, x, y, z)
    image = array("f", [math.inf]) * (80 * 40)
    assert sim.visible(image, 1., 0., .08)
    assert not camera.visible(image, 1., 0., .08)
    planner = GapPlanner(.257, .49, .5, .5, 1.8)
    assert planner.plan([], lambda x, y, z: camera.visible(image, x, y, z), 0., 0.)[1] == 0.
