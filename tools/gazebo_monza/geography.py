"""Distance and heading samples for a closed course in local metres."""
from bisect import bisect_right
from math import atan2, ceil, cos, dist, pi


def _stations(points):
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1] + dist(a, b))
    return distances


def _point_at(points, stations, s):
    s = min(max(s, stations[0]), stations[-1])
    i = min(bisect_right(stations, s) - 1, len(points) - 2)
    fraction = (s - stations[i]) / (stations[i + 1] - stations[i])
    a, b = points[i:i + 2]
    return (a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1]))


def closed_trajectory(midline, start):
    """Sample a closed drive path, smoothing through the timing line as well as every bend."""
    stations = _stations(midline)
    length = stations[-1]

    def smoothed(s):
        # Wrap the 0.4 m wide smoothing kernel through the lap seam.
        a, b, c = (_point_at(midline, stations, (s + ds) % length) for ds in (-.2, 0, .2))
        return ((a[0] + 2*b[0] + c[0]) / 4, (a[1] + 2*b[1] + c[1]) / 4)

    first = smoothed(0)
    # Preserve the position of the timing line from the course drawing.
    spacing = .10
    count = ceil(length / spacing)
    points = []
    for i in range(count):
        s = length * i / count
        x, y = smoothed(s)
        taper = .5 * (1 + cos(pi * min(1, min(s, length - s) / 3.0)))
        points.append((x + (start[0] - first[0])*taper, y + (start[1] - first[1])*taper))
    stations = _stations(points + points[:1])
    poses = []
    for i, (x, y) in enumerate(points):
        a, b = points[i - 1], points[(i + 1) % count]
        poses.append([stations[i], x, y, atan2(b[1] - a[1], b[0] - a[0])])
    poses.append([stations[-1], *poses[0][1:]])
    return poses
