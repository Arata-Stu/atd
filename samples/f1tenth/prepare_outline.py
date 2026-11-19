"""F1TENTH の中心線を、小型 RC 用の閉路の設計中心に変換する（raceline 環境）。"""
import argparse
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize


def curvature_limited_seed(points, spacing):
    """積分して初期形状を作る。急な折り返しでも辺がゼロ長に潰れないようにする。"""
    edges = np.roll(points, -1, axis=0) - points
    count = len(points)
    angle = np.unwrap(np.tile(np.arctan2(edges[:, 1], edges[:, 0]), 3))
    lower, upper = angle.copy(), angle.copy()
    max_turn = spacing * .73
    for i in range(1, len(angle)):
        lower[i] = min(lower[i], lower[i - 1] + max_turn)
        upper[i] = max(upper[i], upper[i - 1] - max_turn)
    for i in reversed(range(len(angle) - 1)):
        lower[i] = min(lower[i], lower[i + 1] + max_turn)
        upper[i] = max(upper[i], upper[i + 1] - max_turn)
    heading = (lower[count:2*count] + upper[count:2*count]) / 2
    steps = np.column_stack((np.cos(heading), np.sin(heading))) * spacing
    steps -= steps.mean(axis=0)
    seed = np.vstack((np.zeros(2), np.cumsum(steps[:-1], axis=0)))
    return seed + (points.mean(axis=0) - seed.mean(axis=0))


def fit_shape(source, seed, spacing):
    """最小変位に、折れ角・辺長・隣接しないコース部分の接近に対する罰則を加える。"""
    count = len(source)
    first, second = np.triu_indices(count, 1)
    arc_distance = np.minimum(second - first, count - (second - first)) * spacing
    chord = np.linalg.norm(source[first] - source[second], axis=1)
    nearby = (arc_distance > 3.5) & (chord < 3.5)
    first, second = first[nearby], second[nearby]
    direction = source[first] - source[second]
    direction /= np.linalg.norm(direction, axis=1)[:, None]

    def objective(flat, curvature):
        points = flat.reshape((-1, 2))
        delta = np.roll(points, -1, axis=0) - points
        lengths = np.maximum(np.linalg.norm(delta, axis=1), 1e-5)
        heading = np.arctan2(delta[:, 1], delta[:, 0])
        turns = (heading - np.roll(heading, 1) + np.pi) % (2*np.pi) - np.pi
        excess_turn = np.maximum(0., np.abs(turns) - curvature * (lengths + np.roll(lengths, 1)) / 2)
        turn_force = 1200 * excess_turn
        angle_force = turn_force * np.sign(turns)
        normal = np.column_stack((-delta[:, 1], delta[:, 0]))
        edge_gradient = (angle_force - np.roll(angle_force, -1))[:, None] * normal / lengths[:, None]**2
        length_force = -curvature/2 * (turn_force + np.roll(turn_force, -1)) + 160*(lengths-spacing)
        edge_gradient += length_force[:, None] * delta / lengths[:, None]
        displacement = points - source
        gradient = np.roll(edge_gradient, 1, axis=0) - edge_gradient + .5 * displacement
        energy = (.25 * np.sum(displacement**2) + 600 * np.dot(excess_turn, excess_turn)
                  + 80 * np.sum((lengths-spacing)**2))

        separation = ((points[first] - points[second]) * direction).sum(axis=1)
        overlap = np.maximum(0., 1.18 - separation)
        push = (-30 * overlap)[:, None] * direction
        np.add.at(gradient, first, push)
        np.add.at(gradient, second, -push)
        return energy + 15*np.dot(overlap, overlap), gradient.ravel()

    # Tighten the turning penalty progressively. Publication is decided by the
    # geometric/vehicle tests of the actual ATD line generators in regenerate.py.
    solution = seed.ravel()
    for curvature in (1.8, 1.15, .78, .68):
        result = minimize(objective, solution, args=(curvature,), jac=True, method='L-BFGS-B',
                          options=dict(maxiter=500, ftol=1e-10, maxcor=8))
        if not np.all(np.isfinite(result.x)):
            raise ValueError('コース外形への近似計算が有限値を返しませんでした')
        solution = result.x
    return solution.reshape((-1, 2))


def outline(raw, source_seed=False):
    closed = np.vstack((raw, raw[0]))
    lengths = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    # Leave space for the road width and corner rounding inside the default
    # 800-sample and one-million-cell region extraction budgets.
    scale = min(72 / lengths.sum(), (300 / np.prod(np.ptp(raw, axis=0)))**.5)
    closed *= scale
    stations = np.r_[0, np.cumsum(lengths * scale)]
    count = round(stations[-1] / .28)
    spacing = stations[-1] / count
    source = np.column_stack([np.interp(np.arange(count)*spacing, stations, closed[:, axis])
                              for axis in (0, 1)])
    # Shanghai の巻き込む複合コーナーは累積方位の初期化で交差する。
    # 入力の点順をそのまま初期値にして、同じ適合基準で解く。
    seed = source.copy() if source_seed else curvature_limited_seed(source, spacing)
    fitted = fit_shape(source, seed, spacing)
    spline = CubicSpline(np.arange(count + 1), np.vstack((fitted, fitted[0])), bc_type='periodic')
    return scale, source, spline(np.arange(0, count, .25))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--source-seed', action='store_true')
    args = parser.parse_args()
    scale, source, dense = outline(np.loadtxt(args.csv, delimiter=',')[:, :2], args.source_seed)
    np.savez(args.output, scale=scale, orig=source, dense=dense)
