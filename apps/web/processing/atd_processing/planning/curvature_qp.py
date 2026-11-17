"""TPH closed-track QP adapter: differentiate tangent and curvature denominator.

The pinned TPH spline matrix, normal offsets, quadprog solver, width/curvature
inequalities and the upstream IQP loop are retained. The upstream frozen-tangent
curvature approximation is replaced with the full first derivative of curvature.
Reference arc-length weights make the discrete cost an approximation to ∫κ² ds.
This remains a local QP approximation, not a global optimum or a lap-time solver.

Primary source/interface:
https://github.com/TUMFTM/trajectory_planning_helpers/blob/master/trajectory_planning_helpers/opt_min_curv.py
"""
from __future__ import annotations

ADAPTER_ID = 'tph_full_curvature_jacobian_v1'


def linearization(reftrack, normvectors, A):
    import numpy as np
    count = len(reftrack)
    if A.shape != (4 * count, 4 * count) or normvectors.shape != (count, 2):
        raise ValueError('閉路 spline / normal の行列形状が一致しません')
    index = np.arange(count)
    following = (index + 1) % count
    rhs = np.zeros((4 * count, 2 + 2 * count))
    rhs[4 * index, :2] = reftrack[:, :2]
    rhs[4 * index + 1, :2] = reftrack[following, :2]
    for axis in range(2):
        columns = 2 + axis * count
        rhs[4 * index, columns + index] = normvectors[:, axis]
        rhs[4 * index + 1, columns + following] = normvectors[following, axis]
    coefficients = np.linalg.solve(A, rhs)
    first, second = coefficients[1::4], 2 * coefficients[2::4]
    x1, y1, x2, y2 = first[:, 0], first[:, 1], second[:, 0], second[:, 1]
    dx1, dy1 = first[:, 2:2 + count], first[:, 2 + count:]
    dx2, dy2 = second[:, 2:2 + count], second[:, 2 + count:]
    speed2 = x1*x1 + y1*y1
    if np.any(speed2 <= 1e-14):
        raise ValueError('spline の接線長が0です')
    numerator = x1*y2 - y1*x2
    curvature = numerator / speed2**1.5
    # All four spline derivatives depend on the lateral offsets. In particular,
    # omitting d(speed^-3) makes a circle's radial curvature gradient incorrect.
    derivative_numerator = (y2[:, None]*dx1 + x1[:, None]*dy2
                            - x2[:, None]*dy1 - y1[:, None]*dx2)
    jacobian = derivative_numerator / speed2[:, None]**1.5
    jacobian -= (3*numerator / speed2**2.5)[:, None] * (x1[:, None]*dx1 + y1[:, None]*dy1)

    def evaluate(alpha):
        u, v = x1 + dx1 @ alpha, y1 + dy1 @ alpha
        a, b = x2 + dx2 @ alpha, y2 + dy2 @ alpha
        denominator = (u*u + v*v)**1.5
        if np.any(denominator <= 1e-14):
            raise ValueError('最適化後の spline 接線長が0です')
        return (u*b - v*a) / denominator
    return curvature, jacobian, evaluate


def opt_min_curv(reftrack, normvectors, A, kappa_bound, w_veh,
                 print_debug=False, plot_debug=False, closed=True, **unused):
    import numpy as np
    import quadprog
    if not closed:
        raise ValueError('ATD の mincurv adapter は閉路専用です')
    kappa, jacobian, evaluate = linearization(reftrack, normvectors, A)
    distance = np.linalg.norm(np.roll(reftrack[:, :2], -1, axis=0) - reftrack[:, :2], axis=1)
    weights = (distance + np.roll(distance, 1)) / 2
    hessian = 2 * jacobian.T @ (weights[:, None] * jacobian)
    hessian = (hessian + hessian.T) / 2
    # Rigid-motion/numerical null modes must not make quadprog reject a PSD cost.
    ridge = max(float(np.max(np.abs(np.diag(hessian)))) * 1e-12, 1e-12)
    hessian += np.eye(len(kappa)) * ridge
    gradient = 2 * jacobian.T @ (weights * kappa)
    right, left = reftrack[:, 2] - w_veh/2, reftrack[:, 3] - w_veh/2
    if np.any(-right > left):
        raise RuntimeError('track might be too small for the requested safety distance')
    # Same TPH alpha sign and width constraints: -left <= alpha <= right.
    constraint = np.vstack((np.eye(len(kappa)), -np.eye(len(kappa)), jacobian, -jacobian))
    bounds = np.concatenate((right, left, kappa_bound-kappa, kappa_bound+kappa))
    alpha = quadprog.solve_qp(hessian, -gradient, -constraint.T, -bounds, 0)[0]
    error = float(np.max(np.abs(evaluate(alpha) - (kappa + jacobian @ alpha))))
    return alpha, error
