# Ported from kart/tools/app/kart_studio/optimizer_worker.py; paired/local fallback is not used.
"""Adapter for the pinned trajectory_planning_helpers fork; no global optimizer checkout."""

import json
import sys
from pathlib import Path


HELPER_COMMIT = 'aa950f6045680366b789dbb855db8d59d54b1db5'
NUMERICAL_PINS = {'trajectory-planning-helpers': '0.80', 'numpy': '2.4.6',
                  'scipy': '1.17.1', 'quadprog': '0.1.13', 'matplotlib': '3.11.0'}


def pinned_installation():
    from importlib.metadata import distribution, PackageNotFoundError
    versions = {}
    for name, expected in NUMERICAL_PINS.items():
        try:
            package = distribution(name)
        except PackageNotFoundError as error:
            raise ValueError('専用 Raceline 環境に依存がありません: ' + name + '。atd-python raceline sync を実行してください') from error
        versions[name] = package.version
        if package.version != expected:
            raise ValueError(f'専用 Raceline 環境の版が不一致です: {name}={package.version} / 必須{expected}。別方式へ fallback しません')
    package = distribution('trajectory-planning-helpers')
    source = json.loads(package.read_text('direct_url.json') or '{}')
    if (source.get('vcs_info', {}).get('commit_id') != HELPER_COMMIT
            or source.get('url', '').removesuffix('.git').rstrip('/') != 'https://github.com/carbuncle01/trajectory_planning_helpers'):
        raise ValueError('trajectory-planning-helpers の固定 fork / commit と一致しません。atd-python raceline sync で専用環境を同期してください')
    return package, source, versions


def _noncrossing_output(create, validate=None, *, refine_on_failure=False):
    """Backtrack against all supplied checks, preserving point order.

    For refinement only, the caller may request a failed candidate back. Prefer
    a geometry-valid curvature failure, otherwise the original width failure.
    The caller must validate again before publishing anything.
    """
    from .geometry import PathValidationError, simple

    failures = []
    original = None
    best_curvature_failure = None
    for exponent in range(9):
        scale = 0.5 ** exponent
        line = create(scale)
        if exponent == 0:
            original = line
        try:
            simple(line, True)
            if validate is not None:
                validate(line)
        except PathValidationError as exc:
            if exc.diagnostic["reason"] == "curvature":
                value = exc.diagnostic["curvature_abs"]
                if best_curvature_failure is None or value < best_curvature_failure[0]:
                    best_curvature_failure = value, line, scale
            exc.diagnostic.update(stage="raceline_optimized_output", lateral_step_scale=scale)
            exc.args = (f"Raceline最適化後の検査: {exc}",)
            failures.append(dict(scale=scale, diagnostic=exc.diagnostic))
            last_error = exc
            continue
        return line, scale, failures
    if refine_on_failure:
        if best_curvature_failure is not None:
            _, line, scale = best_curvature_failure
            return line, scale, failures
        return original, 1.0, failures
    raise last_error


def _validate_candidate(line, corridor, p):
    from . import footprint
    from .lines import validate_raceline_curvature

    corridor.validate(line, p["vehicle_width"] / 2 + p["margin"])
    footprint.validate_line(line, True, p, corridor=corridor)
    validate_raceline_curvature(line, p)


def _refine_curvature(alpha, args, solve_qp, interpolate):
    """Relinearize curvature on the same spline knots and fixed lateral axes."""
    from .lines import curvature

    def evaluate(offset):
        line = interpolate(offset).tolist()
        peak = max(abs(curvature(line[i - 1], point, line[(i + 1) % len(line)]))
                   for i, point in enumerate(line))
        return peak, line

    peak, line = evaluate(alpha)
    history = [peak]
    # Check the interpolated output: satisfying the linearized knot constraints
    # can still exceed the bound in a tight bend. Keep only improving steps.
    for _ in range(8):
        if peak <= args["kappa_bound"]:
            break
        track = args["reftrack"].copy()
        track[:, :2] += alpha[:, None] * args["normvectors"]
        track[:, 2] -= alpha  # remaining right and left offsets in the original corridor
        track[:, 3] += alpha
        delta, _ = solve_qp(**dict(args, reftrack=track))
        best = None
        for scale in (1., .5, .25, .125):
            offset = alpha + scale * delta
            candidate_peak, candidate = evaluate(offset)
            if candidate_peak < peak and (best is None or candidate_peak < best[0]):
                best = candidate_peak, candidate, offset
        if best is None:
            break
        peak, line, alpha = best
        history.append(peak)
    return line, history


def _validated_solution(solve, corridor, p):
    """Tighten QP bounds only after measuring a rejected interpolated result."""
    from .geometry import PathValidationError, distance

    clearance = p["vehicle_width"] / 2 + p["margin"]
    reserve = 0.0
    last_error = None
    attempts = []
    refine_curvature = False
    while len(attempts) < 8:
        try:
            line = solve(reserve, refine_curvature)
        except (ValueError, RuntimeError) as exc:
            if last_error is not None and (
                "constraints are inconsistent" in str(exc) or "track might be too small" in str(exc)
            ):
                # Preserve the concrete failing location if tighter bounds have
                # no QP solution. Never return the preceding invalid candidate.
                raise last_error from exc
            raise
        try:
            _validate_candidate(line, corridor, p)
        except PathValidationError as exc:
            last_error = exc
            detail = exc.diagnostic
            detail["stage"] = "raceline_optimized_output"
            if detail["reason"] == "curvature":
                if p["optimizer"] == "mincurv" and not refine_curvature:
                    refine_curvature = True
                    continue
                raise
            attempts.append(dict(reserve_m=reserve, diagnostic=detail))
            gap = detail.get("distance_m", clearance)
            missing = clearance - gap
            if detail["reason"] == "outside":
                missing = clearance + gap
            for point in detail.get("footprint", []):
                if not corridor.contains(point):
                    missing = max(missing, min(distance(point, a, b) for a, b in corridor.boundaries))
            # A 1 mm numerical buffer avoids repeatedly touching an active
            # bound. Repeated violations increase it, within a bounded retry.
            reserve = max(reserve * 2, reserve + max(0, missing) + 0.001)
            continue
        return line, reserve, attempts
    raise last_error


def run(document, p):
    try:
        return _run(document, p)
    except (ValueError, RuntimeError) as exc:
        if "constraints are inconsistent" not in str(exc) and "track might be too small" not in str(exc):
            raise
        raise ValueError(
            "Racelineの最適化制約を同時に満たせませんでした。"
            f"方式={p['optimizer']}、曲率上限={p['curvature_limit']:.3g} 1/m"
            f"（最小旋回半径={1 / p['curvature_limit']:.3g} m）、"
            f"車幅={p['vehicle_width']:.3g} m、片側余裕={p['margin']:.3g} m。"
            "境界の急な折れ・狭い区間・左右境界の対応を確認してください。"
            "曲率上限や点間隔の見直しで改善する場合もありますが、"
            "この失敗だけでは実際の走行経路が存在しないとは断定できません。"
            "依存の読み込みには成功しています。"
        ) from exc


def _run(document, p):
    package, package_source, numerical_versions = pinned_installation()

    import numpy as np
    import trajectory_planning_helpers as tph
    from scipy.interpolate import splev, splprep

    from .raceline_reference import build, validate
    from .curvature_qp import opt_min_curv as corrected_qp, ADAPTER_ID
    from .lines import curvature_metrics

    center, corridor, reference_info = build(document, p["spacing"])
    reference_metrics = curvature_metrics(document["lines"]["centerline"]["points"])
    ref = np.asarray(center)
    closed_ref = np.vstack((ref, ref[0]))
    spline, _ = splprep(closed_ref.T, s=len(ref) * 0.05**2, per=True)
    dense = np.asarray(splev(np.linspace(0, 1, max(1000, len(ref) * 10)), spline)).T
    stations = np.r_[0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
    target = np.linspace(0, stations[-1], len(ref), endpoint=False)
    ref = np.column_stack(
        [np.interp(target, stations, dense[:, axis]) for axis in range(2)]
    )
    validate(ref.tolist(), corridor, 0, reference_info, stage="raceline_smoothed_reference")
    cx, cy, matrix, normals = tph.calc_splines.calc_splines(
        path=np.vstack((ref, ref[0])), use_dist_scaling=False
    )

    # Measure actual available width along each spline normal, independent of
    # boundary labels/winding. Positive alpha follows the helper's right normal.
    def width(point, normal):
        distances = []
        for a, b in corridor.boundaries:
            segment = np.asarray(b) - a
            delta = np.asarray(a) - point
            det = normal[0] * segment[1] - normal[1] * segment[0]
            if abs(det) < 1e-10:
                continue
            t = (delta[0] * segment[1] - delta[1] * segment[0]) / det
            u = (delta[0] * normal[1] - delta[1] * normal[0]) / det
            if t > 0 and -1e-9 <= u <= 1 + 1e-9:
                distances.append(t)
        if not distances:
            raise ValueError("法線方向に境界が見つかりません")
        return min(distances)

    widths = np.asarray(
        [[width(pt, normal), width(pt, -normal)] for pt, normal in zip(ref, normals)]
    )
    # Start with exactly the requested allowance. Any correction is measured
    # on the interpolated output, rather than adding a fixed 5 cm everywhere.
    required_width = p["vehicle_width"] + 2 * p["margin"]
    total_widths = widths.sum(axis=1)
    narrowest = int(np.argmin(total_widths))
    if total_widths[narrowest] < required_width:
        raise ValueError(
            f"Racelineの幅制約を満たせません。参照点{narrowest}"
            f"（x={ref[narrowest, 0]:.3f}, y={ref[narrowest, 1]:.3f} m）の"
            f"法線方向幅={total_widths[narrowest]:.3f} m、必要幅={required_width:.3f} m。"
            "必要幅は車幅＋両側余裕です。"
            "境界と車両寸法・余裕を確認してください。"
        )
    # A reference point may start inside the requested clearance: the QP can
    # move it to the other side. Only the optimized result must meet it.
    track = np.column_stack((ref, widths))
    if p["optimizer"] != "mincurv":
        lengths = tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=cx, coeffs_y=cy)
        psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(
            coeffs_x=cx,
            coeffs_y=cy,
            ind_spls=np.arange(len(ref)),
            t_spls=np.zeros(len(ref)),
        )
        dkappa = (np.roll(kappa, -1) - np.roll(kappa, 1)) / (
            lengths + np.roll(lengths, 1)
        )

    step_scale = 1.0
    topology_refinements = []
    iqp_history = []
    step_refinements = []
    curvature_refinements = []

    def solve(reserve, refine_curvature):
        nonlocal step_scale
        # IQP mutates its input widths. Every retry starts from the same track.
        current_track, current_normals = track.copy(), normals.copy()
        args = dict(
            reftrack=current_track,
            normvectors=current_normals,
            A=matrix,
            kappa_bound=p["curvature_limit"],
            w_veh=required_width + 2 * reserve,
            print_debug=False,
            plot_debug=False,
        )
        if p["optimizer"] == "mincurv":
            alpha = corrected_qp(**args)[0]
        else:
            from .iqp import monitored_iqp

            upstream_qp = tph.opt_min_curv.opt_min_curv
            tph.opt_min_curv.opt_min_curv = corrected_qp
            try:
                optimized = monitored_iqp(tph, dict(
                    **args, spline_len=lengths, psi=psi, kappa=kappa, dkappa=dkappa,
                    stepsize_interp=p["spacing"], iters_min=3, curv_error_allowed=0.01,
                ), history=iqp_history)
            finally:
                tph.opt_min_curv.opt_min_curv = upstream_qp
            alpha, current_track, current_normals = optimized[:3]

        def interpolate(offset):
            result = tph.create_raceline.create_raceline(
                refline=current_track[:, :2],
                normvectors=current_normals,
                alpha=offset,
                stepsize_interp=min(p["spacing"], 0.05),
            )[0]
            if not np.isfinite(result).all():
                raise ValueError("最適化結果に非有限値があります")
            return result

        if refine_curvature:
            result, peaks = _refine_curvature(alpha, args, corrected_qp, interpolate)
            curvature_refinements.append(dict(reserve_m=reserve, peak_curvatures_1_per_m=peaks))
            step_scale = 1.
            return result

        def create(scale):
            result = interpolate(alpha * scale if p["optimizer"] == "mincurv" else alpha)
            if p['optimizer'] == 'mincurv_iqp':
                # IQP's current_track already includes all preceding moves. A
                # final-alpha-only scale cannot undo an invalid cumulative move.
                # Blend the whole cumulative route from the saved Centerline,
                # preserving cyclic point order, then run every original check.
                parameter = np.arange(len(result)) * len(ref) / len(result)
                indexes = np.floor(parameter).astype(int)
                t = parameter - indexes
                basis = np.column_stack((np.ones(len(t)), t, t*t, t*t*t))
                baseline = np.column_stack((np.sum(cx[indexes]*basis, axis=1),
                                             np.sum(cy[indexes]*basis, axis=1)))
                result = baseline + scale * (result - baseline)
            return result.tolist()

        result, step_scale, failures = _noncrossing_output(
            create, lambda line: _validate_candidate(line, corridor, p), refine_on_failure=True
        )
        step_refinements.extend(dict(reserve_m=reserve, **failure) for failure in failures)
        topology_refinements.extend(dict(reserve_m=reserve, **failure) for failure in failures
                                    if failure["diagnostic"]["reason"] == "self_intersection")
        return result

    result, reserve, attempts = _validated_solution(solve, corridor, p)
    output_metrics = curvature_metrics(result)
    reference_energy = reference_metrics['squared_curvature_integral_1_per_m']
    if output_metrics['squared_curvature_integral_1_per_m'] > reference_energy * 1.0001 + 1e-8:
        error = ValueError('最適化候補の実際の曲率二乗の弧長積分が保存済み Centerline より増加しました。候補を公開しません')
        error.diagnostic = dict(stage='raceline_actual_objective', reason='objective_increase',
                                reference=reference_metrics, output=output_metrics)
        raise error
    return dict(
        points=result,
        optimizer_info=dict(
            package="trajectory-planning-helpers",
            version=package.version,
            source=package_source,
            numerical_versions=numerical_versions,
            helper_commit=HELPER_COMMIT,
            curvature_linearization=ADAPTER_ID,
            objective="sum_squared_linearized_curvature_reference_arc_weights",
            objective_units="1/m",
            reference_curvature_metrics=reference_metrics,
            smoothed_reference_curvature_metrics=curvature_metrics(ref.tolist()),
            output_curvature_metrics=output_metrics,
            actual_objective_verified=True,
            smoothing_rms_m=0.05,
            interpolation_reserve_m=reserve,
            clearance_refinements=attempts,
            curvature_refinements=curvature_refinements,
            reference_info=reference_info,
            lateral_step_scale=step_scale,
            step_backtracking="cumulative_from_initial_reference_spline" if p["optimizer"] == "mincurv_iqp" else "reference_normal_offsets",
            topology_refinements=topology_refinements,
            step_refinements=step_refinements,
            iqp_history=iqp_history,
            verification_spacing_m=min(p["spacing"], 0.05),
        ),
    )


if __name__ == "__main__":
    data = json.loads(Path(sys.argv[1]).read_text())
    try:
        result = run(data["document"], data["parameters"])
    except ValueError as exc:
        import traceback

        Path(sys.argv[2]).write_text(json.dumps(dict(
            error=str(exc), diagnostic=getattr(exc, "diagnostic", None)
        )))
        traceback.print_exc()
        sys.exit(1)
    Path(sys.argv[2]).write_text(json.dumps(result, allow_nan=False))
