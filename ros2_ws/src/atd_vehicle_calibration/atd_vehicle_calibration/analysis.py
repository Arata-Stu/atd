"""Pure, offline response analysis. No ROS, fitting, actuator mapping, or file I/O.

Motion is evaluated at the rear-axle/body origin by motion.estimate_motion.
Pico is held from the most recent *past* sample; IMU is nearest in the same
Pico continuity segment. Unknown/invalid values are never replaced with zero.
"""
from bisect import bisect_left, bisect_right
from collections import Counter, defaultdict
import math
from statistics import mean, median, pstdev


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _vector(value, size):
    return isinstance(value, (list, tuple)) and len(value) == size and all(_number(v) for v in value)


def _stats(values):
    values = sorted(v for v in values if _number(v))
    if not values:
        return dict(count=0, mean=None, median=None, std=None, min=None, max=None, p10=None, p90=None)

    def quantile(q):
        index = (len(values) - 1) * q
        lo = int(index)
        return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (index - lo)

    return dict(count=len(values), mean=mean(values), median=median(values), std=pstdev(values),
                min=values[0], max=values[-1], p10=quantile(0.1), p90=quantile(0.9))


class _PicoStream:
    """Interval lookup rejects overlapping ROS-clock epochs rather than sorting them together."""
    def __init__(self, samples, max_gap_s, max_sync_s):
        self.chunks = []
        self.rejected = Counter()
        chunk = None
        for sample in samples:
            stamp = sample.get('stamp')
            if not _number(stamp) or stamp <= 0:
                self.rejected['invalid_pico_stamp'] += 1
                chunk = None
                continue
            identity = (sample.get('segment'), sample.get('instance_id'), sample.get('serial'))
            if (chunk is None or chunk['identity'] != identity or stamp <= chunk['times'][-1]
                    or stamp - chunk['times'][-1] > max_gap_s):
                if self.chunks and stamp > self.chunks[-1]['times'][-1]:
                    self.chunks[-1]['next_start'] = stamp
                chunk = dict(identity=identity, times=[], samples=[], next_start=None)
                self.chunks.append(chunk)
            chunk['times'].append(float(stamp))
            chunk['samples'].append(sample)
        self.chunks.sort(key=lambda item: item['times'][0])
        self.starts = [item['times'][0] for item in self.chunks]
        self.prefix_end = []
        last = -math.inf
        for item in self.chunks:
            last = max(last, item['times'][-1] + max_sync_s)
            self.prefix_end.append(last)

    def at(self, stamp, max_sync_s):
        found = None
        epochs = 0
        index = bisect_right(self.starts, stamp) - 1
        while index >= 0 and self.prefix_end[index] >= stamp:
            chunk = self.chunks[index]
            if (chunk['times'][0] <= stamp <= chunk['times'][-1] + max_sync_s
                    and (chunk['next_start'] is None or stamp < chunk['next_start'])):
                epochs += 1
                if epochs > 1:
                    return None, 'ambiguous_pico_clock_epoch'
                position = bisect_right(chunk['times'], stamp) - 1
                age = stamp - chunk['times'][position]
                if age <= max_sync_s:
                    found = (chunk['samples'][position], age, chunk, index)
            index -= 1
        return (found, '') if found else (None, 'pico_not_synchronized')


class _ImuStream:
    def __init__(self, samples):
        self.groups = defaultdict(list)
        self.bad_groups = set()
        self.rejected = Counter()
        for sample in samples:
            kind, stamp = sample.get('kind'), sample.get('stamp')
            if kind not in ('accel', 'gyro') or not _number(stamp) or stamp <= 0 or not _vector(sample.get('vector_body'), 3):
                self.rejected['invalid_imu_sample'] += 1
                continue
            key = (sample.get('segment'), kind)
            group = self.groups[key]
            if group and stamp <= group[-1]['stamp']:
                self.bad_groups.add(key)
                self.rejected['imu_clock_regression'] += 1
            group.append(sample)
        self.times = {key: [item['stamp'] for item in group] for key, group in self.groups.items()}

    def at(self, stamp, segment, kind, chunk, max_sync_s):
        key = (segment, kind)
        if key in self.bad_groups or key not in self.groups:
            return None
        times = self.times[key]
        index = bisect_left(times, stamp)
        candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
        if not candidates:
            return None
        index = min(candidates, key=lambda i: (abs(times[i] - stamp), times[i]))
        if (abs(times[index] - stamp) > max_sync_s or times[index] < chunk['times'][0]
                or times[index] > chunk['times'][-1] + max_sync_s):
            return None
        return self.groups[key][index]


def _pico_reason(sample):
    if sample.get('manual_valid') is not True:
        return 'manual_input_invalid'
    if sample.get('rc_valid') is not True:
        return 'rc_invalid'
    if 'connected' in sample and sample['connected'] is not True:
        return 'pico_disconnected'
    if 'active_path' in sample and sample['active_path'] != 1:
        return 'not_propo_manual'
    if 'sbus_flags' in sample and (not isinstance(sample['sbus_flags'], int) or sample['sbus_flags'] & 12):
        return 'sbus_lost_or_failsafe'
    if 'sbus_age_ms' in sample and (not _number(sample['sbus_age_ms']) or not 0 <= sample['sbus_age_ms'] <= 50):
        return 'sbus_stale'
    if 'fault_bits' in sample and sample['fault_bits'] != 0:
        return 'pico_fault'
    for key in ('power_ready', 'sbus_fresh'):
        if key in sample and sample[key] is not True:
            return key + '_false'
    if sample.get('segment') is None:
        return 'missing_pico_segment'
    for key in ('steering_raw', 'throttle_raw'):
        value = sample.get(key)
        if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 2047:
            return 'invalid_raw_input'
    return ''


def _gravity_body(orientation, gravity):
    if gravity is None or not _vector(orientation, 4):
        return None
    length = math.hypot(*orientation)
    if not math.isfinite(length) or length < 1e-12:
        return None
    x, y, z, w = (value / length for value in orientation)
    gx, gy, gz = gravity
    return ((1 - 2 * (y*y + z*z)) * gx + 2 * (x*y + z*w) * gy + 2 * (x*z - y*w) * gz,
            2 * (x*y - z*w) * gx + (1 - 2 * (x*x + z*z)) * gy + 2 * (y*z + x*w) * gz,
            2 * (x*z + y*w) * gx + 2 * (y*z - x*w) * gy + (1 - 2 * (x*x + y*y)) * gz)


def _motion_reason(sample):
    if sample.get('valid', True) is not True or sample.get('tracking_valid', True) is not True:
        return 'tracking_invalid'
    if not _number(sample.get('stamp')) or sample['stamp'] <= 0:
        return 'invalid_motion_stamp'
    if sample.get('segment') is None:
        return 'missing_motion_segment'
    if not _vector(sample.get('velocity_body'), 3) or not _vector(sample.get('acceleration_body'), 3) or not _number(sample.get('yaw_rate_rps')):
        return 'invalid_motion_sample'
    return ''


def _empty_row(sample):
    row = dict(stamp=sample.get('stamp') if _number(sample.get('stamp')) else None, motion_valid=False,
               valid=False, reason='', segment=None, motion_segment=sample.get('segment'), pico_segment=None,
               steering_raw=None, throttle_raw=None, pico_age_s=None, vo_vx_mps=None, vo_ax_mps2=None,
               vo_yaw_rate_rps=None, speed_mps=None, curvature_inv_m=None, equivalent_steering_rad=None,
               imu_gyro_z_rps=None, imu_gyro_dt_s=None, imu_accel_x_specific_mps2=None,
               imu_accel_y_specific_mps2=None, imu_accel_z_specific_mps2=None, imu_accel_dt_s=None,
               imu_ax_at_sensor_mps2=None, gyro_minus_vo_yaw_rate_rps=None,
               longitudinal_regime=None, turn_regime=None, episode_id=None, sustained=False)
    return row


def _episodes(rows, min_duration_s, raw_tolerance):
    groups = []
    bounds = {}
    for row in rows:
        if not row['valid']:
            continue
        key = (row['segment'], row['longitudinal_regime'], row['turn_regime'])
        change = bool(groups) and any(max(row[field], bounds[field][1]) - min(row[field], bounds[field][0]) > raw_tolerance
                                      for field in ('steering_raw', 'throttle_raw'))
        if not groups or groups[-1][0] != key or change:
            groups.append((key, []))
            bounds = {field: [row[field], row[field]] for field in ('steering_raw', 'throttle_raw')}
        else:
            for field in bounds:
                bounds[field] = [min(bounds[field][0], row[field]), max(bounds[field][1], row[field])]
        groups[-1][1].append(row)
    result = []
    for identifier, (_, group) in enumerate(groups):
        duration = group[-1]['stamp'] - group[0]['stamp']
        stable = all(max(row[key] for row in group) - min(row[key] for row in group) <= raw_tolerance
                     for key in ('steering_raw', 'throttle_raw'))
        sustained = duration >= min_duration_s
        for row in group:
            row['episode_id'], row['sustained'] = identifier, sustained
        result.append(dict(id=identifier, segment=group[0]['segment'], start=group[0]['stamp'], end=group[-1]['stamp'],
                           duration_s=duration, samples=len(group), longitudinal_regime=group[0]['longitudinal_regime'],
                           turn_regime=group[0]['turn_regime'], sustained=sustained, stable_raw=stable,
                           steering_raw=_stats([row['steering_raw'] for row in group]),
                           throttle_raw=_stats([row['throttle_raw'] for row in group]),
                           vx_mps=_stats([row['vo_vx_mps'] for row in group]),
                           ax_mps2=_stats([row['vo_ax_mps2'] for row in group]),
                           yaw_rate_rps=_stats([row['vo_yaw_rate_rps'] for row in group])))
    return result


def _tables(rows, episodes, raw_width, edges, min_speed, steady_accel):
    throttle, steering, steady = defaultdict(list), defaultdict(list), defaultdict(list)
    for row in rows:
        if not row['valid']:
            continue
        direction = 'near_zero' if row['speed_mps'] < min_speed else 'forward' if row['vo_vx_mps'] > 0 else 'reverse'
        speed_bin = max(0, bisect_right(edges, row['speed_mps']) - 1)
        throttle_bin, steering_bin = row['throttle_raw'] // raw_width, row['steering_raw'] // raw_width
        throttle[throttle_bin, speed_bin, direction].append(row)
        if row['curvature_inv_m'] is not None:
            steering[steering_bin, speed_bin, direction].append(row)
        episode = episodes[row['episode_id']]
        if (row['sustained'] and episode['stable_raw'] and abs(row['vo_ax_mps2']) <= steady_accel
                and row['longitudinal_regime'] in ('steady', 'low_speed')):
            steady[throttle_bin, direction].append(row)
    response = dict(throttle_acceleration=[], throttle_steady_speed=[], steering_response=[])
    for (raw_bin, direction), group in sorted(steady.items()):
        response['throttle_steady_speed'].append(dict(raw_lo=raw_bin * raw_width,
            raw_hi=min(2047, (raw_bin + 1) * raw_width - 1), velocity_direction=direction,
            samples=len(group), speed_mps=_stats([row['speed_mps'] for row in group]),
            vx_mps=_stats([row['vo_vx_mps'] for row in group])))
    for table, groups in (('throttle_acceleration', throttle), ('steering_response', steering)):
        for (raw_bin, speed_bin, direction), group in sorted(groups.items()):
            cell = dict(raw_lo=raw_bin * raw_width, raw_hi=min(2047, (raw_bin + 1) * raw_width - 1),
                        speed_lo_mps=edges[speed_bin], speed_hi_mps=edges[speed_bin + 1] if speed_bin + 1 < len(edges) else None,
                        velocity_direction=direction, samples=len(group))
            if table == 'throttle_acceleration':
                cell['ax_mps2'] = _stats([row['vo_ax_mps2'] for row in group])
            else:
                for field in ('yaw_rate_rps', 'curvature_inv_m', 'equivalent_steering_rad'):
                    row_field = 'vo_yaw_rate_rps' if field == 'yaw_rate_rps' else field
                    cell[field] = _stats([row[row_field] for row in group])
            response[table].append(cell)
    return response


def _validation(current, reference):
    if reference is None:
        return None
    if (reference.get('schema_version') != 1 or current['binning'] != reference.get('binning')
            or current['thresholds'] != reference.get('thresholds')):
        raise ValueError('reference summary uses incompatible schema/binning/thresholds')
    method_fields = ('world_frame', 'wheelbase_m', 'gravity_world_mps2', 'motion_acceleration')
    if any(current['method'].get(key) != reference.get('method', {}).get(key) for key in method_fields):
        raise ValueError('reference summary uses incompatible frame/gravity/wheelbase assumptions')
    motion_fields = ('world_frame', 'base_frame', 'input_pose_frame', 'body_to_pose', 'window_s', 'max_gap_s',
                     'min_samples', 'max_speed_mps', 'max_angular_rate_rps', 'max_acceleration_mps2', 'derivative')
    current_motion = current['source']['motion_provenance']
    prior_motion = reference.get('source', {}).get('motion_provenance', {})
    if any(current_motion.get(key) != prior_motion.get(key) for key in motion_fields):
        raise ValueError('reference summary uses incompatible motion estimation settings')
    result = {}
    for table, statistic, unit in (('throttle_acceleration', 'ax_mps2', 'm/s^2'), ('throttle_steady_speed', 'speed_mps', 'm/s'),
                                  ('steering_response', 'curvature_inv_m', '1/m')):
        def key(cell):
            return tuple(cell.get(item) for item in ('raw_lo', 'raw_hi', 'speed_lo_mps', 'speed_hi_mps', 'velocity_direction'))
        old = {key(cell): cell for cell in reference.get('responses', {}).get(table, [])}
        cells, samples, total, squared_error = [], 0, 0, 0.0
        for cell in current['responses'][table]:
            prior = old.get(key(cell))
            observed = cell[statistic]
            total += observed['count']
            expected = prior.get(statistic, {}) if prior else {}
            if not observed['count'] or not _number(expected.get('mean')):
                continue
            error = observed['mean'] - expected['mean']
            n = observed['count']
            mse = observed['std']**2 + error**2
            samples += n
            squared_error += n * mse
            cells.append(dict(dimensions={name: cell[name] for name in ('raw_lo', 'raw_hi', 'velocity_direction', 'speed_lo_mps', 'speed_hi_mps') if name in cell},
                              reference_samples=expected.get('count'), samples=n, reference_mean=expected['mean'],
                              observed_mean=observed['mean'], mean_difference=error, rmse=math.sqrt(mse)))
        result[table] = dict(total_samples=total, covered_samples=samples, out_of_range_samples=total-samples,
                             rmse=math.sqrt(squared_error / samples) if samples else None, rmse_unit=unit, cells=cells)
    return dict(kind='held_out_cell_mean_repeatability', reference_source=reference.get('source'), tables=result)


def _rejection_counts(values):
    if isinstance(values, list):
        return dict(total=len(values), reasons=dict(Counter(item.get('reason', 'unknown') for item in values)))
    return values


def analyze(bag, motion, *, wheelbase_m=None, max_sync_s=0.05, max_gap_s=0.2, min_speed_mps=0.15,
            raw_bin_width=64, speed_bin_edges_mps=(0.0, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0),
            steady_accel_mps2=0.2, turn_rate_rps=0.1, min_regime_duration_s=0.5,
            raw_stability_tolerance=8, gravity_world_mps2=None, reference_summary=None):
    """Return (scalar time-series rows, JSON-compatible observational summary).

    Thresholds are analysis choices, not identified vehicle parameters. Optional
    gravity is the gravitational acceleration vector in the explicitly named
    VO world frame. R(world<-body)^T*g + measured specific force is acceleration
    at the IMU sensor, NOT lever-arm-corrected acceleration at the rear axle.
    """
    positive = (max_sync_s, max_gap_s, min_speed_mps, steady_accel_mps2, turn_rate_rps, min_regime_duration_s)
    if (not all(_number(value) and value > 0 for value in positive) or max_sync_s > max_gap_s
            or not isinstance(raw_bin_width, int) or isinstance(raw_bin_width, bool) or not 1 <= raw_bin_width <= 2048
            or not isinstance(raw_stability_tolerance, int) or isinstance(raw_stability_tolerance, bool) or raw_stability_tolerance < 0):
        raise ValueError('invalid synchronization/regime/bin thresholds')
    if wheelbase_m is not None and (not _number(wheelbase_m) or wheelbase_m <= 0):
        raise ValueError('wheelbase_m must be a measured positive length or None')
    edges = tuple(speed_bin_edges_mps)
    if not edges or edges[0] != 0 or not all(_number(v) and v >= 0 for v in edges) or any(a >= b for a, b in zip(edges, edges[1:])):
        raise ValueError('speed bin edges must be finite, start at zero and strictly increase')
    if gravity_world_mps2 is not None and not _vector(gravity_world_mps2, 3):
        raise ValueError('gravity_world_mps2 must be an explicit finite 3-vector or None')
    for key, expected in (('world_frame', 'odom'), ('base_frame', 'base_link')):
        if key in motion.get('provenance', {}) and motion['provenance'][key] != expected:
            raise ValueError('motion provenance must use odom / base_link')
    pico = _PicoStream(bag.get('pico', []), max_gap_s, max_sync_s)
    imu = _ImuStream(bag.get('imu', []))
    rows, segment = [], -1
    previous_stamp, previous_key, highest_stamp = None, None, -math.inf
    for sample in motion.get('samples', []):
        row = _empty_row(sample)
        reason = _motion_reason(sample)
        stamp = row['stamp']
        if not reason and stamp <= highest_stamp:
            reason = 'motion_clock_regression'
        if _number(stamp):
            highest_stamp = max(highest_stamp, stamp)
        if not reason:
            vx, ax, yaw = sample['velocity_body'][0], sample['acceleration_body'][0], sample['yaw_rate_rps']
            speed = abs(vx)
            curvature = yaw / vx if speed >= min_speed_mps else None
            speed_change = ax * (1 if vx >= 0 else -1)
            longitudinal = 'low_speed' if speed < min_speed_mps else 'accelerating' if speed_change > steady_accel_mps2 else 'decelerating' if speed_change < -steady_accel_mps2 else 'steady'
            row.update(motion_valid=True, vo_vx_mps=vx, vo_ax_mps2=ax, vo_yaw_rate_rps=yaw,
                       speed_mps=speed, curvature_inv_m=curvature,
                       equivalent_steering_rad=math.atan(wheelbase_m * curvature) if wheelbase_m is not None and curvature is not None else None,
                       longitudinal_regime=longitudinal,
                       turn_regime='left' if yaw > turn_rate_rps else 'right' if yaw < -turn_rate_rps else 'straight')
        synchronized = None
        if not reason:
            synchronized, reason = pico.at(stamp, max_sync_s)
        if synchronized:
            control, age, chunk, chunk_id = synchronized
            row.update(pico_segment=control.get('segment'), steering_raw=control.get('steering_raw'),
                       throttle_raw=control.get('throttle_raw'), pico_age_s=age)
            reason = _pico_reason(control)
            for field in ('steering_raw', 'throttle_raw'):
                if not isinstance(row[field], int) or isinstance(row[field], bool) or not 0 <= row[field] <= 2047:
                    row[field] = None
            if not reason and 'window_start' in sample and 'window_end' in sample:
                if (not _number(sample['window_start']) or not _number(sample['window_end'])
                        or not sample['window_start'] <= stamp <= sample['window_end']
                        or sample['window_start'] < chunk['times'][0]
                        or sample['window_end'] > chunk['times'][-1] + max_sync_s):
                    reason = 'motion_window_crosses_control_segment'
            for kind in ('accel', 'gyro'):
                measured = imu.at(stamp, control.get('segment'), kind, chunk, max_sync_s)
                if measured is None:
                    continue
                vector = measured['vector_body']
                if kind == 'gyro':
                    row.update(imu_gyro_z_rps=vector[2], imu_gyro_dt_s=measured['stamp'] - stamp,
                               gyro_minus_vo_yaw_rate_rps=vector[2] - yaw)
                else:
                    row.update(imu_accel_x_specific_mps2=vector[0], imu_accel_y_specific_mps2=vector[1],
                               imu_accel_z_specific_mps2=vector[2], imu_accel_dt_s=measured['stamp'] - stamp)
                    gravity = _gravity_body(sample.get('orientation'), gravity_world_mps2)
                    if gravity is not None:
                        row['imu_ax_at_sensor_mps2'] = vector[0] + gravity[0]
        if reason:
            row['reason'] = reason
            previous_key = None
            previous_stamp = stamp
            rows.append(row)
            continue
        key = (sample['segment'], chunk_id)
        if previous_key != key or previous_stamp is None or stamp - previous_stamp > max_gap_s:
            segment += 1
        previous_key, previous_stamp = key, stamp
        row.update(valid=True, segment=segment)
        rows.append(row)
    episodes = _episodes(rows, min_regime_duration_s, raw_stability_tolerance)
    usable = [row for row in rows if row['valid']]
    segment_rows = defaultdict(list)
    for row in usable:
        segment_rows[row['segment']].append(row)
    metadata_fields = ('storage_identifier', 'message_count', 'starting_time', 'duration')
    provenance_fields = ('clock', 'base_frame', 'camera_frame', 'camera_frames', 'max_gap_s', 'imu_acceleration', 'pico_input')
    summary = dict(schema_version=1,
        source=dict(bag_path=bag.get('bag_path'), metadata={key: bag.get('metadata', {})[key] for key in metadata_fields if key in bag.get('metadata', {})},
                    bag_provenance={key: bag.get('provenance', {})[key] for key in provenance_fields if key in bag.get('provenance', {})},
                    motion_provenance=motion.get('provenance', {})),
        method=dict(speed_reference='body/rear-axle VO longitudinal velocity', world_frame='odom',
                    control_sync='latest past Pico, no interpolation across continuity segments',
                    imu_sync='nearest within same Pico continuity segment',
                    motion_acceleration='world position second derivative rotated into body',
                    imu_acceleration='specific force rotated into body at sensor; gravity optional; no lever-arm correction',
                    gravity_world_mps2=list(gravity_world_mps2) if gravity_world_mps2 is not None else None,
                    wheelbase_m=wheelbase_m, steering_equivalence='atan(wheelbase * yaw_rate / longitudinal_velocity)',
                    actuator_model_fitted=False, input_sign_or_endpoints_assumed=False,
                    response_statistics='sample statistics, correlated observations; no confidence interval or response delay fitted',
                    tracking_validity='inherited motion validity/segments; a TUM file alone cannot certify VO tracking'),
        thresholds=dict(max_sync_s=max_sync_s, max_gap_s=max_gap_s, min_speed_mps=min_speed_mps,
                        steady_accel_mps2=steady_accel_mps2, turn_rate_rps=turn_rate_rps,
                        min_regime_duration_s=min_regime_duration_s, raw_stability_tolerance=raw_stability_tolerance),
        binning=dict(raw_bin_width=raw_bin_width, raw_range=[0, 2047], speed_bin_edges_mps=list(edges),
                     speed='absolute body vx', directions=['near_zero', 'forward', 'reverse'],
                     bounds='raw inclusive; speed lower-inclusive/upper-exclusive; final speed upper unbounded',
                     unobserved_cells='not listed; estimate is null/unknown, never zero; no extrapolation',
                     steady_accel_mps2=steady_accel_mps2, min_speed_mps=min_speed_mps,
                     min_regime_duration_s=min_regime_duration_s, raw_stability_tolerance=raw_stability_tolerance),
        coverage=dict(motion_samples=len(rows), motion_valid_samples=sum(row['motion_valid'] for row in rows),
                      valid_samples=len(usable), excluded_samples=len(rows)-len(usable),
                      exclusion_reasons=dict(Counter(row['reason'] for row in rows if not row['valid'])),
                      pico_preprocessing_rejected=dict(pico.rejected), imu_preprocessing_rejected=dict(imu.rejected),
                      upstream_rejected=dict(bag=_rejection_counts(bag.get('rejected', [])), motion=_rejection_counts(motion.get('rejected', []))),
                      imu_accel_samples=sum(row['imu_accel_x_specific_mps2'] is not None for row in usable),
                      imu_gyro_samples=sum(row['imu_gyro_z_rps'] is not None for row in usable),
                      gravity_corrected_samples=sum(row['imu_ax_at_sensor_mps2'] is not None for row in usable),
                      curvature_samples=sum(row['curvature_inv_m'] is not None for row in usable),
                      longitudinal_regimes=dict(Counter(row['longitudinal_regime'] for row in usable)),
                      turn_regimes=dict(Counter(row['turn_regime'] for row in usable)),
                      sustained_episode_regimes=dict(Counter(episode['longitudinal_regime'] for episode in episodes if episode['sustained'])),
                      steering_raw_range=[min(row['steering_raw'] for row in usable), max(row['steering_raw'] for row in usable)] if usable else None,
                      throttle_raw_range=[min(row['throttle_raw'] for row in usable), max(row['throttle_raw'] for row in usable)] if usable else None,
                      speed_mps=_stats([row['speed_mps'] for row in usable])),
        segments=[dict(id=identifier, motion_segment=group[0]['motion_segment'], pico_segment=group[0]['pico_segment'],
                       start=group[0]['stamp'], end=group[-1]['stamp'], duration_s=group[-1]['stamp']-group[0]['stamp'],
                       samples=len(group)) for identifier, group in segment_rows.items()], episodes=episodes)
    summary['responses'] = _tables(rows, episodes, raw_bin_width, edges, min_speed_mps, steady_accel_mps2)
    summary['validation'] = _validation(summary, reference_summary)
    return rows, summary
