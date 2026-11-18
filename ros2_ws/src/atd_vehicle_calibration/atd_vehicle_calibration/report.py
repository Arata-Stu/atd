"""CSV の数値と同じ試料を、時刻と単位を揃えて表示する。"""
from itertools import groupby
from math import nan


def plot_response(rows, path, bag_name):
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    fig = Figure(figsize=(13, 9), layout='constrained')
    FigureCanvasAgg(fig)
    fig.suptitle(f'{bag_name}   |   Vehicle response at the rear axle', fontsize=14, parse_math=False)
    grid = fig.add_gridspec(4, 2, width_ratios=[1.7, 1])
    first = fig.add_subplot(grid[0, 0])
    axes = [first] + [fig.add_subplot(grid[i, 0], sharex=first) for i in range(1, 4)]
    start = rows[0]['stamp']

    def trace(axis, field, label, color, segment):
        xs, ys = [], []
        for group, samples in groupby(rows, key=lambda row: row[segment]):
            if group is None:
                continue
            for row in samples:
                xs.append(row['stamp'] - start)
                ys.append(row[field] if row[field] is not None else nan)
            xs.append(nan)
            ys.append(nan)
        axis.plot(xs, ys, label=label, color=color, linewidth=1.0)

    trace(axes[0], 'throttle_raw', 'CH2 throttle', '#be671b', 'segment')
    trace(axes[0], 'steering_raw', 'CH1 steer', '#097d70', 'segment')
    trace(axes[1], 'vo_vx_mps', 'Stereo odometry', '#183f68', 'motion_segment')
    trace(axes[2], 'vo_ax_mps2', 'Stereo odometry', '#183f68', 'motion_segment')
    if any(row['imu_ax_at_sensor_mps2'] is not None for row in rows):
        trace(axes[2], 'imu_ax_at_sensor_mps2', 'IMU at sensor', '#be671b', 'segment')
    trace(axes[3], 'vo_yaw_rate_rps', 'Stereo odometry', '#183f68', 'motion_segment')
    trace(axes[3], 'imu_gyro_z_rps', 'IMU gyro Z', '#be671b', 'segment')
    for axis, unit in zip(axes, ['S.BUS [raw]', 'Forward speed [m/s]', 'Acceleration [m/s²]', 'Yaw rate [rad/s]']):
        axis.set_ylabel(unit)
        axis.legend(loc='upper right', frameon=False, ncols=2, fontsize=8)
        axis.grid(alpha=0.2)
        axis.label_outer()
    axes[-1].set_xlabel('Time since first motion estimate [s]')
    for index, x, y, title, unit in [
        (0, 'throttle_raw', 'vo_ax_mps2', 'Throttle response', 'Forward acceleration [m/s²]'),
        (2, 'steering_raw', 'curvature_inv_m', 'Steering response', 'Curvature [1/m]'),
    ]:
        axis = fig.add_subplot(grid[index:index + 2, 1])
        data = [r for r in rows if r['valid'] and r[x] is not None and r[y] is not None]
        if data:
            dots = axis.scatter([r[x] for r in data], [r[y] for r in data], c=[r['vo_vx_mps'] for r in data],
                                s=5, alpha=0.6, cmap='viridis', linewidths=0)
            fig.colorbar(dots, ax=axis, label='Forward speed [m/s]')
        else:
            axis.text(0.5, 0.5, 'No eligible samples', transform=axis.transAxes, ha='center')
        axis.set(title=title, xlabel='CH2 throttle [raw]' if index == 0 else 'CH1 steer [raw]', ylabel=unit)
        axis.grid(alpha=0.2)
    fig.savefig(path, dpi=140)
