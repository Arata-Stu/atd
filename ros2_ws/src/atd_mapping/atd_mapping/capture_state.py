"""ROS-independent acceptance rules for saved-map snapshot acquisition."""


def require_tf_publisher(info, expected_gid):
    publisher = info.get('publisher_gid')
    if publisher is None or not any(publisher['data']):
        raise ValueError('/tf メッセージの publisher GID を取得できません')
    actual = publisher['data'].hex()
    if actual != expected_gid:
        raise ValueError(f'/tf に cuVSLAM 以外の発行元があります（GID: {actual}, 期待値: {expected_gid}）')


class LocalizationGate:
    def __init__(self):
        self.stamp = -1
        self.since = None
        self.localized = False
        self.last_receipt = 0.0
        self.map_status = None
        self.vo_status = None
        self.last_success_stamp = None

    def update(self, stamp, values, receipt):
        if stamp < self.stamp:
            return False
        before = self.localized
        self.stamp, self.last_receipt = stamp, receipt
        self.map_status = values.get("localized_in_exist_map")
        self.vo_status = values.get("vo_status")
        self.localized = self.map_status == "Yes" and self.vo_status == "OK"
        if not self.localized:
            self.since = None
        else:
            self.last_success_stamp = stamp
            if not before:
                self.since = stamp
        return before != self.localized

    def accepts(self, stamp):
        return self.localized and self.since is not None and stamp >= self.since

    def require_final(self, expected_stamp, tolerance_ns):
        issues = []
        if self.stamp < 0:
            issues.append("/diagnostics に visual_slam の診断が届いていません")
        else:
            if not self.localized:
                if self.last_success_stamp is None:
                    issues.append("保存地図へのlocalize成功を一度も確認できませんでした")
                else:
                    issues.append(f"保存地図へのlocalize成功後に追跡条件が失われました（最後の成功 {self.last_success_stamp} ns）")
            if self.stamp < expected_stamp - tolerance_ns:
                issues.append(f"診断が入力末尾より {(expected_stamp-self.stamp)/1e9:.3f} 秒遅れています")
        if issues:
            raise ValueError("、".join(issues) + "。" + self.diagnostic_summary()
                             + f"、入力末尾={expected_stamp} ns、末尾許容={tolerance_ns/1e9:.3f} 秒")

    def diagnostic_summary(self):
        stamp = f"{self.stamp} ns" if self.stamp >= 0 else "未受信"
        return (f"診断stamp={stamp}、localized_in_exist_map={self.map_status or '不明'}"
                f"、vo_status={self.vo_status or '不明'}")


def validate_final_samples(gate, expected, tolerance, cloud_stamp, path_stamp, tf_stamp,
                           *, hint_required=False, hint_stamp=None):
    """更新駆動の可視化cloudと、末尾必須のdiag/path/TFを分けて判定する。"""
    issues = []
    try:
        gate.require_final(expected, tolerance)
    except ValueError as exc:
        issues.append(str(exc))
    if hint_required and hint_stamp is None:
        issues.append('VGLの姿勢hintを受信していません')
    if gate.localized:
        for name, stamp, tail_required in (('非空点群', cloud_stamp, False),
                                           ('SLAM軌跡', path_stamp, True),
                                           ('map→odom', tf_stamp, True)):
            if stamp is None or not gate.accepts(stamp):
                issues.append(f'最終localize成功区間の{name}がありません')
            elif tail_required and stamp < expected - tolerance:
                issues.append(f'{name}が入力末尾より {(expected-stamp)/1e9:.3f} 秒遅れています'
                              f'（許容 {tolerance/1e9:.3f} 秒）')
    if issues:
        samples = [('入力末尾', expected), ('点群', cloud_stamp), ('軌跡', path_stamp), ('map→odom', tf_stamp)]
        if hint_required:
            samples.append(('VGL hint', hint_stamp))
        stamps = '、'.join(f"{name}={stamp if stamp is not None else '未受信'}" for name, stamp in samples)
        raise ValueError('。'.join(issues) + f'。受信stamp(ns) {stamps}')
    return dict(cloud_tail_age_s=(expected-cloud_stamp)/1e9,
                cloud_within_tail_tolerance=cloud_stamp >= expected-tolerance)
