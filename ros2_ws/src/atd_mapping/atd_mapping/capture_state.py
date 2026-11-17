"""ROS-independent acceptance rules for saved-map snapshot acquisition."""


class LocalizationGate:
    def __init__(self):
        self.stamp = -1
        self.since = None
        self.localized = False
        self.last_receipt = 0.0

    def update(self, stamp, values, receipt):
        if stamp < self.stamp:
            return False
        before = self.localized
        self.stamp, self.last_receipt = stamp, receipt
        self.localized = (
            values.get("localized_in_exist_map") == "Yes"
            and values.get("vo_status") == "OK"
        )
        if not self.localized:
            self.since = None
        elif not before:
            self.since = stamp
        return before != self.localized

    def accepts(self, stamp):
        return self.localized and self.since is not None and stamp >= self.since

    def require_final(self, expected_stamp, tolerance_ns):
        if not self.localized or self.stamp < expected_stamp - tolerance_ns:
            raise ValueError(
                "保存地図へのlocalize未成功・追跡喪失・再生末尾の処理未確認"
            )


def validate_final_samples(gate, expected, tolerance, cloud_stamp, path_stamp, tf_stamp):
    """更新駆動の可視化cloudと、末尾必須のdiag/path/TFを分けて判定する。"""
    gate.require_final(expected, tolerance)
    if not gate.accepts(cloud_stamp):
        raise ValueError('点群が最後のlocalize成功区間に属していません')
    if not gate.accepts(path_stamp) or path_stamp < expected - tolerance:
        raise ValueError('SLAM軌跡が最後のlocalize成功区間の再生末尾に到達していません')
    if tf_stamp is None or not gate.accepts(tf_stamp) or tf_stamp < expected - tolerance:
        raise ValueError('再生末尾のmap→odomがありません')
    return dict(cloud_tail_age_s=(expected-cloud_stamp)/1e9,
                cloud_within_tail_tolerance=cloud_stamp >= expected-tolerance)
