import { clockText } from './live_timing';
import { MapCanvas } from './MapCanvas';
import { ReferenceLineLegend } from './SpeedLegend';
import { mapIssue, trackingLabels, trackingPose } from './localization_view';
import type { MapReference } from './localization_view';
import { captureRoute, LINE_NAMES } from './planning_view';
import { controlOverlay, liveControlIssue } from './control_view';
import type { Point } from './types';
import type { GazeboGroundTruth, LiveStatus } from './workspace_types';

export function GazeboGroundTruthLegend({truth}: {truth: GazeboGroundTruth | null}) {
  return <span title={truth ? `接触判定の車体外形（余裕 ${truth.footprint.margin} m）。青い印は前方。GT 観測 ${truth.stamp_s.toFixed(3)} s` : '地図に整列した新しい Gazebo GT を待っています。'}>
    <i className="gt-footprint"/>GT 車体{!truth && ' 待機中'}
  </span>;
}

export function LookaheadLegend({distance}: {distance?: number | null}) {
  return <span title="Pure Pursuit の計算時の後輪軸から先読み点までの直線距離">
    <i className="control-target"/>先読み点 {distance == null ? '—' : `${distance.toFixed(2)} m`}
  </span>;
}

export function CaptureMap({status, reference, trail, enabled, expired, planExpired, controlExpired, groundTruth, error, reload}: {
  status: LiveStatus | null; reference: MapReference | null; trail: Point[];
  enabled: boolean; expired: boolean; planExpired: boolean; controlExpired: boolean; error: string; reload: () => void;
  groundTruth?: GazeboGroundTruth | null;
}) {
  const loc = status?.localization;
  const mapIdentity = loc ?? groundTruth;
  const mismatch = mapIdentity && reference ? mapIssue(mapIdentity, reference) : '';
  const doc = mapIdentity && reference && !mismatch ? reference.document : null;
  const tracking = !enabled ? '表示停止' : !status?.connected ? '未接続' : !loc ? '未受信'
    : expired || loc.stale ? '更新途絶' : trackingLabels[loc.state];
  const pose = doc && enabled && !expired ? trackingPose(status) : null;
  const {line, plan, issue: routeIssue} = captureRoute(status, expired || planExpired);
  const hasLine = !!doc && !!line && (doc.lines[line]?.points?.length ?? 0) >= 2;
  const path = hasLine && pose ? plan?.trajectory : undefined;
  const control = hasLine && pose && plan && !liveControlIssue(status, controlExpired) ? status?.control : null;
  const mapId = mapIdentity?.map_id || status?.auto_drive?.current?.map_id;
  const mapLabel = doc?.title && doc.title !== mapId ? `${doc.title} (${mapId})` : mapId;
  let issue = mismatch || error;
  if (!issue && enabled && status?.connected) {
    if (!loc) issue = '自己位置情報の受信を待っています。';
    else if (!doc) issue = 'Notebook の保存地図を照合中です。';
    else if (expired || loc.stale) issue = '新しい自己位置の受信を待っています。';
    else if (loc.state !== 'tracking') issue = loc.reason;
    else if (!pose) issue = '同じ観測時刻の map 座標の自己位置を待っています。';
    else if (routeIssue) issue = routeIssue;
    else if (line && !hasLine) issue = `地図に ${LINE_NAMES[line]} が保存されていません。`;
  }
  return <section className="capture-map" aria-label="走行地図と自己位置">
    <header className="capture-map-heading"><h4>走行地図と自己位置</h4><span>{tracking}</span></header>
    <p className="capture-map-id">{mapLabel || '地図未確認'} · {line ? LINE_NAMES[line] : 'ライン未確認'}</p>
    {doc && <MapCanvas doc={doc} autoFit referenceLine={hasLine ? line : null}
      pose={pose} groundTruth={groundTruth} trail={pose ? trail : []} forwardPath={path} planPose={plan?.vehicle_pose}
      controlTarget={controlOverlay(control, control?.vehicle_pose)}/>}
    {doc && <div className="capture-map-legend planning-legend">
      {line && hasLine && <ReferenceLineLegend raceline={line === 'raceline' ? doc.lines.raceline : undefined}>
        <span><i/>{LINE_NAMES[line]}</span>
      </ReferenceLineLegend>}
      {!!path?.length && <span><i className="plan-path"/>前方軌道</span>}
      {hasLine && <LookaheadLegend distance={control?.actual_lookahead_m}/>}
      {!!pose && (groundTruth !== undefined
        ? <span><i className="plan-trail"/>推定軌跡・姿勢</span>
        : <><span><i className="plan-trail"/>実走軌跡</span><span>矢印は車両の向き</span></>)}
      {groundTruth !== undefined && <GazeboGroundTruthLegend truth={groundTruth}/>}
    </div>}
    {pose && loc && <p className="capture-map-meta">X {pose.x.toFixed(2)} m · Y {pose.y.toFixed(2)} m ·
      向き {(pose.yaw * 180 / Math.PI).toFixed(1)}° · 観測 {status?.identity?.environment === 'gazebo' ? loc.stamp.toFixed(3) + ' s' : clockText(loc.stamp)}</p>}
    {issue && <div className="capture-map-issue" role="status">{issue}
      {(error || mismatch) && <button className="button small" onClick={reload}>地図を再読込</button>}</div>}
  </section>;
}
