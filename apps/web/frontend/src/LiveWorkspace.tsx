import { useEffect, useRef, useState } from 'react';
import { api, availabilityText } from './api';
import type { Cloud, LiveStatus, MapDocument, Parameter } from './workspace_types';
import type { Point } from './types';
import { mapIssue, streamId, trackingLabels } from './localization_view';
import type { MapReference } from './localization_view';
import { MapCanvas } from './MapCanvas';
import { Field, Icon, Tag } from './workspace_ui';

interface Sample { time: number; value: number }
function SpeedPlot({ samples, label, unit }: { samples: Sample[]; label: string; unit: string }) {
  const width = 900, height = 170, pad = 38;
  const angularExtent = Math.max(.1, ...samples.map(p => Math.abs(p.value))) * 1.1;
  const min = unit === 'rad/s' ? -angularExtent : Math.min(0, ...samples.map(p => p.value)) * 1.1;
  const max = unit === 'rad/s' ? angularExtent : Math.max(1, ...samples.map(p => p.value)) * 1.1;
  const first = samples[0]?.time ?? 0, last = samples.at(-1)?.time ?? 1;
  const span = Math.max(30, last - first);
  const line = samples.map((p, index) => `${!index || p.time - samples[index - 1].time > 5 ? 'M' : 'L'}${pad + (p.time - (last - span)) / span * (width - pad - 16)},${height - pad - (p.value - min) / (max - min) * (height - pad - 18)}`).join(' ');
  return <div className="speed-plot"><svg viewBox={`0 0 ${width} ${height}`} role="img" aria-label={`受信した${label}の時系列。横軸は測定時刻の秒、縦軸は${unit}。`}>{[0, .5, 1].map(p => <g key={p}><line x1={pad} y1={height - pad - p * (height - pad - 18)} x2={width - 16} y2={height - pad - p * (height - pad - 18)} stroke="#e0e6e3"/><text x="2" y={height - pad - p * (height - pad - 18) + 4} fontSize="11" fill="#87958d">{(min + p * (max - min)).toFixed(1)}</text></g>)}<text x={width - 35} y={height - 6} fontSize="11" fill="#87958d">秒</text><text x={pad} y={height - 6} fontSize="11" fill="#87958d">−{span.toFixed(0)}</text><text x={width - 55} y={height - 6} fontSize="11" fill="#87958d">0</text><path d={line} fill="none" stroke="#16876b" strokeWidth="2.5" strokeLinejoin="round"/></svg>{!samples.length && <span className="chart-empty">{label}データの受信を待っています</span>}</div>;
}
export function LiveWorkspace({ notify }: { notify: (message: string, error?: boolean) => void }) {
  const [status, setStatus] = useState<LiveStatus | null>(null), [error, setError] = useState(''), [enabled, setEnabled] = useState(true);
  const [maps, setMaps] = useState<MapDocument[]>([]), [mapId, setMapId] = useState(''), [reference, setReference] = useState<MapReference | null>(null), [cloud, setCloud] = useState<Cloud | null>(null);
  const [samples, setSamples] = useState<Sample[]>([]), [node, setNode] = useState(''), [parameters, setParameters] = useState<Parameter[]>([]), [drafts, setDrafts] = useState<Record<string, string>>({}), [busy, setBusy] = useState(false);
  const [angularSamples, setAngularSamples] = useState<Sample[]>([]), [plotMetric, setPlotMetric] = useState<'linear' | 'angular'>('linear');
  const [expired, setExpired] = useState(false);
  const [mapError, setMapError] = useState(''), [mapReload, setMapReload] = useState(0);
  const selectedId = mapId || status?.localization?.map_id || '';
  const doc = reference?.document.id === selectedId ? reference.document : null;
  const deployment = status?.localization?.hdmap_sha256;
  const [trail, setTrail] = useState<Point[]>([]);
  const poseStream = useRef('');
  const velocityStream = useRef(''), lastVelocityStamp = useRef<number | null>(null);
  useEffect(() => { api<{ items: MapDocument[] }>('maps').then(x => setMaps(x.items)).catch(e => notify(e.message, true)); }, [notify]);
  useEffect(() => {
    if (!enabled) return; let cancelled = false; let timer: ReturnType<typeof setTimeout>, expiry: ReturnType<typeof setTimeout>;
    async function poll() {
      try {
        const start = performance.now();
        const value = await api<LiveStatus>('live/status'); if (cancelled) return; setStatus(value); setError('');
        clearTimeout(expiry);
        const loc = value.localization;
        const remaining = loc ? loc.remaining_s * 1000 - (performance.now() - start) : 2500;
        const timely = Number.isFinite(remaining) && remaining > 0;
        setExpired(!timely);
        const stream = timely ? streamId(value) : '';
        if (stream !== poseStream.current) { poseStream.current = stream; setTrail([]); }
        if (timely) expiry = setTimeout(() => { setExpired(true); setTrail([]); poseStream.current = ''; }, remaining);
        if (stream && value.pose?.frame === 'map' && !value.pose.stale) {
          const pose = value.pose;
          setTrail(previous => { const last = previous.at(-1); return last && Math.hypot(last[0] - pose.x, last[1] - pose.y) < .01 ? previous : [...previous.slice(-5999), [pose.x, pose.y]]; });
        }
        const velocity = timely && (!loc || stream) ? value.velocity : null;
        const nextVelocityStream = loc ? stream : `${value.identity?.instance_id ?? ''}:${velocity?.frame ?? ''}`;
        if (nextVelocityStream !== velocityStream.current) { velocityStream.current = nextVelocityStream; lastVelocityStamp.current = null; setSamples([]); setAngularSamples([]); }
        if (value.connected && velocity && !velocity.stale && typeof velocity.age_s === 'number' && Number.isFinite(velocity.age_s) && velocity.age_s >= 0 && typeof velocity.stamp === 'number' && Number.isFinite(velocity.stamp) && velocity.stamp >= 0 && velocity.stamp !== lastVelocityStamp.current) {
          const time = velocity.stamp;
          if (lastVelocityStamp.current !== null && time < lastVelocityStamp.current) { setSamples([]); setAngularSamples([]); }
          lastVelocityStamp.current = time;
          if (Number.isFinite(velocity.linear_mps)) setSamples(previous => [...previous.slice(-239), { time, value: velocity.linear_mps }]);
          if (Number.isFinite(velocity.angular_rps)) setAngularSamples(previous => [...previous.slice(-239), { time, value: velocity.angular_rps }]);
        }
      } catch (e) { if (!cancelled) { setError((e as Error).message); setStatus(null); setTrail([]); poseStream.current = ''; } }
      if (!cancelled) timer = setTimeout(poll, 750);
    }
    void poll(); return () => { cancelled = true; clearTimeout(timer); clearTimeout(expiry); };
  }, [enabled]);
  useEffect(() => {
    let cancelled = false;
    setReference(null); setCloud(null); setMapError('');
    if (!selectedId) return;
    const path = 'maps/' + encodeURIComponent(selectedId);
    api<MapReference>(path + '/live-reference')
      .then(map => { if (!cancelled) setReference(map); })
      .catch(e => { if (!cancelled) setMapError(e.message); });
    api<Cloud>(path + '/cloud')
      .then(points => { if (!cancelled) setCloud(points); })
      .catch(() => { /* The Raceline overlay can also be used without a point cloud. */ });
    return () => { cancelled = true; };
  }, [selectedId, deployment, mapReload]);
  async function loadParameters() {
    if (!node) return; setBusy(true);
    try { const result = await api<{ items: Parameter[] }>('live/parameters?node=' + encodeURIComponent(node)); setParameters(result.items); setDrafts(Object.fromEntries(result.items.map(p => [p.name, typeof p.value === 'string' ? p.value : JSON.stringify(p.value)]))); }
    catch (e) { notify((e as Error).message, true); } finally { setBusy(false); }
  }
  const loc = status?.localization;
  const current = enabled && status?.connected && !expired && (!loc || (!loc.stale && loc.state === 'tracking'));
  const freshPose = current && status?.pose && !status.pose.stale ? status.pose : null;
  const issue = loc && doc && reference ? mapIssue(loc, reference) : '';
  const mapPose = loc && doc && !issue && freshPose?.frame === 'map' ? freshPose : null;
  const freshVelocity = current && status?.velocity && !status.velocity.stale ? status.velocity : null;
  const tracking = !enabled ? '表示停止' : !status?.connected ? '未接続' : !loc ? '未受信'
    : expired || loc.stale ? '更新途絶' : trackingLabels[loc.state];
  return <div className="content-page live-page"><div className="page-intro"><div><span className="eyebrow">LIVE OBSERVATION</span><h2>車両の現在を、ひとつの画面で。</h2><p>Jetsonの自己位置・速度と、公開されたパラメータを確認します。</p></div><div className="button-row"><span className={'connection-indicator ' + (enabled && status?.connected ? 'online' : '')}><i/>{!enabled ? '表示を一時停止' : status?.connected ? '観測接続中' : '未接続'}</span><button className="button" onClick={() => { setStatus(null); setTrail([]); setEnabled(!enabled); }}>{enabled ? '表示を一時停止' : '表示を再開'}</button></div></div>
    {(error || (!status?.connected && status)) && <div className="inline-note"><Icon name="info" size={17}/><span>{error || availabilityText(status?.availability) || 'Jetsonの観測接続を確認してください。「データ転送」の接続設定を共用します。'}</span></div>}
    <div className="telemetry-grid"><div className="metric-card"><span>車速</span><strong>{freshVelocity ? freshVelocity.linear_mps.toFixed(2) : '—'}<small>m/s</small></strong><span className="metric-caption">{freshVelocity ? `受信 ${freshVelocity.age_s?.toFixed(1) ?? '—'} 秒前` : '新しい速度データなし'}</span></div><div className="metric-card"><span>自己位置 X / Y</span><strong className="coordinate-value">{freshPose ? `${freshPose.x.toFixed(2)} / ${freshPose.y.toFixed(2)}` : '— / —'}<small>m</small></strong><span className="metric-caption">{freshPose?.frame || '座標系未受信'}</span></div><div className="metric-card"><span>姿勢角</span><strong>{freshPose ? (freshPose.yaw * 180 / Math.PI).toFixed(1) : '—'}<small>°</small></strong><span className="metric-caption">{freshPose ? `${freshPose.frame} 座標系のYaw` : '座標系未受信'}</span></div><div className="metric-card"><span>自己位置の追跡状態</span><strong aria-live="polite">{tracking}</strong><span className="metric-caption">{loc ? loc.map_id : '地図 ID 未受信'}</span></div></div>
    <div className="live-columns"><section className="content-card live-map"><div className="card-heading"><h3>Raceline と自己位置</h3><select aria-label="表示する地図" value={mapId} onChange={e => setMapId(e.target.value)}><option value="">Jetson の地図を表示{loc?.map_id ? ' · ' + loc.map_id : ''}</option>{maps.map(map => <option value={map.id} key={map.id}>{map.title || map.id}</option>)}</select></div><MapCanvas doc={doc} cloud={doc ? cloud : null} visible={{centerline: false, custom: false}} pose={mapPose} trail={mapPose ? trail : []}/>
      <div className="localization-legend"><span><i/>Raceline</span><span><i/>実走軌跡</span><span>矢印は車両の前方</span></div>
      {(issue || mapError) && <div className="inline-note" role="status"><span>{mapError ? `Notebook の地図「${selectedId}」を確認してください。${mapError}` : issue}</span>
        <button className="button small" onClick={() => setMapReload(n => n + 1)}>地図を再読込</button></div>}
      {doc && !doc.lines.raceline?.points?.length && <p className="inline-note">この地図には Raceline が保存されていません。</p>}
      {loc && tracking !== '追跡中' && <p className="small-note">{expired ? '新しい状態の受信を待っています。' : loc.reason}</p>}</section>
    <section className="content-card parameter-card"><div className="card-heading"><h3>パラメータ調整</h3><Icon name="settings" size={18}/></div><div className="parameter-body"><Field label="対象ノード"><select value={node} onChange={e => { setNode(e.target.value); setParameters([]); }}><option value="">ノードを選択</option>{status?.parameter_nodes?.map(name => <option key={name} value={name}>{name}</option>)}</select></Field><button className="button full" disabled={!node || busy} onClick={() => void loadParameters()}><Icon name="refresh" size={15}/>現在値を取得</button>{!parameters.length && <p className="small-note">変更を許可されたパラメータだけを操作できます。値は明示的に適用します。</p>}<div className="parameter-list">{parameters.map(parameter => <div className="parameter-item" key={parameter.name}><div><strong>{parameter.name}</strong>{(parameter.read_only || !parameter.allowed) && <Tag>{parameter.read_only ? '読取専用' : '変更対象外'}</Tag>}</div>{parameter.description && <p>{parameter.description}</p>}<div className="parameter-input"><input aria-label={parameter.name} disabled={parameter.read_only || !parameter.allowed || busy} value={drafts[parameter.name] ?? ''} onChange={e => setDrafts({ ...drafts, [parameter.name]: e.target.value })}/><button className="button small" disabled={parameter.read_only || !parameter.allowed || busy || drafts[parameter.name] === (typeof parameter.value === 'string' ? parameter.value : JSON.stringify(parameter.value))} onClick={async () => { setBusy(true); try { const text = drafts[parameter.name]; const value = typeof parameter.value === 'string' ? text : JSON.parse(text); await api('live/parameters', { node, values: { [parameter.name]: value } }); await loadParameters(); notify(`${parameter.name} を適用しました`); } catch (e) { notify((e as Error).message, true); } finally { setBusy(false); } }}>適用</button></div></div>)}</div></div></section></div>
    <section className="content-card"><div className="card-heading"><div className="button-row"><h3>走行データの時系列</h3><select className="plot-selector" aria-label="時系列に表示する値" value={plotMetric} onChange={e => setPlotMetric(e.target.value as 'linear' | 'angular')}><option value="linear">車速 [m/s]</option><option value="angular">角速度 [rad/s]</option></select></div><button className="button quiet small" onClick={() => { setSamples([]); setAngularSamples([]); }}>履歴をクリア</button></div><SpeedPlot samples={plotMetric === 'linear' ? samples : angularSamples} label={plotMetric === 'linear' ? '車速' : '角速度'} unit={plotMetric === 'linear' ? 'm/s' : 'rad/s'}/></section>
  </div>;
}
