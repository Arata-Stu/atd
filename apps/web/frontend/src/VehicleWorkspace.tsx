import { useEffect, useRef, useState } from 'react';
import { api } from './api';
import { Field, Icon, Tag } from './workspace_ui';
import { VEHICLE_DIMENSIONS, VEHICLE_LABELS, VEHICLE_RPY, VEHICLE_XYZ, sameVehicleDraft, vehicleApplied, vehicleBodyPlan, vehicleCadReference, vehicleCameraAxes, vehicleDraft, vehicleDraftProfile, vehicleIdentityMatches, vehicleProfileIssue, vehicleSourceText, vehicleStatusFresh } from './vehicle_profile';
import type { VehicleCadReference, VehicleDimension, VehicleDraft, VehicleIdentity, VehicleProfile, VehicleProfileResult, VehicleState } from './vehicle_profile';

const millimetres = (value: number | null) => value === null ? '未測定' : `${Number((value * 1000).toPrecision(8))} mm`;
const POSE_LABELS = { x: 'X · 前方向', y: 'Y · 左方向', z: 'Z · 上方向', roll: 'Roll · X軸回転', pitch: 'Pitch · Y軸回転', yaw: 'Yaw · Z軸回転' };
type MeasurementKey = VehicleDimension | `camera_${keyof typeof POSE_LABELS}`;

function MeasurementGuide({ profile, active, select, hover, side }: { profile: VehicleProfile; active: MeasurementKey | null; select: (key: MeasurementKey) => void; hover: (key: MeasurementKey | null) => void; side: boolean }) {
  function dimension(key: VehicleDimension, x1: number, y1: number, x2: number, y2: number, tx: number, ty: number, anchor: 'start' | 'middle' | 'end' = 'middle') {
    const vertical = x1 === x2;
    return <g key={key} className={'vehicle-dimension-line' + (active === key ? ' active' : '')} onMouseEnter={() => hover(key)} onMouseLeave={() => hover(null)} onClick={() => select(key)}>
      <g role="button" tabIndex={0} aria-label={`測定箇所 ${VEHICLE_LABELS[key].label}`} onFocus={() => hover(key)} onBlur={() => hover(null)} onKeyDown={event => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); select(key); } }}>
      <rect x={Math.min(x1, x2) - 8} y={Math.min(y1, y2) - 8} width={Math.abs(x2 - x1) + 16} height={Math.abs(y2 - y1) + 16} fill="transparent"/>
      <line x1={x1} y1={y1} x2={x2} y2={y2}/><line x1={x1 - (vertical ? 5 : 0)} y1={y1 - (vertical ? 0 : 5)} x2={x1 + (vertical ? 5 : 0)} y2={y1 + (vertical ? 0 : 5)}/><line x1={x2 - (vertical ? 5 : 0)} y1={y2 - (vertical ? 0 : 5)} x2={x2 + (vertical ? 5 : 0)} y2={y2 + (vertical ? 0 : 5)}/>
      </g>
      <text x={tx} y={ty} textAnchor={anchor}>{VEHICLE_LABELS[key].label} {millimetres(profile.dimensions[key].value_m)}</text>
    </g>;
  }
  return <div className="vehicle-measurement-diagrams"><figure style={{ display: side ? 'none' : undefined }}><figcaption><strong>どこを測るか · 上面</strong><span>測定位置の模式図 · 縮尺なし</span></figcaption><svg viewBox="0 0 640 360" role="img" aria-label="車体寸法の上面測定ガイド">
    <rect x="150" y="130" width="320" height="120" rx="12" fill="#eaf1ec" stroke="#bacdbf" strokeDasharray="6 4"/>
    <path d="M220 145v90M410 145v90" stroke="#a3b8aa"/>{[220, 410].flatMap(x => [145, 235].map(y => <circle key={x + '-' + y} cx={x} cy={y} r="5" fill="#fff" stroke="#91ab99"/>))}
    <path d="M220 190h48m-5-4 5 4-5 4M220 190v-35m-4 5 4-5 4 5" fill="none" stroke="#638b70"/><circle cx="220" cy="190" r="4" fill="#528064"/><text x="226" y="209">後輪車軸中心 · base_link</text><text x="274" y="193">+x 前</text><text x="226" y="155">+y 左</text>
    {dimension('body_length', 150, 65, 470, 65, 310, 50)}
    {dimension('body_width', 520, 130, 520, 250, 540, 118, 'end')}
    {dimension('rear_track', 100, 145, 100, 235, 80, 120, 'start')}
    {dimension('front_track', 440, 145, 440, 235, 425, 278, 'start')}
    {dimension('wheelbase', 220, 310, 410, 310, 315, 337)}
    {dimension('rear_overhang', 150, 280, 220, 280, 205, 300, 'end')}
  </svg></figure><figure style={{ display: side ? undefined : 'none' }}><figcaption><strong>どこを測るか · 側面</strong><span>測定位置の模式図 · 縮尺なし</span></figcaption><svg viewBox="0 0 640 360" role="img" aria-label="車体全高の側面測定ガイド">
    <path d="M150 220V150h65l35-30h120l45 30h55v70Z" fill="#eaf1ec" stroke="#bacdbf" strokeDasharray="6 4"/><path d="M115 245h420" stroke="#b3c2b8"/>
    {[220, 410].map(x => <g key={x}><circle cx={x} cy="230" r="15" fill="#f8fbf8" stroke="#9db6a5"/><circle cx={x} cy="230" r="3" fill="#648a71"/></g>)}
    <text x="145" y="265">路面</text><text x="209" y="286">後輪車軸中心 · base_link</text><path d="M220 230h42m-5-4 5 4-5 4M220 230v-38m-4 5 4-5 4 5" fill="none" stroke="#638b70"/><text x="270" y="234">+x 前</text><text x="226" y="193">+z 上</text>
    {dimension('body_height', 520, 120, 520, 245, 525, 103, 'end')}
    <text x="150" y="322">全高は路面から測定。車軸中心からの高さではありません。</text>
  </svg></figure><p>寸法線を選ぶと入力欄へ移動します。入力欄に触れると対応する寸法線が強調されます。模式図の形状・タイヤ径は実寸を示しません。</p></div>;
}

function VehicleProjection({ profile, side = false, active, select, hover }: { profile: VehicleProfile; side?: boolean; active: MeasurementKey | null; select: (key: MeasurementKey) => void; hover: (key: MeasurementKey | null) => void }) {
  const body = vehicleBodyPlan(profile), dimensions = profile.dimensions;
  const wheelbase = dimensions.wheelbase.value_m, frontTrack = dimensions.front_track.value_m, rearTrack = dimensions.rear_track.value_m;
  const cameraX = profile.camera.translation_m.x, cameraV = profile.camera.translation_m[side ? 'z' : 'y'];
  const cameraKnown = cameraX !== null && cameraV !== null && VEHICLE_XYZ.every(key => profile.camera.translation_m[key] !== null);
  const points: [number, number][] = [[0, 0]];
  if (wheelbase !== null) points.push([wheelbase, 0]);
  if (!side && body) points.push([body.rear, -body.halfWidth], [body.front, body.halfWidth]);
  if (!side && rearTrack !== null) points.push([0, -rearTrack / 2], [0, rearTrack / 2]);
  if (!side && wheelbase !== null && frontTrack !== null) points.push([wheelbase, -frontTrack / 2], [wheelbase, frontTrack / 2]);
  if (cameraKnown) points.push([cameraX, cameraV]);
  const minX = Math.min(...points.map(point => point[0])), maxX = Math.max(...points.map(point => point[0]));
  const minV = Math.min(...points.map(point => point[1])), maxV = Math.max(...points.map(point => point[1]));
  const spanX = Math.max(.3, maxX - minX + .12), spanV = Math.max(.22, maxV - minV + .12);
  const scale = Math.min(410 / spanX, 210 / spanV), centerX = (minX + maxX) / 2, centerV = (minV + maxV) / 2;
  const sx = (x: number) => 260 + (x - centerX) * scale, sv = (v: number) => 144 - (v - centerV) * scale;
  const left = centerX - 205 / scale, right = centerX + 205 / scale, low = centerV - 105 / scale, high = centerV + 105 / scale;
  const step = [.025, .05, .1, .2, .5, 1, 2, 5, 10].find(value => value >= Math.max(spanX, spanV) / 6) ?? 10;
  const ticks = (minimum: number, maximum: number) => Array.from({ length: Math.max(0, Math.floor(maximum / step) - Math.ceil(minimum / step) + 1) }, (_, index) => (Math.ceil(minimum / step) + index) * step);
  const axes = vehicleCameraAxes(profile), axisNames = ['x', 'y', 'z'], axisColors = ['#bb695e', '#548e68', '#638cb6'];
  const marker = side ? 'vehicle-side-arrow' : 'vehicle-plan-arrow';
  function cameraOffset(axis: 'x' | 'y' | 'z', x1: number, y1: number, x2: number, y2: number, tx: number, ty: number, value: number) {
    const key: MeasurementKey = `camera_${axis}`;
    return <g className={'vehicle-camera-offset' + (active === key ? ' active' : '')} onMouseEnter={() => hover(key)} onMouseLeave={() => hover(null)} onClick={() => select(key)}>
      <g role="button" tabIndex={0} aria-label={`測定箇所 カメラ ${axis.toUpperCase()}`} onFocus={() => hover(key)} onBlur={() => hover(null)} onKeyDown={event => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); select(key); } }}>
        <rect x={Math.min(x1, x2) - 8} y={Math.min(y1, y2) - 8} width={Math.abs(x2 - x1) + 16} height={Math.abs(y2 - y1) + 16} fill="transparent"/>
        <line x1={x1} y1={y1} x2={x2} y2={y2}/>
      </g><text x={tx} y={ty}>{axis.toUpperCase()} {millimetres(value)}</text>
    </g>;
  }
  return <figure className="vehicle-projection"><figcaption><strong>{side ? '側面 · XZ' : '上面 · XY'}</strong><span>{side ? 'x 前 / z 上' : 'x 前 / y 左'} · 単位 mm</span></figcaption>
    <svg viewBox="0 0 520 302" role="img" aria-label={side ? '車両とカメラ配置の側面図' : '車両とカメラ配置の上面図'}>
      <defs><marker id={marker} viewBox="0 0 8 8" refX="7" refY="4" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0 0 8 4 0 8" fill="#688577"/></marker></defs>
      <rect x="46" y="28" width="428" height="232" rx="7" fill="#f8fbf8"/>
      {ticks(left, right).map(value => <line key={'x' + value} x1={sx(value)} y1="39" x2={sx(value)} y2="249" stroke="#e8efea"/>)}
      {ticks(low, high).map(value => <line key={'v' + value} x1="55" y1={sv(value)} x2="465" y2={sv(value)} stroke="#e8efea"/>)}
      {!side && body && <rect x={sx(body.rear)} y={sv(body.halfWidth)} width={(body.front - body.rear) * scale} height={2 * body.halfWidth * scale} rx="4" fill="#dfece4" fillOpacity=".7" stroke="#98b3a2" strokeDasharray="5 3"/>}
      {!side && rearTrack !== null && <g stroke="#85968c"><line x1={sx(0)} y1={sv(-rearTrack / 2)} x2={sx(0)} y2={sv(rearTrack / 2)}/>{[-1, 1].map(direction => <circle key={direction} cx={sx(0)} cy={sv(direction * rearTrack / 2)} r="4" fill="#fff"/>)}</g>}
      {wheelbase !== null && <g><circle cx={sx(wheelbase)} cy={sv(0)} r="3" fill="#97aa9c"/><text x={sx(wheelbase)} y={sv(0) + 18} textAnchor="middle">前輪車軸中心</text>{!side && frontTrack !== null && <g stroke="#85968c"><line x1={sx(wheelbase)} y1={sv(-frontTrack / 2)} x2={sx(wheelbase)} y2={sv(frontTrack / 2)}/>{[-1, 1].map(direction => <circle key={direction} cx={sx(wheelbase)} cy={sv(direction * frontTrack / 2)} r="4" fill="#fff"/>)}</g>}</g>}
      <g stroke="#688577" strokeWidth="1.4"><line x1={sx(0)} y1={sv(0)} x2={sx(0) + 37} y2={sv(0)} markerEnd={`url(#${marker})`}/><line x1={sx(0)} y1={sv(0)} x2={sx(0)} y2={sv(0) - 37} markerEnd={`url(#${marker})`}/></g>
      <circle cx={sx(0)} cy={sv(0)} r="4" fill="#477b60"/>
      <text x={sx(0) + 42} y={sv(0) + 4}>+x</text><text x={sx(0) + 5} y={sv(0) - 40}>{side ? '+z' : '+y'}</text>
      <text x={sx(0) - 8} y={sv(0) + 18} textAnchor="end">base_link</text>
      {cameraKnown && <g>{cameraOffset('x', sx(0), sv(0), sx(cameraX), sv(0), (sx(0) + sx(cameraX)) / 2, sv(0) - 7, cameraX)}{cameraOffset(side ? 'z' : 'y', sx(cameraX), sv(0), sx(cameraX), sv(cameraV), sx(cameraX) + 7, (sv(0) + sv(cameraV)) / 2 - 5, cameraV)}<g pointerEvents="none"><circle cx={sx(cameraX)} cy={sv(cameraV)} r="6" fill={profile.camera.enabled ? '#4f96b3' : '#fff'} stroke="#4f96b3" strokeWidth="2"/>
        {axes?.map((axis, index) => { const dx = axis[0] * 30, dy = -axis[side ? 2 : 1] * 30; return Math.hypot(dx, dy) < 2 ? null : <g key={index}><line x1={sx(cameraX)} y1={sv(cameraV)} x2={sx(cameraX) + dx} y2={sv(cameraV) + dy} stroke={axisColors[index]} strokeWidth={active === `camera_${['roll', 'pitch', 'yaw'][index]}` ? 4 : 2}/><text x={sx(cameraX) + dx + 3} y={sv(cameraV) + dy - 3} fill={axisColors[index]}>{axisNames[index]}</text></g>; })}
        <text x={sx(cameraX) + 10} y={sv(cameraV) + 38}>camera_link{profile.camera.enabled ? '' : '（候補）'}</text></g></g>}
      {wheelbase !== null && <g stroke="#a0b2a4"><line x1={sx(0)} y1="269" x2={sx(wheelbase)} y2="269"/><line x1={sx(0)} y1="265" x2={sx(0)} y2="273"/><line x1={sx(wheelbase)} y1="265" x2={sx(wheelbase)} y2="273"/><text x={(sx(0) + sx(wheelbase)) / 2} y="287" textAnchor="middle" stroke="none">ホイールベース {millimetres(wheelbase)}</text></g>}
      <text x="58" y="21">1目盛 {Number((step * 1000).toPrecision(6))} mm</text>
    </svg>
    <p>{side ? 'z=0 は後輪車軸中心です。地面・車体下端の高さは補完していません。' : body ? '車体外形は左右対称の模式図です。丸印は車輪の回転中心を示します。' : '外形は全長・全幅・後方オーバーハングを入力すると表示します。'}</p>
    {!cameraKnown && <p>この面のカメラ位置は未測定です。</p>}
  </figure>;
}

export function VehicleWorkspace({ notify, onDirty }: { notify: (message: string, error?: boolean) => void; onDirty: (value: boolean) => void }) {
  const [draft, setDraft] = useState<VehicleDraft | null>(null), [persisted, setPersisted] = useState<VehicleProfile | null>(null);
  const [cadReference, setCadReference] = useState<VehicleCadReference | null>(null);
  const [revision, setRevision] = useState(''), [path, setPath] = useState(''), [saved, setSaved] = useState(false);
  const [state, setState] = useState<VehicleState | null>(null), [identity, setIdentity] = useState<VehicleIdentity | null>(null);
  const [receivedAt, setReceivedAt] = useState(0), [now, setNow] = useState(Date.now());
  const [busy, setBusy] = useState(false), [error, setError] = useState(''), [statusError, setStatusError] = useState(''), [bindingInvalid, setBindingInvalid] = useState(false);
  const [activeMeasurement, setActiveMeasurement] = useState<MeasurementKey | null>(null), [diagramMode, setDiagramMode] = useState<'guide' | 'pose'>('guide'), [diagramSide, setDiagramSide] = useState(false);
  const measurementInputs = useRef<Record<string, HTMLInputElement | null>>({}), focusedMeasurement = useRef<MeasurementKey | null>(null);
  const mounted = useRef(true), profileEpoch = useRef(0), statusEpoch = useRef(0), mutationPending = useRef(false);
  const target = useRef<VehicleIdentity | null>(null), lastConnected = useRef<boolean | null>(null);
  const dirty = !!draft && !sameVehicleDraft(draft, persisted && vehicleDraft(persisted));
  const matching = !bindingInvalid && vehicleIdentityMatches(identity, state?.identity);
  const responseFresh = !!state?.connected && receivedAt > 0 && now >= receivedAt && now - receivedAt <= 3000;
  const fresh = matching && vehicleStatusFresh(state, receivedAt, now);
  const applied = saved && matching && vehicleApplied(state, receivedAt, now, persisted, path);
  const converted = draft ? vehicleDraftProfile(draft) : null;
  const profile = converted?.profile;
  const dimensionCount = profile ? VEHICLE_DIMENSIONS.filter(key => profile.dimensions[key].value_m !== null).length : 0;

  useEffect(() => { onDirty(dirty); }, [dirty, onDirty]);
  useEffect(() => {
    mounted.current = true;
    let cancelled = false, timer: ReturnType<typeof setTimeout>;
    const clock = setInterval(() => setNow(Date.now()), 300);
    async function poll() {
      const epoch = statusEpoch.current;
      try {
        const result = await api<VehicleState>('vehicle/status');
        if (!cancelled && !mutationPending.current && epoch === statusEpoch.current) {
          if (target.current && ((lastConnected.current !== null && lastConnected.current !== result.connected) || !vehicleIdentityMatches(target.current, result.identity))) setBindingInvalid(true);
          lastConnected.current = result.connected; setState(result); setReceivedAt(Date.now()); setNow(Date.now()); setStatusError('');
        }
      } catch (e) {
        if (!cancelled && !mutationPending.current && epoch === statusEpoch.current) { if (target.current) setBindingInvalid(true); lastConnected.current = false; setState(null); setStatusError((e as Error).message); }
      }
      if (!cancelled) timer = setTimeout(poll, 800);
    }
    void poll();
    return () => { cancelled = true; mounted.current = false; ++profileEpoch.current; clearTimeout(timer); clearInterval(clock); };
  }, []);
  function accept(result: VehicleProfileResult) {
    const issue = vehicleProfileIssue(result.profile);
    if (issue) throw Error(issue);
    if (!result.connected || !result.identity?.instance_id) throw Error('Jetson に接続してから設定を再取得してください。');
    setDraft(vehicleDraft(result.profile)); setPersisted(result.profile); setCadReference(vehicleCadReference(result.cad_reference)); setRevision(result.revision); setPath(result.path); setSaved(result.saved);
    target.current = result.identity; lastConnected.current = true; setIdentity(result.identity); setBindingInvalid(false); setError('');
  }
  async function read() {
    const epoch = ++profileEpoch.current;
    const result = await api<VehicleProfileResult>('vehicle/profile');
    if (mounted.current && epoch === profileEpoch.current) { ++statusEpoch.current; accept(result); }
  }
  useEffect(() => { void read().catch(e => { if (mounted.current) setError((e as Error).message); }); }, []);
  async function run(action: () => Promise<void>) {
    setBusy(true);
    try { await action(); }
    catch (e) { if (mounted.current) { setError((e as Error).message); notify((e as Error).message, true); } }
    finally { if (mounted.current) setBusy(false); }
  }
  async function save() {
    if (!converted || converted.issue || !matching || !responseFresh) return;
    mutationPending.current = true; ++statusEpoch.current;
    try {
      const result = await api<VehicleProfileResult>('vehicle/profile', { profile: converted.profile, revision, instance_id: identity?.instance_id }, 'PUT');
      if (mounted.current) { accept(result); notify('車両設定を保存しました。静的 TF の反映には Jetson 側のノード再起動が必要です'); }
    } finally { mutationPending.current = false; ++statusEpoch.current; }
  }
  function editDimension(key: VehicleDimension, text: string) {
    setDraft(current => current && ({ ...current, dimensions: { ...current.dimensions, [key]: { value_mm: text, source: text.trim() ? 'manual' : 'unmeasured' } } })); setError('');
  }
  function editPose(kind: 'translation_mm' | 'rotation_rpy_deg', key: string, text: string) {
    setDraft(current => {
      if (!current) return null;
      const camera = { ...current.camera, [kind]: { ...current.camera[kind], [key]: text } };
      camera.source = [...Object.values(camera.translation_mm), ...Object.values(camera.rotation_rpy_deg)].some(value => value.trim()) ? 'manual' : 'unmeasured';
      return { ...current, camera };
    }); setError('');
  }
  function highlight(key: MeasurementKey | null) {
    const selected = key ?? focusedMeasurement.current;
    setActiveMeasurement(selected);
    if (!selected) return;
    if (selected.startsWith('camera_')) { setDiagramMode('pose'); if (selected === 'camera_y') setDiagramSide(false); if (selected === 'camera_z' || selected === 'camera_pitch') setDiagramSide(true); }
    else { setDiagramMode('guide'); setDiagramSide(selected === 'body_height'); }
  }
  function selectMeasurement(key: MeasurementKey) { highlight(key); measurementInputs.current[key]?.focus(); measurementInputs.current[key]?.scrollIntoView({ block: 'nearest', behavior: 'smooth' }); }
  const fieldFocus = (key: MeasurementKey) => ({ onMouseEnter: () => highlight(key), onMouseLeave: () => highlight(null), onFocus: () => { focusedMeasurement.current = key; highlight(key); }, onBlur: () => { focusedMeasurement.current = null; highlight(null); } });

  return <div className="content-page vehicle-page"><div className="page-intro"><div><span className="eyebrow">VEHICLE GEOMETRY</span><h2>実測値から、車体とセンサの配置を整える。</h2><p>後輪車軸中心を基準に mm・度で入力します。未測定の項目は空欄のまま保存できます。</p></div><span className="outline-chip">BASE_LINK</span></div>
    <section className="content-card vehicle-state-card"><div className="card-heading"><h3>設定と適用状態</h3><Tag>{dirty ? '未保存の編集あり' : saved ? '保存済み' : '未保存'}</Tag></div><div className="vehicle-state-body">
      <div className="vehicle-status-grid"><div><span>Jetson 接続</span><strong>{responseFresh ? '接続中' : '未接続／未確認'}</strong><small>{state?.identity?.hostname || '「データ転送」で接続先を設定'}</small></div><div><span>静的 TF</span><strong>{applied ? '適用済み' : fresh && saved ? '再起動待ち' : '適用未確認'}</strong><small>{fresh ? state?.status?.tf_published ? 'ノードから配信状態を受信' : 'TF 配信は未確認' : '新しいノード状態を待機'}</small></div><div><span>実測・入力</span><strong>車体寸法 {dimensionCount} / 7</strong><small>{profile?.camera.source === 'unmeasured' ? 'カメラ配置は未測定' : `カメラ配置: ${vehicleSourceText(profile?.camera.source ?? 'unmeasured')}`}</small></div></div>
      <p className="small-note">保存は Jetson の設定ファイルを更新します。静的 TF ノードの再起動後に反映されます。この画面では再起動・走行操作を行いません。</p>
      {(error || statusError) && <p className="inline-note" role="alert">{error || statusError}</p>}
      {bindingInvalid && <p className="inline-note">Jetson の接続先または起動状態が変わりました。編集中の値は保持しています。再取得してから保存してください。</p>}
      <div className="button-row wrapping"><button className="button primary" disabled={!draft || busy || !matching || !responseFresh || !!converted?.issue || (!dirty && saved)} onClick={() => void run(save)}><Icon name="save" size={15}/>車両設定を保存</button><button className="button" disabled={busy} onClick={() => { if (dirty && !window.confirm('未保存の車両設定を破棄し、Jetson から再取得しますか？')) return; void run(read); }}>車両設定を再取得</button></div>
      {converted?.issue && <p className="inline-note" role="alert">{converted.issue}</p>}
      {path && <details><summary>保存先と基準フレーム</summary><p className="small-note">{path}</p><p className="small-note">base_link → rear_axle / front_axle / camera_link</p></details>}
    </div></section>
    <div className="vehicle-origin-note"><Icon name="info" size={18}/><div><strong>原点: 後輪車軸の左右回転中心の中点</strong><p>+X は前、+Y は左、+Z は上。カメラの位置はこの原点から camera_link の原点までを測定します。空欄を 0 として扱いません。</p></div></div>
    <div className="vehicle-editor-layout">{profile && <section className="content-card vehicle-preview-card"><div className="card-heading"><h3>測定箇所と配置</h3><Tag>{diagramMode === 'guide' ? '模式図・縮尺なし' : '編集中の値'}</Tag></div><div className="vehicle-diagram-controls"><div className="tabs"><button className={diagramMode === 'guide' ? 'active' : ''} onClick={() => setDiagramMode('guide')}>測定箇所のガイド</button><button className={diagramMode === 'pose' ? 'active' : ''} onClick={() => setDiagramMode('pose')}>入力値の配置</button></div><div className="button-row"><button className={'button small' + (!diagramSide ? ' primary' : '')} onClick={() => setDiagramSide(false)}>上面 XY</button><button className={'button small' + (diagramSide ? ' primary' : '')} onClick={() => setDiagramSide(true)}>側面 XZ</button></div></div>{diagramMode === 'guide' ? <MeasurementGuide profile={profile} active={activeMeasurement} select={selectMeasurement} hover={highlight} side={diagramSide}/> : <div className="vehicle-projections"><VehicleProjection profile={profile} side={diagramSide} active={activeMeasurement} select={selectMeasurement} hover={highlight}/></div>}<p className="card-footnote">図は測定箇所と入力値の確認用です。実際に配信中の TF とは別です。姿勢は固定軸 RPY（X → Y → Z）、保存単位は m・rad です。</p></section>}
    {draft && <div className="vehicle-form-columns"><section className="content-card"><div className="card-heading"><h3>車体寸法</h3><Tag>mm</Tag></div><div className="vehicle-form-body">{VEHICLE_DIMENSIONS.map(key => <div className={'vehicle-measurement' + (activeMeasurement === key ? ' highlighted' : '')} key={key} {...fieldFocus(key)}><Field label={VEHICLE_LABELS[key].label} hint={VEHICLE_LABELS[key].hint}><div className="vehicle-unit-input"><input ref={element => { measurementInputs.current[key] = element; }} type="text" inputMode="decimal" aria-label={`車体 ${key} mm`} placeholder="未測定" disabled={busy} value={draft.dimensions[key].value_mm} onChange={e => editDimension(key, e.target.value)}/><span>mm</span></div></Field><small>{vehicleSourceText(draft.dimensions[key].source)}</small></div>)}<p className="small-note">車体全高は路面からの高さです。base_link の路面からの高さや、車体下端の位置には流用しません。</p></div></section>
      <section className="content-card"><div className="card-heading"><h3>D455 の取付配置</h3><Tag>base_link → camera_link</Tag></div><div className="vehicle-form-body"><label className="vehicle-camera-enable"><input type="checkbox" aria-label="カメラ取付 TF を有効にする" disabled={busy} checked={draft.camera.enabled} onChange={e => { setDraft({ ...draft, camera: { ...draft.camera, enabled: e.target.checked } }); setError(''); }}/>再起動後にカメラ取付 TF を配信する</label><p className="small-note">有効にする場合は位置・姿勢をすべて入力してください。配信しない状態では部分的な測定値も保存できます。</p><h4>位置</h4><div className="vehicle-pose-grid">{VEHICLE_XYZ.map(key => <div key={key} className={'vehicle-pose-measurement' + (activeMeasurement === `camera_${key}` ? ' highlighted' : '')} {...fieldFocus(`camera_${key}`)}><Field label={POSE_LABELS[key]}><div className="vehicle-unit-input"><input ref={element => { measurementInputs.current[`camera_${key}`] = element; }} type="text" inputMode="decimal" aria-label={`カメラ ${key} mm`} placeholder="未測定" disabled={busy} value={draft.camera.translation_mm[key]} onChange={e => editPose('translation_mm', key, e.target.value)}/><span>mm</span></div></Field></div>)}</div><h4>姿勢</h4><div className="vehicle-pose-grid">{VEHICLE_RPY.map(key => <div key={key} className={'vehicle-pose-measurement' + (activeMeasurement === `camera_${key}` ? ' highlighted' : '')} {...fieldFocus(`camera_${key}`)}><Field label={POSE_LABELS[key]}><div className="vehicle-unit-input"><input ref={element => { measurementInputs.current[`camera_${key}`] = element; }} type="text" inputMode="decimal" aria-label={`カメラ ${key} deg`} placeholder="未測定" disabled={busy} value={draft.camera.rotation_rpy_deg[key]} onChange={e => editPose('rotation_rpy_deg', key, e.target.value)}/><span>°</span></div></Field></div>)}</div><p className="small-note">配置の由来: {vehicleSourceText(draft.camera.source)}。D455 内部の各センサ・optical frame 間の TF は RealSense driver が管理します。</p></div></section></div>}</div>
    {cadReference && <section className="content-card vehicle-cad-reference"><div className="card-heading"><h3>CAD の参考寸法</h3><Tag>実測値とは別</Tag></div><div className="vehicle-form-body"><strong>印刷フレーム {VEHICLE_XYZ.map(key => Number((cadReference.axis_extent_m[key] * 1000).toPrecision(8))).join(' × ')} mm（CAD 軸 XYZ）</strong><p>{cadReference.caveat}</p><small>参照: {cadReference.path}</small><p className="small-note">車体寸法・カメラ配置には自動入力しません。参照値は保存対象外です。</p></div></section>}
  </div>;
}
