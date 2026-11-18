import { useEffect, useRef, useState } from 'react';
import { api } from './api';
import { Field, Icon, Tag } from './workspace_ui';
import { PROPO_DISPLAY_COUNTS, PROPO_ROLES, picoConnectionText, picoFaultText, picoPathText, propoAvailableChannels, propoConnectionMatches, propoDisplayCount, propoInputFresh, propoPosition, propoProfileIssue, propoPulseUs, propoRcValid, propoReasonText, propoResponseFresh, propoRoleInputCurrent, propoSourceMatches, propoTelemetryFresh, propoVisibleChannelCount, samePropoProfile } from './propo_profile';
import type { PropoBinding, PropoDisplayCount, PropoIdentity, PropoPosition, PropoProfile, PropoProfileResult, PropoRole, PropoState } from './propo_profile';

const ROLE_LABELS: Record<PropoRole, string> = { recording: '記録の開始・停止', auxiliary_1: '予備 1', auxiliary_2: '予備 2' };
const FIXED_LABELS = ['操舵', 'スロットル', 'プロポ／Jetson 切替'];
const DISPLAY_COUNT_KEY = 'atd.propo.display-channel-count';
const PWM_TICKS = [1000, 1250, 1500, 1750, 2000];
function positionLabel(role: PropoRole, position: PropoPosition | null): string {
  return position === 'unassigned' ? '未割当' : position === 'middle' ? '中間' : position === null ? '—'
    : role === 'recording' ? position === 'high' ? '記録開始側' : '記録停止側' : position === 'high' ? '高側' : '低側';
}
const rawText = (value: number | undefined) => Number.isFinite(value) ? String(value) : '—';

export function PropoSettingsPanel({ notify, onDirty }: {
  notify: (message: string, error?: boolean) => void; onDirty: (value: boolean) => void;
}) {
  const [profile, setProfile] = useState<PropoProfile | null>(null), [persisted, setPersisted] = useState<PropoProfile | null>(null);
  const [revision, setRevision] = useState(''), [saved, setSaved] = useState(false), [busy, setBusy] = useState(false);
  const [status, setStatus] = useState<PropoState | null>(null), [identity, setIdentity] = useState<PropoIdentity | null>(null);
  const [receivedAt, setReceivedAt] = useState(0), [now, setNow] = useState(Date.now());
  const [error, setError] = useState(''), [statusError, setStatusError] = useState('');
  const [bindingInvalid, setBindingInvalid] = useState(false);
  const [channels, setChannels] = useState<number[]>([]);
  const [preferredCount, setPreferredCount] = useState<PropoDisplayCount>(() => {
    try { return propoDisplayCount(window.localStorage.getItem(DISPLAY_COUNT_KEY)); }
    catch { return 6; }
  });
  const mounted = useRef(true), statusEpoch = useRef(0), profileEpoch = useRef(0), mutationPending = useRef(false);
  const targetIdentity = useRef<PropoIdentity | null>(null), lastConnected = useRef<boolean | null>(null);
  const dirty = !!profile && !samePropoProfile(profile, persisted);
  const matching = !bindingInvalid && propoConnectionMatches(identity, status?.identity);
  const responseFresh = propoResponseFresh(status, receivedAt, now);
  const inputFresh = propoInputFresh(status, receivedAt, now);
  const telemetryFresh = propoTelemetryFresh(status, receivedAt, now), validRc = propoRcValid(status, receivedAt, now);
  const sourceMatches = propoSourceMatches(status);
  const roleInputCurrent = propoRoleInputCurrent(status, receivedAt, now);
  const configuring = matching && inputFresh && status?.input?.configuring === true;
  const applied = saved && matching && inputFresh && status?.input?.profile_valid === true
    && samePropoProfile(persisted, status.input.profile);
  const issue = profile ? propoProfileIssue(profile, channels) : '';
  const board = responseFresh ? status?.board : null, telemetry = telemetryFresh ? board?.telemetry : null;
  const connectionNote = picoConnectionText(board?.reason);
  const rawChannels = telemetry?.channels;
  const activeProfile = matching && inputFresh ? status?.input?.profile : null;
  const minimumCount = propoVisibleChannelCount(6, profile, persisted, activeProfile);
  const visibleCount = propoVisibleChannelCount(preferredCount, profile, persisted, activeProfile);

  useEffect(() => { onDirty(dirty); }, [dirty, onDirty]);
  useEffect(() => {
    try { window.localStorage.setItem(DISPLAY_COUNT_KEY, String(preferredCount)); }
    catch { /* 保存できないブラウザでも、この画面内では選択を保持する。 */ }
  }, [preferredCount]);
  useEffect(() => {
    mounted.current = true;
    let cancelled = false, timer: ReturnType<typeof setTimeout>;
    const clock = setInterval(() => setNow(Date.now()), 100);
    async function poll() {
      const requestEpoch = statusEpoch.current;
      try {
        const result = await api<PropoState>('propo/status');
        if (!cancelled && !mutationPending.current && requestEpoch === statusEpoch.current) {
          if (targetIdentity.current && ((lastConnected.current !== null && lastConnected.current !== result.connected)
              || !propoConnectionMatches(targetIdentity.current, result.identity))) setBindingInvalid(true);
          lastConnected.current = result.connected;
          setStatus(result); setReceivedAt(Date.now()); setNow(Date.now()); setStatusError('');
        }
      } catch (e) {
        if (!cancelled && !mutationPending.current && requestEpoch === statusEpoch.current) {
          if (targetIdentity.current) setBindingInvalid(true);
          lastConnected.current = false;
          setStatus(null); setStatusError((e as Error).message);
        }
      }
      if (!cancelled) timer = setTimeout(poll, 200);
    }
    void poll();
    return () => { cancelled = true; mounted.current = false; ++profileEpoch.current; clearTimeout(timer); clearInterval(clock); };
  }, []);

  async function readProfile() {
    const requestEpoch = ++profileEpoch.current;
    const result = await api<PropoProfileResult>('propo/profile');
    if (!mounted.current || requestEpoch !== profileEpoch.current) return;
    const available = propoAvailableChannels(result.channels);
    const issue = propoProfileIssue(result.profile, available);
    if (issue) throw Error(issue);
    if (!available.length) throw Error('接続先の役割チャンネルを取得できません。接続してから再取得してください。');
    if (!result.connected || !result.identity?.instance_id) throw Error('Jetson の接続状態を確認できません。接続してから再取得してください。');
    ++statusEpoch.current;
    targetIdentity.current = result.identity; lastConnected.current = true;
    setBindingInvalid(false);
    setProfile(result.profile); setPersisted(result.profile); setRevision(result.revision);
    setChannels(available);
    setSaved(result.saved); setIdentity(result.identity ?? null); setError('');
  }
  useEffect(() => { void readProfile().catch(e => { if (mounted.current) setError((e as Error).message); }); }, []);

  async function run(action: () => Promise<void>) {
    setBusy(true);
    try { await action(); }
    catch (e) { if (mounted.current) { setError((e as Error).message); notify((e as Error).message, true); } }
    finally { if (mounted.current) setBusy(false); }
  }
  async function mutate<T>(path: string, body: object, method?: string): Promise<T> {
    mutationPending.current = true; ++statusEpoch.current;
    // 設定操作中も直近の基板表示を保ち、役割状態だけを未確認に戻す。
    setStatus(current => current && ({ ...current, input: null, ready: false }));
    try { return await api<T>(path, { ...body, instance_id: identity?.instance_id }, method); }
    finally {
      mutationPending.current = false; ++statusEpoch.current;
      if (mounted.current) setStatus(current => current && ({ ...current, input: null, ready: false }));
    }
  }
  async function service(path: 'configure' | 'reload', body: object) {
    const result = await mutate<{ applied: boolean; message?: string }>('propo/' + path, body);
    if (!result.applied) throw Error(result.message || 'Jetsonでの反映を確認できません。');
    if (mounted.current) setError('');
  }
  function edit(role: PropoRole, patch: Partial<PropoBinding>) {
    setProfile(current => current && ({ ...current, roles: { ...current.roles, [role]: { ...current.roles[role], ...patch } } }));
    setError('');
  }

  const usbConnected = responseFresh && board?.stale === false && board.connected;
  const receivedAge = telemetry ? (telemetry.age_s ?? 0) + Math.max(0, now - receivedAt) / 1000 : null;
  const flags = telemetry?.sbus_flags;
  const canResume = configuring && applied && !dirty && !busy && roleInputCurrent;

  return <section className="content-card input-config" aria-label="プロポの入力と役割" style={{ marginTop: 22 }}>
    <div className="card-heading"><h3>プロポの入力と役割</h3><Tag>{!responseFresh ? 'Jetson 未接続' : telemetryFresh ? validRc ? '入力を受信中' : '入力は無効' : usbConnected ? 'USB 接続・入力待ち' : 'Pico 未接続'}</Tag></div>
    <div className="input-config-body">
      <p>Jetson に USB 接続した Pico のチャンネル入力と基板状態を確認します。基本の走行操作はプロポと Pico で完結します。</p>
      {(!responseFresh || statusError) && <p className="inline-note">{statusError || status?.message || 'Jetson の監視接続を待っています。「データ転送」の接続先と、Jetson の入力起動を確認してください。'}</p>}
      <div className="controller-summary">
        <span className="controller-symbol"><Icon name="live" size={28}/></span>
        <div><strong>{!responseFresh ? 'USB 状態未確認' : usbConnected ? 'Pico 接続中' : 'Pico 接続待ち'}</strong><p>{board?.serial ? `Serial: ${board.serial}` : 'USB デバイス未確認'}</p><small>{board?.path || '接続先は Jetson'}</small></div>
        <div className="controller-state"><span className={'connection-indicator ' + (validRc ? 'online' : '')}><i/>{validRc ? 'プロポ信号は有効' : telemetryFresh ? 'プロポ信号を確認してください' : '新しい信号なし'}</span><small>{receivedAge === null ? 'サンプル未受信' : `${receivedAge.toFixed(2)} 秒前のサンプル`}</small></div>
      </div>
      {connectionNote && <p className="inline-note">{connectionNote}</p>}
      <div className="propo-monitor-options"><label>表示チャンネル数<select aria-label="プロポ 表示チャンネル数" value={visibleCount} onChange={e => setPreferredCount(propoDisplayCount(e.target.value))}>{PROPO_DISPLAY_COUNTS.map(count => <option key={count} value={count} disabled={count < minimumCount}>{count}ch</option>)}</select></label><small>{visibleCount > preferredCount ? `割り当て済みのチャンネルを含むため ${visibleCount}ch まで表示しています。` : '表示の選択はこの Notebook に保存します。'}</small></div>
      <div className="mapping-scroll"><table className="mapping-table propo-channel-table"><thead><tr><th>入力</th><th>PWM 相当（µs）</th><th>S.BUS raw</th><th>配置</th></tr></thead><tbody>
        {Array.from({ length: visibleCount }, (_, index) => index + 1).map(channel => {
          const value = rawChannels?.[channel - 1];
          const pulse = propoPulseUs(value);
          const role = profile && PROPO_ROLES.find(name => profile.roles[name].channel === channel);
          return <tr key={channel}><td><strong>CH{channel}</strong></td><td><div className="propo-pulse-meter"><strong>{pulse === null ? '—' : `${pulse} µs`}</strong><div className={'propo-pulse-track' + (!validRc ? ' signal-invalid' : '')} role="img" aria-label={`CH${channel} PWM 相当 ${pulse === null ? '未受信' : `${pulse} µs`}（参考値）`}>
            {pulse !== null && <i className="propo-pulse-fill" style={{ width: `${(pulse - 1000) / 10}%` }}/>}
            {PWM_TICKS.map(tick => <span key={tick} className={'propo-pulse-tick' + (tick === 1500 ? ' neutral' : '')} style={{ left: `${(tick - 1000) / 10}%` }}/>) }
            {pulse !== null && <i className="propo-pulse-pointer" style={{ left: `${(pulse - 1000) / 10}%` }}/>}
          </div><div className="propo-pulse-scale" aria-hidden="true">{PWM_TICKS.map(tick => <span key={tick} style={{ left: `${(tick - 1000) / 10}%` }}>{tick}</span>)}</div></div></td><td>{pulse === null ? '—' : value}</td><td>{channel <= 3 ? `${FIXED_LABELS[channel - 1]}（Pico 固定）` : role ? `${ROLE_LABELS[role]}${dirty ? '（編集中）' : ''}` : '未割当'}</td></tr>;
        })}
      </tbody></table></div>
      <p className="small-note">PWM 相当は S.BUS raw（0 ～ 2047）から共通の基準で換算した参考値です。1000 ～ 2000 µs、1500 µs を中間の目安として表示します。Pico の CH 別の反転・校正は反映しておらず、実際のサーボ／ESC 出力の測定値ではありません。</p>
      <p className="small-note">CH1 は操舵、CH2 はスロットル、CH3 はプロポ／Jetson 切替です。Pico 側の固定配置はこの画面では変更不可です。無効な信号の受信値は監視用に表示し、操作要求には利用しません。</p>
      <div className="mapping-scroll"><table className="mapping-table"><thead><tr><th>基板状態</th><th>値</th><th>受信状態</th><th>値</th></tr></thead><tbody>
        <tr><td>基板が選択した操作経路</td><td>{picoPathText(telemetry?.active_path)}</td><td>S.BUS 受信</td><td>{telemetry ? telemetry.sbus_fresh ? '新しい信号あり' : '期限切れ／未受信' : '—'}</td></tr>
        <tr><td>BEC 電圧</td><td>{telemetry && Number.isFinite(telemetry.vbec_mv) ? `${(telemetry.vbec_mv! / 1000).toFixed(2)} V` : '—'}</td><td>frame lost</td><td>{telemetry && Number.isInteger(flags) ? (flags! & 4) ? 'あり' : 'なし' : '—'}</td></tr>
        <tr><td>電源状態</td><td>{typeof telemetry?.power_ready === 'boolean' ? telemetry.power_ready ? '準備完了' : '準備未完了' : '—'}</td><td>failsafe</td><td>{telemetry && Number.isInteger(flags) ? (flags! & 8) ? 'あり' : 'なし' : '—'}</td></tr>
        <tr><td>基板の異常表示</td><td colSpan={3}>{picoFaultText(telemetry?.fault_bits)}</td></tr>
      </tbody></table></div>
      <details><summary>受信カウンタ</summary><div className="field-grid">
        {[['受信フレーム', telemetry?.sbus_frames], ['frame lost 累積', telemetry?.sbus_lost_frames], ['failsafe 累積', telemetry?.sbus_failsafe_frames], ['UART エラー', telemetry?.uart_errors], ['RPM エッジ数', telemetry?.rpm_edges], ['監視送信の破棄', telemetry?.tx1_dropped], ['監視要求の拒否', telemetry?.rx1_rejected]].map(([label, value]) => <div key={String(label)}><small>{label}</small><p>{rawText(typeof value === 'number' ? value : undefined)}</p></div>)}
      </div></details>

      <div className="card-heading" style={{ marginTop: 22 }}><h3>Jetson 側の役割配置</h3><Tag>{dirty ? '未保存' : !saved ? '未設定' : applied ? '反映済み' : inputFresh && matching ? '未反映' : '反映未確認'}</Tag></div>
      {error && <p className="inline-note" role="alert">{error}</p>}
      {profile && (bindingInvalid || responseFresh && !matching) && <p className="inline-note">接続先または Jetson の接続状態が変わりました。編集中の内容は保持しています。設定を再取得してから操作してください。</p>}
      <div className="config-mode-bar"><Icon name="info" size={17}/><p>{configuring ? '役割設定中は記録などの操作要求だけを止めます。プロポによる走行操作は変わりません。' : 'CH4 ～ CH16 の利用可能な入力を記録操作や予備の役割へ割り当てます。設定変更は Pico の走行用配置には影響しません。'}</p></div>
      <div className="button-row wrapping">
        <button className="button" disabled={!matching || !inputFresh || busy || configuring} onClick={() => void run(async () => { await service('configure', { enabled: true }); notify('プロポの役割設定を開始しました'); })}>プロポ役割設定を開始</button>
        <button className="button primary" disabled={!profile || !configuring || busy || !!issue || (!dirty && saved)} onClick={() => void run(async () => {
          const result = await mutate<PropoProfileResult>('propo/profile', { profile, revision }, 'PUT');
          if (mounted.current) { setProfile(result.profile); setPersisted(result.profile); setRevision(result.revision); setSaved(result.saved); setError(''); }
          notify('プロポの役割を保存しました。「再読込」で反映してください');
        })}><Icon name="save" size={15}/>プロポ役割を保存</button>
        <button className="button" disabled={!configuring || !saved || dirty || busy} onClick={() => void run(async () => { await service('reload', {}); notify('プロポの役割を再読込しました。操作要求は停止中です'); })}>プロポ役割を再読込</button>
        <button className="button" disabled={!canResume} onClick={() => void run(async () => { await service('configure', { enabled: false }); notify('プロポの操作要求を再開しました。現在のスイッチ位置を確認してから変化を検出します'); })}>プロポ操作要求を再開</button>
        <button className="button" disabled={busy} onClick={() => {
          if (dirty && !window.confirm('未保存のプロポ設定を破棄して、Jetsonから再取得しますか？')) return;
          void run(readProfile);
        }}>プロポ役割を再取得</button>
      </div>
      {profile && <>
        <div className="field-grid"><Field label="位置の確認時間（ms）"><input type="number" aria-label="プロポ 位置の確認時間" min="0" max="1000" step="10" disabled={!configuring || busy} value={Number.isFinite(profile.debounce_ms) ? profile.debounce_ms : ''} onChange={e => { setProfile({ ...profile, debounce_ms: e.target.value === '' ? Number.NaN : Number(e.target.value) }); setError(''); }}/></Field></div>
        <div className="mapping-scroll"><table className="mapping-table"><thead><tr><th>役割</th><th>チャンネル</th><th>低側／高側の閾値（raw）</th><th>反転</th><th>編集中の raw・位置</th><th>Jetson の適用中配置</th></tr></thead><tbody>
          {PROPO_ROLES.map(role => {
            const binding = profile.roles[role];
            const value = binding.channel && validRc ? rawChannels?.[binding.channel - 1] ?? null : null;
            const position = propoPosition(binding, value);
            const actual = roleInputCurrent && matching ? status?.input?.roles?.[role] : null;
            return <tr key={role}><td><strong>{ROLE_LABELS[role]}</strong><small>{role === 'recording' ? '高側で開始要求／低側で停止要求' : '現在は状態表示のみ'}</small></td>
              <td><select aria-label={`${role} プロポチャンネル`} disabled={!configuring || busy} value={binding.channel} onChange={e => edit(role, { channel: Number(e.target.value) })}><option value={0}>未割当</option>{channels.map(channel => <option value={channel} key={channel}>CH{channel}</option>)}</select></td>
              <td><div className="calibration-inputs">{(['low_threshold', 'high_threshold'] as const).map(key => <label key={key}><small>{key === 'low_threshold' ? '低側以下' : '高側以上'}</small><input type="number" aria-label={`${role} ${key}`} min="0" max="2047" step="1" disabled={!configuring || busy} value={Number.isFinite(binding[key]) ? binding[key] : ''} onChange={e => edit(role, { [key]: e.target.value === '' ? Number.NaN : Number(e.target.value) })}/><button className="text-button" disabled={!configuring || busy || value === null} onClick={() => value !== null && edit(role, { [key]: value })}>現在値を取得</button></label>)}</div></td>
              <td><label style={{ whiteSpace: 'nowrap' }}><input type="checkbox" style={{ width: 'auto' }} aria-label={`${role} 位置反転`} disabled={!configuring || busy} checked={binding.inverted} onChange={e => edit(role, { inverted: e.target.checked })}/>高低を反転</label></td>
              <td>{value === null ? '—' : value}<br/><small>{positionLabel(role, position)}</small></td>
              <td>{actual ? actual.channel ? `CH${actual.channel} · ${rawText(actual.raw ?? undefined)}` : '未割当' : '—'}<br/><small>{actual ? positionLabel(role, actual.position === 'unknown' ? null : actual.position) : '新しい入力を待機'}</small></td>
            </tr>;
          })}
        </tbody></table></div>
        {issue && <p className="inline-note" role="alert">{issue}</p>}
        {saved && inputFresh && matching && !applied && <p className="small-note">保存した配置はまだ Jetson で反映されていません。</p>}
        {inputFresh && matching && status?.input && (!status.input.ready || sourceMatches) && <p className="small-note">役割の状態: {propoReasonText(status.input.reason)}</p>}
        {inputFresh && status?.input?.ready && !sourceMatches && <p className="small-note">受信元が切り替わりました。新しい Pico 入力に対応する役割状態を待っています。</p>}
      </>}
      <p className="small-note">役割の閾値・現在値の取得は S.BUS raw（0 ～ 2047）を使います。閾値の間は中間位置です。「raw・位置」は編集中の設定による確認表示で、確認時間を経た実際のイベントとは別です。再接続や設定の再開だけでは記録を開始しません。</p>
      <p className="small-note">記録チャンネルは開始・停止の要求を生成します。実際の rosbag 記録処理は今後接続します。予備2枠に機能は割り当てていません。</p>
    </div>
  </section>;
}
