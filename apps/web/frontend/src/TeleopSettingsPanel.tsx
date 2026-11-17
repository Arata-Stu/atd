import { useEffect, useRef, useState } from 'react';
import { api } from './api';
import { Icon, Tag } from './workspace_ui';
import { sameTeleopProfile, teleopConnectionMatches, teleopFresh, teleopPreview, teleopProfileIssue, teleopReasonText, teleopRolesAssigned } from './teleop_profile';
import type { AxisRole, BridgeIdentity, ButtonRole, JoySample, TeleopProfile, TeleopProfileResult, TeleopState } from './teleop_profile';

const AXIS_LABELS: Record<string, string> = { left_x: '左スティック X', left_y: '左スティック Y', right_x: '右スティック X', right_y: '右スティック Y', l2: 'L2', r2: 'R2', dpad_x: '十字キー X', dpad_y: '十字キー Y' };
const BUTTON_LABELS: Record<string, string> = { cross: '×', circle: '○', triangle: '△', square: '□', l1: 'L1', r1: 'R1', l2: 'L2', r2: 'R2', create: 'Create', options: 'Options', ps: 'PS', l3: 'L3', r3: 'R3', dpad_up: '↑', dpad_down: '↓', dpad_left: '←', dpad_right: '→' };
const ROLE_LABELS: Record<ButtonRole, string> = { enable: '運転許可の要求（押下中）', record_start: '記録開始の要求', record_stop: '記録終了の要求' };

export function TeleopSettingsPanel({ normalized, inputReady, inputReceivedAt, notify, onDirty }: {
  normalized: JoySample | null; inputReady: boolean; inputReceivedAt: number;
  notify: (message: string, error?: boolean) => void; onDirty: (value: boolean) => void;
}) {
  const [profile, setProfile] = useState<TeleopProfile | null>(null), [persisted, setPersisted] = useState<TeleopProfile | null>(null);
  const [revision, setRevision] = useState(''), [saved, setSaved] = useState(false), [busy, setBusy] = useState(false);
  const [axes, setAxes] = useState<string[]>([]), [buttons, setButtons] = useState<string[]>([]);
  const [status, setStatus] = useState<TeleopState | null>(null), [identity, setIdentity] = useState<BridgeIdentity | null>(null);
  const [receivedAt, setReceivedAt] = useState(0), [now, setNow] = useState(Date.now()), [error, setError] = useState(''), [statusError, setStatusError] = useState('');
  const mounted = useRef(true);
  const statusEpoch = useRef(0), mutationPending = useRef(false);
  const dirty = !!profile && !sameTeleopProfile(profile, persisted);
  const fresh = teleopFresh(status, receivedAt, now), matching = teleopConnectionMatches(identity, status?.identity);
  const configuring = fresh && matching && status?.status?.configuring === true;
  const applied = saved && fresh && matching && status?.status?.profile_valid === true && sameTeleopProfile(persisted, status.status.profile);
  const issue = profile ? teleopProfileIssue(profile) : '', assigned = !!profile && teleopRolesAssigned(profile);
  const inputFresh = inputReady && inputReceivedAt > 0 && Math.max(0, now - inputReceivedAt) <= 3000;
  const preview = profile ? teleopPreview(profile, normalized, inputFresh) : null;

  useEffect(() => { onDirty(dirty); }, [dirty, onDirty]);
  useEffect(() => {
    mounted.current = true;
    let cancelled = false, timer: ReturnType<typeof setTimeout>;
    const clock = setInterval(() => setNow(Date.now()), 500);
    async function poll() {
      const requestEpoch = statusEpoch.current;
      try {
        const result = await api<TeleopState>('teleop/status');
        if (!cancelled && !mutationPending.current && requestEpoch === statusEpoch.current) { setStatus(result); setReceivedAt(Date.now()); setNow(Date.now()); setStatusError(''); }
      } catch (e) { if (!cancelled && !mutationPending.current && requestEpoch === statusEpoch.current) { setStatus(null); setStatusError((e as Error).message); } }
      if (!cancelled) timer = setTimeout(poll, 800);
    }
    void poll();
    return () => { cancelled = true; mounted.current = false; clearTimeout(timer); clearInterval(clock); };
  }, []);

  async function readProfile() {
    const result = await api<TeleopProfileResult>('teleop/profile');
    if (!mounted.current) return;
    setProfile(result.profile); setPersisted(result.profile); setRevision(result.revision);
    setSaved(result.saved); setAxes(result.axes); setButtons(result.buttons); setIdentity(result.identity ?? null); setError('');
  }
  useEffect(() => { void readProfile().catch(e => { if (mounted.current) setError((e as Error).message); }); }, []);

  async function run(action: () => Promise<void>) {
    setBusy(true);
    try { await action(); } catch (e) { if (mounted.current) { setError((e as Error).message); notify((e as Error).message, true); } }
    finally { if (mounted.current) setBusy(false); }
  }
  async function mutate<T>(path: string, body: object, method?: string): Promise<T> {
    mutationPending.current = true;
    ++statusEpoch.current;
    setStatus(null);
    try { return await api<T>(path, { ...body, instance_id: identity?.instance_id }, method); }
    finally {
      mutationPending.current = false;
      // 変更要求より前・処理中に開始した状態取得を採用しない。
      ++statusEpoch.current;
      if (mounted.current) setStatus(null);
    }
  }
  async function service(path: 'configure' | 'reload', body: object) {
    const result = await mutate<{ applied: boolean; message?: string }>('teleop/' + path, body);
    if (!result.applied) throw Error(result.message || 'Jetsonでの反映を確認できません。');
    if (mounted.current) { setStatus(null); setError(''); }
  }
  function editAxis(role: AxisRole, patch: Partial<TeleopProfile['axes'][AxisRole]>) {
    setProfile(current => current && ({ ...current, axes: { ...current.axes, [role]: { ...current.axes[role], ...patch } } }));
  }
  function editButton(role: ButtonRole, name: string) {
    setProfile(current => current && ({ ...current, buttons: { ...current.buttons, [role]: name } }));
  }
  const selectOptions = (names: string[], labels: Record<string, string>) => <><option value="">未割当</option>{names.map(name => <option value={name} key={name}>{labels[name] || name} ({name})</option>)}</>;

  return <section className="content-card input-config" aria-label="運転操作の割り当て" style={{ marginTop: 22 }}>
    <div className="card-heading"><h3>運転操作の割り当て</h3><Tag>{dirty ? '未保存' : !saved ? '未設定' : applied ? '反映済み' : fresh && matching ? '未反映' : '反映未確認'}</Tag></div>
    <div className="input-config-body">
      <p>標準軸・ボタンに、操舵・スロットル・記録操作の役割を割り当てます。設定はJetsonに保存します。</p>
      {(error || !fresh) && <p className="inline-note">{error || statusError || status?.message || 'Jetsonの操作設定状態を受信していません。入力を起動し、接続設定を確認してください。'}</p>}
      {profile && status?.connected && !matching && <p className="inline-note">接続先またはJetsonの起動状態が変わりました。編集中の内容は保持しています。設定を再取得してから操作してください。</p>}
      <div className="config-mode-bar"><Icon name="info" size={17}/><p>{configuring ? '設定中は運転指令を無効にします。保存・再読込後、全入力を離して指令出力を再開します。' : '割り当てを変更するには「役割設定を開始」を選びます。指令出力の再開は走行許可とは別の操作です。'}</p></div>
      <div className="button-row wrapping">
        <button className="button" disabled={!fresh || !matching || busy || configuring} onClick={() => void run(async () => { await service('configure', { enabled: true }); notify('役割設定モードを要求しました'); })}>役割設定を開始</button>
        <button className="button primary" disabled={!profile || !configuring || busy || !!issue || (!dirty && saved)} onClick={() => void run(async () => {
          const result = await mutate<TeleopProfileResult>('teleop/profile', { profile, revision }, 'PUT');
          if (mounted.current) { setProfile(result.profile); setPersisted(result.profile); setRevision(result.revision); setSaved(result.saved); setError(''); }
          notify('役割を保存しました。「役割を再読込」でJetsonに反映してください');
        })}><Icon name="save" size={15}/>役割を保存</button>
        <button className="button" disabled={!configuring || !saved || dirty || busy} onClick={() => void run(async () => { await service('reload', {}); notify('役割を再読込しました。全入力を離して指令出力を再開できます'); })}>役割を再読込</button>
        <button className="button" disabled={!configuring || !applied || dirty || busy || !inputFresh || !assigned} onClick={() => void run(async () => { await service('configure', { enabled: false }); notify('指令出力の再開を要求しました。走行許可は別途必要です'); })}>指令出力を再開</button>
        <button className="button" disabled={busy} onClick={() => {
          if (dirty && !window.confirm('未保存の役割設定を破棄して、Jetsonから再取得しますか？')) return;
          void run(readProfile);
        }}>役割を再取得</button>
      </div>
      {saved && fresh && matching && !applied && <p className="small-note">保存した設定と、Jetsonで読み込まれた設定が一致していません。</p>}
      {profile && <>
        <div className="mapping-scroll"><table className="mapping-table"><thead><tr><th>役割</th><th>主入力</th><th>差し引く入力</th><th>倍率・反転</th><th>編集内容のプレビュー</th></tr></thead><tbody>
          {(['steering', 'throttle'] as const).map(role => <tr key={role}>
            <td><strong>{role === 'steering' ? '操舵' : 'スロットル'}</strong><small>{role === 'steering' ? '＋ 左 / − 右' : '＋ 前進要求 / − 後退要求'}</small></td>
            <td><select aria-label={`${role} 主入力`} disabled={!configuring || busy} value={profile.axes[role].axis} onChange={e => editAxis(role, { axis: e.target.value })}>{selectOptions(axes, AXIS_LABELS)}</select></td>
            <td><select aria-label={`${role} 差し引く入力`} disabled={!configuring || busy} value={profile.axes[role].subtract_axis} onChange={e => editAxis(role, { subtract_axis: e.target.value })}>{selectOptions(axes, AXIS_LABELS)}</select></td>
            <td><input type="number" aria-label={`${role} 倍率`} min="-1" max="1" step="0.05" disabled={!configuring || busy} value={Number.isFinite(profile.axes[role].scale) ? profile.axes[role].scale : ''} onChange={e => editAxis(role, { scale: e.target.value === '' ? Number.NaN : Number(e.target.value) })}/><button className="text-button" disabled={!configuring || busy || !Number.isFinite(profile.axes[role].scale)} onClick={() => editAxis(role, { scale: -profile.axes[role].scale })}>反転</button></td>
            <td>{preview?.[role]?.toFixed(2) ?? '—'}</td>
          </tr>)}
        </tbody></table></div>
        <p className="small-note">値は（主入力 − 差し引く入力）× 倍率を −1 ～ 1 に制限します。片方の入力だけでも割り当てられます。</p>
        <div className="mapping-scroll"><table className="mapping-table"><thead><tr><th>ボタンの役割</th><th>標準ボタン</th></tr></thead><tbody>
          {(['enable', 'record_start', 'record_stop'] as const).map(role => <tr key={role}><td><strong>{ROLE_LABELS[role]}</strong></td><td><select aria-label={`${role} 標準ボタン`} disabled={!configuring || busy} value={profile.buttons[role]} onChange={e => editButton(role, e.target.value)}>{selectOptions(buttons, BUTTON_LABELS)}</select></td></tr>)}
        </tbody></table></div>
        {issue && <p className="inline-note" role="alert">{issue}</p>}
        {!assigned && <p className="small-note">操舵とスロットルを両方割り当てるまで、有効な運転指令は出力しません。</p>}
        <p className="small-note">プレビューは現在の入力と編集中の設定による計算値です。Jetsonの実際の出力とは別です。入力設定中・未接続時は表示しません。</p>
        {fresh && matching && status?.status?.reason && <p className="small-note">Jetsonの状態: {teleopReasonText(status.status.reason)}</p>}
      </>}
      <p className="small-note">基板への送信とrosbag記録は、この運転指令・操作要求に後から接続します。</p>
    </div>
  </section>;
}
