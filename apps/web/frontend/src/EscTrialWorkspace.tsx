import { useEffect, useRef, useState } from 'react';
import { api, availabilityText } from './api';
import {
  ACTIONS, ACTIVE_TRIAL, EXAMPLE_STEPS, STATE_LABELS, TRIAL_LABELS,
  draftStep, needsLevel, signedThrottle, stepCommand, trialPlan,
} from './esc_trial';
import type { DraftStep, EscAction, EscStatus, EscTrial } from './esc_trial';
import { Icon } from './ui';
import './esc_trial.css';

const timeFormat = new Intl.DateTimeFormat('ja-JP', {hour: '2-digit', minute: '2-digit', second: '2-digit', fractionalSecondDigits: 3, hour12: false});

function TrialMonitor({trial, fresh, cancelling, onCancel}: {
  trial: EscTrial | null; fresh: boolean; cancelling: boolean; onCancel: () => void;
}) {
  const index = trial?.step_index ?? null;
  const current = fresh && trial?.feedback_stale !== true;
  const step = index === null ? null : trial?.steps[index];
  const stateName = trial?.esc_state && (STATE_LABELS[trial.esc_state] || trial.esc_state);
  return <section className="content-card esc-monitor" aria-label="試験モニター">
    <header className="card-heading"><h3>試験モニター</h3>
      {trial && <strong className={'esc-status esc-' + trial.status}>{fresh ? TRIAL_LABELS[trial.status] : '状態未確認'}</strong>}</header>
    <div className="esc-card-body">
      {!trial ? <p>{fresh ? '予約されている試験はありません。' : 'Jetson から試験状態を取得すると、ここに表示します。'}</p> : <>
        <p className="esc-trial-id">試験 ID <code>{trial.id}</code></p>
        {fresh && trial.status === 'queued' && <p>Jetson の試験端末で手順を確認し、この ID を入力して開始します。</p>}
        <dl className="esc-readings">
          <div><dt>{ACTIVE_TRIAL.has(trial.status) ? '実行ステップ' : '最終ステップ'}</dt><dd>{fresh && step && index !== null ? (index + 1) + ' / ' + trial.steps.length + ' · ' + ACTIONS[step.action] : '—'}</dd></div>
          <div><dt>推定 ESC 状態</dt><dd>{current && stateName || '—'}</dd></div>
          <div><dt>スロットル指令</dt><dd>{current && typeof trial.throttle === 'number' ? signedThrottle(trial.throttle) : '—'}</dd></div>
          <div><dt>基板報告 PWM</dt><dd>{current && typeof trial.pwm_us === 'number' ? trial.pwm_us + ' µs' : '—'}</dd>
            {current && trial.status === 'running' && typeof trial.requested_pwm_us === 'number' && <small>目標 {trial.requested_pwm_us} µs · {trial.pwm_observed ? '一致を確認' : '照合待ち'}</small>}</div>
        </dl>
        {fresh && trial.feedback_stale === true && <p className="small-note">基板の新しい出力報告を待っています。</p>}
        {fresh && trial.message && <p className={trial.status === 'failed' ? 'inline-note' : 'esc-message'} role={trial.status === 'failed' ? 'alert' : 'status'}>{trial.message}</p>}
        {fresh && step?.action === 'wait_stopped' && trial.status === 'running' && <p className="small-note">目視確認モードでは、駆動輪の停止を確認して Jetson 端末に <code>STOP</code> を入力します。</p>}
        {trial.stop_evidence && <p className="small-note">直近の停止確認 · {trial.stop_evidence}</p>}
        <details className="esc-submitted-steps"><summary>予約した手順（{trial.steps.length} ステップ）</summary>
          <ol>{trial.steps.map((s, i) => <li key={i} aria-current={fresh && i === index ? 'step' : undefined}>
            <strong>{ACTIONS[s.action]}</strong> · {stepCommand(s)} · {s.action === 'wait_stopped' ? '最大 ' : ''}{s.duration_s} 秒</li>)}</ol>
        </details>
        {!!trial.state_history?.length && <section className="esc-history" aria-label="ESC 状態の推移">
          <h4>状態の推移</h4><div className="esc-history-table"><table>
            <thead><tr><th>時刻</th><th>手順</th><th>試験 / 推定状態</th><th>指令</th><th>PWM [µs]</th></tr></thead>
            <tbody>{[...trial.state_history].reverse().map((event, i) => <tr key={trial.state_history!.length - i}>
              <td>{timeFormat.format(event.updated_at * 1000)}</td><td>{event.step_index === null ? '—' : event.step_index + 1}</td>
              <td>{TRIAL_LABELS[event.status]} / {STATE_LABELS[event.esc_state] || event.esc_state || '—'}</td>
              <td>{typeof event.throttle === 'number' ? signedThrottle(event.throttle) : '—'}</td>
              <td>{event.pwm_us ?? '—'}</td></tr>)}</tbody>
          </table></div>
        </section>}
        {ACTIVE_TRIAL.has(trial.status) && <button className="button esc-cancel" disabled={cancelling || trial.status === 'cancel_requested'} onClick={onCancel}>
          {cancelling || trial.status === 'cancel_requested' ? '中止を要求中…' : 'この試験を中止'}</button>}
      </>}
    </div>
  </section>;
}

export function EscTrialWorkspace() {
  const [rows, setRows] = useState(() => EXAMPLE_STEPS.map((step, i) => draftStep(i, step)));
  const nextKey = useRef(rows.length);
  const [status, setStatus] = useState<EscStatus | null>(null), [received, setReceived] = useState(0);
  const [statusError, setStatusError] = useState(''), [actionError, setActionError] = useState('');
  const [busy, setBusy] = useState<'prepare' | 'cancel' | ''>('');
  const [now, setNow] = useState(Date.now);
  const sequence = useRef(0), sending = useRef(false);
  useEffect(() => {
    let mounted = true, timer: ReturnType<typeof setTimeout>;
    async function poll() {
      const version = sequence.current;
      if (!sending.current) {
        try {
          const data = await api<EscStatus>('esc/status');
          if (mounted && version === sequence.current) {
            if (data.connected === false) setStatusError(data.message || availabilityText(data.availability) || 'Jetson の観測 bridge に接続できません。');
            else { setStatus(data); setReceived(Date.now()); setNow(Date.now()); setStatusError(''); }
          }
        } catch (e) { if (mounted && version === sequence.current) setStatusError((e as Error).message); }
      }
      if (mounted) timer = setTimeout(poll, 800);
    }
    void poll();
    const clock = setInterval(() => setNow(Date.now()), 1000);
    return () => { mounted = false; clearTimeout(timer); clearInterval(clock); };
  }, []);
  const fresh = !!received && !statusError && now >= received && now - received < 4000;
  const trial = status?.trial ?? null, instance = status?.identity?.instance_id;
  const {steps, issue} = trialPlan(rows);
  const canPrepare = fresh && status?.enabled && !!instance && !busy && !issue && !(trial && ACTIVE_TRIAL.has(trial.status));

  function update(key: number, change: Partial<DraftStep>) {
    setRows(current => current.map(row => row.key === key ? {...row, ...change} : row)); setActionError('');
  }
  function move(index: number, offset: number) {
    setRows(current => {
      const next = [...current];
      [next[index], next[index + offset]] = [next[index + offset], next[index]];
      return next;
    }); setActionError('');
  }
  async function send(action: 'prepare' | 'cancel') {
    if (sending.current || (action === 'prepare' ? !canPrepare : !trial || !ACTIVE_TRIAL.has(trial.status))) return;
    sending.current = true; ++sequence.current;
    setBusy(action); setActionError('');
    try {
      const body = action === 'prepare' ? {instance_id: instance, steps} : {id: trial!.id};
      const next = await api<EscStatus>('esc/' + action, body);
      if (next.connected === false) throw new Error(next.message || availabilityText(next.availability) || 'Jetson に接続できません。');
      setStatus(current => ({...current, ...next})); setStatusError(''); setReceived(Date.now()); setNow(Date.now());
    } catch (e) {
      setActionError((e as Error).message);
      setReceived(0);
    } finally { ++sequence.current; sending.current = false; setBusy(''); }
  }

  return <div className="content-page esc-page">
    <div className="page-intro"><div><h2>ESC 試験</h2><p>スロットルの手順を Jetson に予約し、端末から開始した試験を監視します。</p></div>
      <span className={'connection-indicator ' + (fresh ? 'online' : '')}><i/>{fresh ? status?.identity?.hostname || 'Jetson 接続中' : statusError ? 'Jetson 未接続' : 'Jetson 接続を確認中'}</span></div>
    {statusError && <div className="inline-note" role="alert">{statusError}　接続先は「データ転送」で設定します。</div>}
    {fresh && !status?.enabled && <div className="inline-note" role="status">試験モードが無効です。Jetson で <code>./scripts/bringup.sh esc-trial</code> を起動してください。</div>}
    {fresh && status?.enabled && !instance && <div className="inline-note">接続先の Jetson を識別できません。接続を確認してください。</div>}
    <div className="esc-columns">
      <section className="content-card" aria-label="試験手順の編集">
        <header className="card-heading"><h3>試験手順</h3></header>
        <form className="esc-card-body" onSubmit={e => { e.preventDefault(); void send('prepare'); }}>
          <p className="esc-step-note">強度は正規化値です。前進には正、制動と後退には負のスロットル指令を送ります。停止確認中は制動を維持します。</p>
          <div className="esc-step-header" aria-hidden="true"><span>順序</span><span>動作</span><span>強度</span><span>時間 [s]</span></div>
          <ol className="esc-steps">{rows.map((row, i) => <li key={row.key}>
            <span className="esc-step-no">{i + 1}</span>
            <select aria-label={'ステップ ' + (i + 1) + ' の動作'} value={row.action}
              onChange={e => { const action = e.target.value as EscAction; update(row.key, {action}); }}>
              {(Object.entries(ACTIONS) as [EscAction, string][]).map(([value, text]) => <option key={value} value={value}>{text}</option>)}
            </select>
            {needsLevel(row.action)
              ? <input type="number" min="0" max="1" step="any" aria-label={'ステップ ' + (i + 1) + ' の強度'} value={row.level} onChange={e => update(row.key, {level: e.target.value})}/>
              : <span className="esc-fixed-command">{row.action === 'wait_stopped' ? '制動維持' : '0'}</span>}
            <label className="esc-duration"><input type="number" min=".02" max="300" step="any"
              aria-label={'ステップ ' + (i + 1) + (row.action === 'wait_stopped' ? ' の停止確認タイムアウト [s]' : ' の継続時間 [s]')}
              value={row.duration} onChange={e => update(row.key, {duration: e.target.value})}/>
              {row.action === 'wait_stopped' && <small>タイムアウト</small>}</label>
            <div className="esc-row-actions">
              <button className="button small" type="button" disabled={i === 0} aria-label={'ステップ ' + (i + 1) + ' を上へ'} onClick={() => move(i, -1)}>↑</button>
              <button className="button small" type="button" disabled={i === rows.length - 1} aria-label={'ステップ ' + (i + 1) + ' を下へ'} onClick={() => move(i, 1)}>↓</button>
              <button className="button small" type="button" aria-label={'ステップ ' + (i + 1) + ' を削除'} onClick={() => { setRows(current => current.filter(r => r.key !== row.key)); setActionError(''); }}>削除</button>
            </div>
          </li>)}</ol>
          <div className="esc-form-actions"><button className="button" type="button" disabled={rows.length >= 128} onClick={() => {
            const newRow = draftStep(nextKey.current++, {action: 'neutral', duration_s: .5});
            setRows(current => [...current, newRow]); setActionError('');
          }}><Icon name="plus" size={14}/>ステップを追加</button>
            <button className="button primary" type="submit" disabled={!canPrepare}><Icon name="upload" size={15}/>{busy === 'prepare' ? '予約中…' : '手順を予約'}</button></div>
          {issue && <p className="inline-note" role="status">{issue}</p>}
        </form>
      </section>
      <TrialMonitor trial={trial} fresh={fresh} cancelling={!!busy} onCancel={() => void send('cancel')}/>
    </div>
    {actionError && <p className="inline-note" role="alert">{actionError}</p>}
  </div>;
}
