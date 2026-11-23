import { useEffect, useState } from 'react';
import { api } from './api';
import { matchesTuningSession, speedRange, tuningChanges, tuningFields, tuningLines, tuningNodes } from './drive_tuning';
import type { NumericTuningKey, TuningDraft, TuningKey, TuningSnapshot, TuningTarget } from './drive_tuning';
import { LINE_NAMES } from './planning_view';
import type { LineKind } from './types';
import { Field, Icon, Modal } from './workspace_ui';

interface Props {
  target: TuningTarget; instanceId: string; connected: boolean;
  planLine: LineKind | null; onClose: () => void;
}

export function DriveTuningDialog({target, instanceId, connected, planLine, onClose}: Props) {
  const [snapshot, setSnapshot] = useState<TuningSnapshot | null>(null);
  const [draft, setDraft] = useState<TuningDraft>({});
  const [reload, setReload] = useState(0);
  const [busy, setBusy] = useState<'read' | 'write' | null>('read');
  const [message, setMessage] = useState(''), [error, setError] = useState('');
  const [needsReadback, setNeedsReadback] = useState(false);
  const node = tuningNodes[target];
  const endpoint = 'live/parameters?target=' + target;
  const source = target === 'gazebo' ? 'Gazebo' : 'Jetson';
  useEffect(() => {
    const controller = new AbortController();
    setBusy('read'); setError(''); setMessage(''); setSnapshot(null);
    api<TuningSnapshot>(endpoint + '&node=' + encodeURIComponent(node), undefined, 'GET', controller.signal)
      .then(data => {
        if (controller.signal.aborted) return;
        if (!matchesTuningSession(data, target, instanceId)) throw new Error('接続先が変わりました。走行設定を開き直してください。');
        setSnapshot(data); setDraft({}); setNeedsReadback(false);
      })
      .catch(e => { if (!controller.signal.aborted) setError((e as Error).message); })
      .finally(() => { if (!controller.signal.aborted) setBusy(null); });
    return () => controller.abort();
  }, [endpoint, node, target, instanceId, reload]);
  const items = snapshot?.items ?? [];
  const lines = tuningLines(items);
  const speed = speedRange(items);
  const {values, issue} = tuningChanges(items, draft);
  const count = Object.keys(values).length;
  const parameter = (key: string) => items.find(p => p.name === key);
  const savedLine = parameter('line')?.value;
  const table = parameter('feedforward_path')?.value;
  const tableLabel = typeof table === 'string' && table ? table : target === 'gazebo' && table === '' ? 'Gazebo の標準 FF 表' : '';
  const speedMin = speed.min !== undefined ? `設定下限 ${speed.min} m/s` : '';
  const speedMax = speed.max !== undefined ? `起動時の上限 ${speed.max} m/s` : '';
  const speedHint = [speedMin, speedMax].filter(Boolean).join(' · ');
  function fieldDisabled(key: TuningKey) {
    const p = parameter(key);
    return !connected || !!busy || needsReadback || p?.allowed !== true || !!p.read_only;
  }
  function update(key: TuningKey, text: string) {
    setDraft(previous => ({...previous, [key]: text}));
    setMessage(''); setError('');
  }
  function numberField(key: NumericTuningKey) {
    const value = parameter(key)?.value, {label, unit} = tuningFields[key];
    return <Field key={key} label={`${label} [${unit}]`}
      hint={draft[key] !== undefined && Number.isFinite(value) && String(value) !== draft[key] ? `現在値 ${value}` : undefined}>
      <input type="number" step="any" aria-label={`${label} [${unit}]`} disabled={fieldDisabled(key)}
        value={draft[key] ?? (typeof value === 'number' ? String(value) : '')}
        onChange={e => update(key, e.target.value)}/>
    </Field>;
  }
  async function apply() {
    if (!snapshot || !connected || busy || needsReadback || issue || !count) return;
    setBusy('write'); setError(''); setMessage('');
    try {
      const result = await api<TuningSnapshot & {applied: boolean}>(endpoint, {node, instance_id: instanceId, values});
      if (!matchesTuningSession(result, target, instanceId)) throw new Error('接続先が変わりました。走行設定を開き直してください。');
      setSnapshot(result);
      if (result.applied !== true) throw new Error('適用後の読戻し値が指定値と一致しません。');
      setDraft({}); setMessage(`${count} 項目を適用し、${source} から現在値を読み戻しました。`);
    } catch (e) {
      setNeedsReadback(true); setError((e as Error).message);
    } finally { setBusy(null); }
  }
  const feedback = !connected ? '接続先を確認できません。接続後に走行設定を開き直してください。'
    : error || issue || message;
  return <Modal title={source + ' の走行設定'} subtitle="この走行に適用する値" onClose={onClose}>
    <form className="drive-tuning" noValidate onSubmit={e => { e.preventDefault(); void apply(); }}>
      {snapshot && <>
        <div className="tuning-fields">
          <Field label="参照ライン" hint={'計画中のライン　' + (planLine ? LINE_NAMES[planLine] : '受信待ち')}>
            <select aria-label="参照ライン" disabled={fieldDisabled('line') || !lines.length}
              value={draft.line ?? (typeof savedLine === 'string' && lines.some(l => l === savedLine) ? savedLine : '')}
              onChange={e => update('line', e.target.value)}>
              {!lines.some(l => l === savedLine) && <option value="" disabled>速度付きラインを確認できません</option>}
              {lines.map(line => <option key={line} value={line}>{LINE_NAMES[line]}</option>)}
            </select>
          </Field>
          <div>{numberField('max_speed_mps')}{speedHint && <small>{speedHint}</small>}</div>
        </div>
        <fieldset><legend>Pure Pursuit の先読み</legend>
          <div className="tuning-fields three">{numberField('min_lookahead_m')}{numberField('lookahead_time_s')}{numberField('max_lookahead_m')}</div>
        </fieldset>
        <fieldset><legend>速度制御 · FF＋PID</legend>
          {tableLabel && <p className="tuning-table">起動時の FF 表　{tableLabel}</p>}
          <div className="tuning-fields three">{numberField('speed_kp')}{numberField('speed_ki')}{numberField('speed_kd')}</div>
        </fieldset>
      </>}
      {busy === 'read' && <p role="status">設定を読み込んでいます…</p>}
      {feedback && <p className="inline-note" role={error || issue || !connected ? 'alert' : 'status'}>{feedback}</p>}
      <div className="modal-actions">
        <button type="button" className="button" disabled={!!busy || !connected} onClick={() => setReload(n => n + 1)}>
          <Icon name="refresh" size={14}/>現在値を再取得</button>
        <button className="button primary" disabled={!connected || !!busy || needsReadback || !!issue || !count}>
          {busy === 'write' ? '適用・確認中…' : '変更を適用'}</button>
      </div>
    </form>
  </Modal>;
}
