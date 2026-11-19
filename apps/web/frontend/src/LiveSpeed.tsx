import { useState } from 'react';
import { api } from './api';
import { GainsText, SpeedComparison, TableSelect, TablesNotice, useSpeedTables } from './SpeedComparison';
import { liveSpeedHistory, liveSpeedIssue } from './speed_view';
import type { LiveStatus } from './workspace_types';

function JetsonSpeedTable({status, fresh}: {status: LiveStatus | null; fresh:boolean}) {
  const tables = useSpeedTables();
  const [choice, setChoice] = useState(''), [busy, setBusy] = useState(false);
  const [error, setError] = useState(''), [saved, setSaved] = useState('');
  const instance = status?.identity?.instance_id, speed = status?.speed_control;
  const digest = fresh ? speed?.proposal_sha256 : '';
  async function save() {
    if (busy || !instance || !choice || !status?.connected) return;
    setBusy(true); setError(''); setSaved('');
    try {
      const answer = await api<{saved:boolean; proposal_sha256:string}>('live/speed-table',
        {instance_id:instance, proposal_sha256:choice}, 'PUT');
      if (!answer.saved || answer.proposal_sha256 !== choice) throw new Error('Jetson に保存された表を確認できません。');
      setSaved(choice);
    } catch (e) { setError((e as Error).message); }
    finally { setBusy(false); }
  }
  return <details className="control-config"><summary>Jetson の FF 表と PID 設定</summary>
    <p>観測中の表 {digest ? speed!.calibration_run_id + ' · ' + digest.slice(0,8) : '未確認'}
      {speed && fresh && <>　<GainsText gains={speed}/></>}</p>
    <TablesNotice tables={tables}/>
    {tables.data && tables.data.items.length > 0 && <div className="speed-form">
      <TableSelect tables={tables.data.items} value={choice} onChange={value => {setChoice(value); setError(''); setSaved('');}}/>
      <button type="button" disabled={busy || !status?.connected || !instance || !choice || digest === choice} className="button small"
        onClick={() => void save()}>{busy ? '保存中…' : 'Jetson の観測に適用'}</button>
    </div>}
    {saved && <p role="status">{digest === saved ? 'Jetson の速度制御ノードに反映済み' : 'Jetson に保存しました。ノードによる読込を待っています。'}</p>}
    {error && <p className="planning-note" role="alert">{error}</p>}
  </details>;
}
export function LiveSpeed({status, expired, enabled}: {status: LiveStatus | null; expired: boolean; enabled: boolean}) {
  const row = status?.speed_control;
  const issue = !enabled ? '表示を一時停止しています。' : liveSpeedIssue(status, expired);
  return <section className="content-card live-speed">
    <SpeedComparison sample={row} issue={issue} points={row && !issue ? liveSpeedHistory(row) : []} time={row?.stamp ?? 0}>
      <JetsonSpeedTable key={status?.identity?.instance_id} status={status} fresh={!issue}/>
    </SpeedComparison>
  </section>;
}
