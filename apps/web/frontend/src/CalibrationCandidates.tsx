import { useEffect, useState } from 'react';
import { api } from './api';
import { adoptionRequest, applicable, LIMIT_KEYS, LIMIT_LABELS } from './calibration_candidates';
import type { LimitCandidate, VehicleCandidates } from './calibration_candidates';
import { restoredSettings } from './map_workflow';
import type { MapDocument, RacelineLimitKey } from './types';
import { Field, Icon } from './ui';

const displayValue = new Intl.NumberFormat('ja-JP', {maximumFractionDigits: 4});
const sideNames = {left: '左旋回', right: '右旋回', straight: '直進'};

function IntervalEvidence({candidate}: {candidate: LimitCandidate}) {
  const [expanded, setExpanded] = useState(false);
  if (!candidate.evidence.length) return null;
  return <details className="interval-evidence" onToggle={event => setExpanded(event.currentTarget.open)}>
    <summary>根拠区間 {candidate.evidence.length} 件を表示（集計 {candidate.episodes} 区間 · {candidate.samples.toLocaleString()} 試料）</summary>
    {expanded && <div className="candidate-table-wrap"><table>
      <thead><tr><th>区間</th><th>走行</th><th>開始〜終了 (ROS 秒)</th><th>速度域 (m/s)</th><th>観測値 ({candidate.unit})</th><th>試料数</th></tr></thead>
      <tbody>{candidate.evidence.map((interval, i) => <tr key={i}>
        <td>{interval.episode_id}</td><td>{sideNames[interval.side]}</td>
        <td>{interval.start_s.toFixed(3)} 〜 {interval.end_s.toFixed(3)}</td>
        <td>{displayValue.format(interval.speed_min_mps)} 〜 {displayValue.format(interval.speed_max_mps)}</td>
        <td>{displayValue.format(interval.value)}</td><td>{interval.samples.toLocaleString()}</td>
      </tr>)}</tbody>
    </table></div>}
  </details>;
}

function FeedforwardTable({proposal, onRefresh}: {proposal: VehicleCandidates; onRefresh: () => void}) {
  const ff = proposal.feedforward;
  const [busy, setBusy] = useState(false), [error, setError] = useState('');
  const [adopting, setAdopting] = useState(false), [adopted, setAdopted] = useState('');
  async function adopt() {
    if (adopting || !ff.usable) return;
    setAdopting(true); setError('');
    try {
      const reply = await api<{table:{proposal_sha256:string}}>('speed/tables',
        {run_id: proposal.run_id, proposal_sha256: proposal.proposal_sha256});
      if (reply.table.proposal_sha256 !== proposal.proposal_sha256) throw new Error('採用した表の版を確認できません。');
      setAdopted(reply.table.proposal_sha256);
    } catch (e) { setError((e as Error).message); }
    finally { setAdopting(false); }
  }
  async function download() {
    if (busy || !ff.usable) return;
    setBusy(true); setError('');
    try {
      const file = await api('calibration/runs/' + encodeURIComponent(proposal.run_id) + '/feedforward.json?' +
        new URLSearchParams({proposal_sha256: proposal.proposal_sha256}));
      const url = URL.createObjectURL(new Blob([JSON.stringify(file, null, 2) + '\n'], {type: 'application/json'}));
      const anchor = document.createElement('a');
      anchor.href = url; anchor.download = 'throttle-ff-' + proposal.run_id + '.json'; anchor.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (e) { setError((e as Error).message); }
    finally { setBusy(false); }
  }
  return <section className="candidate-section" aria-label="速度フィードフォワード">
    <h4>速度フィードフォワード</h4>
    <p>前進時の目標速度に対応するプロポのスロットル入力（S.BUS CH2 raw）です。</p>
    <p role="status"><strong>{ff.usable ? '補間可能' : '表の採用保留'}</strong> · {ff.reason}</p>
    {ff.usable && ff.speed_min_mps !== null && ff.speed_max_mps !== null && <p>
      適用範囲 {displayValue.format(ff.speed_min_mps)} 〜 {displayValue.format(ff.speed_max_mps)} m/s（点間は線形補間、外挿なし）
    </p>}
    {ff.points.length > 0 && <div className="candidate-table-wrap"><table aria-label="スロットルのフィードフォワード表">
      <thead><tr><th>定常速度 (m/s)</th><th>スロットル (CH2 raw)</th><th>観測速度域 (m/s)</th><th>区間数</th><th>試料数</th></tr></thead>
      <tbody>{ff.points.map((point, index) => <tr key={index}><td>{displayValue.format(point.speed_mps)}</td>
        <td>{displayValue.format(point.throttle_raw)}</td>
        <td>{displayValue.format(point.speed_min_mps)} 〜 {displayValue.format(point.speed_max_mps)}</td>
        <td>{point.episodes}</td><td>{point.samples.toLocaleString()}</td></tr>)}</tbody>
    </table></div>}
    {error && <div className="inline-note" role="alert">{error}<button className="button small" onClick={onRefresh}>候補を再取得</button></div>}
    {ff.usable && <div className="button-row wrapping"><button className="button small" disabled={adopting || adopted === proposal.proposal_sha256} onClick={() => void adopt()}>
      {adopted === proposal.proposal_sha256 ? 'Notebook に採用済み' : adopting ? '採用中…' : '速度比較に採用'}
    </button><button className="button small" disabled={busy} onClick={() => void download()}>
      <Icon name="download" size={14}/>{busy ? '保存準備中…' : 'フィードフォワード表を JSON で保存'}
    </button></div>}
  </section>;
}

function CandidateReview({runId, onOpenMap}: {runId: string; onOpenMap: (id: string) => void}) {
  const [proposal, setProposal] = useState<VehicleCandidates | null>(null);
  const [maps, setMaps] = useState<MapDocument[] | null>(null), [mapId, setMapId] = useState('');
  const [fields, setFields] = useState<RacelineLimitKey[]>([]);
  const [error, setError] = useState(''), [mapError, setMapError] = useState(''), [saveError, setSaveError] = useState('');
  const [saving, setSaving] = useState(false), [saved, setSaved] = useState(''), [version, setVersion] = useState(0);

  useEffect(() => {
    let current = true;
    api<VehicleCandidates>('calibration/runs/' + encodeURIComponent(runId) + '/candidates')
      .then(value => { if (current) setProposal(value); })
      .catch(e => { if (current) setError(e.message); });
    api<{items: MapDocument[]}>('maps')
      .then(value => { if (current) setMaps(value.items); })
      .catch(e => { if (current) setMapError(e.message); });
    return () => { current = false; };
  }, [runId, version]);

  function refresh() {
    setError(''); setMapError(''); setSaveError(''); setSaved(''); setFields([]);
    setProposal(null); setMaps(null); setMapId(''); setVersion(value => value + 1);
  }

  const map = maps?.find(doc => doc.id === mapId);
  const currentSettings = map && restoredSettings(map, 'raceline');
  const adopted = map?.raceline_calibration;
  const request = proposal && adoptionRequest(proposal, map, fields);

  function chooseMap(id: string) {
    const prior = maps?.find(doc => doc.id === id)?.raceline_calibration;
    const same = prior && proposal && prior.run_id === runId && prior.proposal_sha256 === proposal.proposal_sha256;
    setFields(same ? LIMIT_KEYS.filter(key => prior.limits[key] !== undefined) : []);
    setMapId(id); setSaveError(''); setSaved('');
  }
  async function adopt() {
    if (!map || !request || saving || saveError) return;
    setSaving(true); setSaved('');
    try {
      const updated = await api<MapDocument>('maps/' + encodeURIComponent(map.id) + '/raceline-calibration', request);
      setMaps(previous => previous?.map(doc => doc.id === updated.id ? updated : doc) ?? []);
      setSaved(`${updated.title || updated.id}（rev ${updated.revision}）に ${request.fields.length} 項目を採用しました。`);
    } catch (e) { setSaveError((e as Error).message); }
    finally { setSaving(false); }
  }

  if (!proposal) return error ? <div role="alert" className="inline-note">{error}
    <button className="button small" onClick={refresh}>再取得</button></div> : <p role="status">校正結果から候補を計算中…</p>;
  return <>
    <section className="candidate-section" aria-label="Raceline の制限候補">
      <h4>Raceline の制限候補</h4>
      <Field label="採用先の地図"><select value={mapId} disabled={!maps?.length || saving} onChange={e => chooseMap(e.target.value)}>
        <option value="">{mapError ? '地図を取得できません' : !maps ? '地図を取得中…' : maps.length ? '地図を選択してください' : '保存済みの地図がありません'}</option>
        {maps?.map(doc => <option key={doc.id} value={doc.id}>{doc.title && doc.title !== doc.id ? doc.title + ' · ' : ''}{doc.id} · rev {doc.revision}</option>)}
      </select></Field>
      {mapError && <div role="alert" className="inline-note">{mapError}<button className="button small" onClick={refresh}>再取得</button></div>}
      {adopted && <p>現在の採用元 {adopted.bag} · {adopted.run_id}。保存すると、チェックした項目だけがこの地図の採用設定に残ります。</p>}
      <div className="candidate-limits">
        {LIMIT_KEYS.map(key => {
          const candidate = proposal.limits[key], available = applicable(candidate);
          return <div className="candidate-limit" key={key}>
            <label><input type="checkbox" aria-label={LIMIT_LABELS[key] + 'を採用'} checked={fields.includes(key)}
              disabled={!map || !available || saving || !!saveError} onChange={event => {
                setFields(event.target.checked ? [...fields, key] : fields.filter(field => field !== key)); setSaved('');
              }}/><span>{LIMIT_LABELS[key]}</span> <strong>{available ? displayValue.format(candidate.value!) + ' ' + candidate.unit : '候補なし'}</strong>
            </label>
            {currentSettings && <small>地図の現在値 {displayValue.format(currentSettings[key])} {candidate.unit}{adopted?.limits[key] !== undefined && '（校正から採用中）'}</small>}
            <p>{candidate.reason}</p>
            <IntervalEvidence candidate={candidate}/>
          </div>;
        })}
      </div>
      {map?.lines.raceline && request && <p>保存済み Raceline への反映には、採用後に再生成が必要です。</p>}
      {saveError && <div className="inline-note" role="alert">{saveError}<button className="button small" onClick={refresh}>候補と地図を再取得</button></div>}
      {saved && <p role="status">{saved}</p>}
      <div className="button-row wrapping"><button className="button primary" disabled={!request || saving || !!saveError} onClick={() => void adopt()}>
        <Icon name="save" size={15}/>{saving ? '保存中…' : '選択した制限を地図に採用'}</button>
        {map && adopted && <button className="button" disabled={saving} onClick={() => onOpenMap(map.id)}>Raceline 生成画面を開く<Icon name="arrow" size={15}/></button>}
      </div>
    </section>
    <FeedforwardTable proposal={proposal} onRefresh={refresh}/>
  </>;
}

export function CalibrationCandidates({runId, onOpenMap}: {runId: string; onOpenMap: (id: string) => void}) {
  const [opened, setOpened] = useState(false);
  return <details className="calibration-candidates" onToggle={e => { if (e.currentTarget.open) setOpened(true); }}>
    <summary>Raceline の制限候補とスロットル表を確認</summary>
    {opened && <CandidateReview runId={runId} onOpenMap={onOpenMap}/>}
  </details>;
}
