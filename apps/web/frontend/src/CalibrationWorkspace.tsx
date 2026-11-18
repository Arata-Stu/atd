import { useEffect, useState } from 'react';
import { api, bytes } from './api';
import { bagFromHash, calibrationHash, referenceCandidates, validPoseFrame, wheelbaseMetres } from './calibration';
import type { CalibrationOptions, CalibrationRun } from './calibration';
import { ACTIVE, STATUS } from './job_status';
import type { Bag, Job } from './types';
import { Field, Icon } from './ui';

const stages = [
  { id: 'preflight', title: '入力確認' },
  { id: 'pose', title: 'VO 軌跡' },
  { id: 'analysis', title: '車両応答解析' },
];

function JobProgress({ job, onLog }: { job: Job; onLog: (id: string) => void }) {
  const index = stages.findIndex(item => item.id === job.stage);
  const noSamples = job.result?.outcome === 'no_eligible_samples';
  const complete = ['succeeded', 'completed'].includes(job.status);
  return <section className="content-card calibration-job" aria-label="解析ジョブ">
    <div className="card-heading"><div><h3>{job.title || '車両校正ジョブ'}</h3></div>
      <button className="button small" onClick={() => onLog(job.id)}><Icon name="list" size={15}/>実行ログ</button>
    </div>
    <div className="calibration-card-body">
      <ol className="calibration-stages">{stages.map((stage, i) => <li key={stage.id}
        className={complete || index > i ? 'done' : index === i ? 'current' : ''}
        aria-current={index === i && ACTIVE.has(job.status) ? 'step' : undefined}>{stage.title}</li>)}</ol>
      <p role="status"><strong>{job.cancel_requested && ACTIVE.has(job.status) ? '停止要求中' : noSamples ? '診断結果あり・有効試料 0 件' : STATUS[job.status] || job.status}</strong>
        {job.message && !complete && <span className="calibration-job-message">{job.message}</span>}
        {complete && typeof job.result?.pose_reused === 'boolean' && <span className="calibration-job-message">{job.result.pose_reused ? '既存の VO 軌跡を使用' : 'VO 軌跡を新規生成'} · {String(job.result.map_id ?? '')}</span>}
      </p>
    </div>
  </section>;
}

function Result({ run }: { run: CalibrationRun }) {
  const root = '/api/calibration/runs/' + encodeURIComponent(run.id) + '/';
  const [imageError, setImageError] = useState(false);
  return <div className="calibration-result">
    <dl><div><dt>走行 bag</dt><dd>{run.bag}</dd></div><div><dt>応答に使える試料 / 時系列全体</dt>
      <dd>{run.eligible_samples.toLocaleString()} / {run.timeseries_samples.toLocaleString()} 件</dd></div></dl>
    {run.eligible_samples === 0 && <p className="inline-note"><Icon name="info" size={16}/>入力と車体運動を対応付けられた試料がありません。summary.json の analysis.coverage.exclusion_reasons で除外理由を確認できます。</p>}
    {imageError ? <p role="alert">応答図を読み込めませんでした。</p> :
      <a className="calibration-plot" href={root + 'response.png'} target="_blank" rel="noreferrer" aria-label="車両応答図を原寸で開く">
        <img src={root + 'response.png'} onError={() => setImageError(true)} alt="プロポ入力、推定速度・加速度、ヨーレートの時系列とスロットル・操舵の応答図"/>
      </a>}
    <div className="button-row wrapping">{['response.png', 'timeseries.csv', 'summary.json'].map(name =>
      <a key={name} className="button" href={root + name} download={name}><Icon name="download" size={14}/>{name}</a>)}</div>
  </div>;
}

export function CalibrationWorkspace({ jobs, onTransfer, onLog }: {
  jobs: Job[]; onTransfer: () => void; onLog: (id: string) => void;
}) {
  const [bags, setBags] = useState<Bag[] | null>(null);
  const [bag, setBag] = useState(() => bagFromHash(window.location.hash)), [poseFrame, setPoseFrame] = useState(''), [mapId, setMapId] = useState('');
  const [wheelbase, setWheelbase] = useState(''), [reference, setReference] = useState('');
  const [options, setOptions] = useState<CalibrationOptions | null>(null);
  const [selectedRun, setSelectedRun] = useState(''), [submitted, setSubmitted] = useState<Job | null>(null);
  const [revision, setRevision] = useState(0), [submitting, setSubmitting] = useState(false);
  const [recordsError, setRecordsError] = useState(''), [optionsError, setOptionsError] = useState(''), [startError, setStartError] = useState('');
  const calibrationJobs = jobs.filter(item => item.action === 'calibration');
  const recordResource = 'record:' + bag;
  const lastJob = bag ? calibrationJobs.find(item => item.resources?.includes(recordResource)) : calibrationJobs[0];
  const job = submitted?.resources?.includes(recordResource)
    ? calibrationJobs.find(item => item.id === submitted.id) ?? submitted : lastJob;
  const completedRunId = typeof job?.result?.run_id === 'string' ? job.result.run_id : '';

  useEffect(() => {
    let current = true;
    setRecordsError('');
    api<{ items: Bag[] }>('records').then(result => { if (current) setBags(result.items); })
      .catch(e => { if (current) setRecordsError(e.message); });
    return () => { current = false; };
  }, [revision]);
  useEffect(() => {
    let current = true;
    setOptionsError('');
    api<CalibrationOptions>('calibration/options' + (bag ? '?bag=' + encodeURIComponent(bag) : ''))
      .then(result => { if (current) setOptions(result); })
      .catch(e => { if (current) { setOptions(null); setOptionsError(e.message); } });
    return () => { current = false; };
  }, [bag, revision, completedRunId]);

  function chooseBag(value: string) {
    window.history.replaceState(null, '', calibrationHash(value));
    setOptions(null); setBag(value); setMapId(''); setPoseFrame(''); setReference(''); setStartError('');
  }
  const poses = options?.poses ?? [], runs = options?.runs ?? [];
  const source = poses.length === 1 ? poses[0] : poses.find(item => item.map_id === mapId);
  const references = options ? referenceCandidates(options) : [];
  const comparison = references.find(item => item.id === reference);
  const run = runs.find(item => item.id === selectedRun) ?? runs.find(item => item.id === completedRunId) ?? runs[0];
  const measured = wheelbaseMetres(wheelbase);
  const usesBag = (item: Job) => ACTIVE.has(item.status) && item.resources?.includes(recordResource);
  const bagBusy = calibrationJobs.some(usesBag) || (job && usesBag(job));
  const canStart = !!options && !!bags?.some(item => item.id === bag) && validPoseFrame(poseFrame) &&
    measured !== null && (poses.length < 2 || !!source) && (!reference || !!comparison) && !bagBusy && !recordsError;

  async function start() {
    if (!canStart || submitting) return;
    setStartError(''); setSubmitting(true);
    try {
      const result = await api<Job>('calibration/runs', {
        bag, pose_frame: poseFrame.trim(), map_id: source?.map_id, wheelbase_m: measured, reference_run: comparison?.id,
      });
      setSubmitted(result); setSelectedRun('');
    } catch (e) { setStartError((e as Error).message); }
    finally { setSubmitting(false); }
  }

  return <div className="content-page calibration-page">
    <div className="page-intro"><div><h2>手動走行の車両校正</h2><p>ステレオ VO で後輪軸中央の運動を推定し、プロポ入力と IMU に重ねて応答を調べます。</p></div>
      <button className="button" onClick={() => { setOptions(null); setRevision(value => value + 1); }}><Icon name="refresh" size={16}/>一覧を更新</button>
    </div>
    <div className="calibration-columns">
      <section className="content-card"><div className="card-heading"><h3>走行を解析</h3>
        <button className="button small" onClick={onTransfer}><Icon name="download" size={15}/>Jetson から bag を受信</button></div>
        <form className="calibration-card-body" onSubmit={e => { e.preventDefault(); void start(); }}>
          {recordsError && <p className="inline-note" role="alert">{recordsError}</p>}
          {optionsError && <p className="inline-note" role="alert">{optionsError}</p>}
          <Field label="Notebook の rosbag"><select required value={bag} onChange={e => chooseBag(e.target.value)}>
            <option value="">{bags === null ? (recordsError ? '一覧を取得できません' : '一覧を取得中…') : bags.length ? '走行を選択してください' : '受信済みの bag がありません'}</option>
            {bags?.map(item => <option key={item.id} value={item.id}>{item.id} · {bytes(item.bytes)}</option>)}
          </select></Field>
          {bag && options && (poses.length > 1 ?
            <Field label="再利用する VO 軌跡"><select required value={mapId} onChange={e => { setMapId(e.target.value); setPoseFrame(''); }}>
              <option value="">生成元の地図を選択してください</option>{poses.map(item => <option key={item.map_id}>{item.map_id}</option>)}
            </select></Field> :
            <p className="calibration-pose-note">{source ? 'この bag の VO 軌跡を再利用します。生成元 ' + source.map_id : 'この bag の VO 軌跡を生成してから解析します。'}</p>)}
          <Field label="確認した VO 軌跡の子フレーム" hint="odom_poses.tum が表す子フレームを、使用する cuVSLAM の設定・ログで確認してください。">
            <input required value={poseFrame} disabled={!bag || !options} list="calibration-frame-hints" autoComplete="off"
              placeholder="確認したフレーム名を選択または入力" onChange={e => setPoseFrame(e.target.value)}/>
          </Field>
          <datalist id="calibration-frame-hints">{Array.from(new Set([source?.base_frame, 'base_link', 'camera_link'].filter(Boolean))).map(frame => <option key={frame} value={frame!}/>)}</datalist>
          {source?.base_frame && <p className="calibration-pose-note">選択した生成設定の base_frame は <code>{source.base_frame}</code> です。</p>}
          {poseFrame && !validPoseFrame(poseFrame) && <p className="inline-note" role="alert">ROS フレーム名を入力してください。先頭・末尾の / や空白は使用できません。</p>}
          <details className="calibration-optional"><summary>ホイールベース・比較走行（任意）</summary>
            <Field label="実測ホイールベース (mm)" hint="入力すると曲率から等価操舵角も推定します。"><input type="number" step="any" min="0" value={wheelbase}
              placeholder="前後の車軸間距離" onChange={e => setWheelbase(e.target.value)}/></Field>
            {measured === null && <p className="inline-note">0 より大きい実測値を指定してください。</p>}
            <Field label="比較元の解析結果" hint="別の走行 bag で作成した応答表を、今回の走行で評価します。"><select value={reference} disabled={!bag || !options} onChange={e => setReference(e.target.value)}>
              <option value="">比較しない</option>{references.map(item => <option key={item.id} value={item.id}>{item.bag} · {item.id}</option>)}
            </select></Field>
          </details>
          {startError && <p className="inline-note" role="alert">{startError}</p>}
          <button className="button primary" disabled={!canStart || submitting}><Icon name="play" size={15}/>{submitting ? '受け付け中…' : bagBusy ? 'この走行を解析中' : '解析を開始'}</button>
        </form>
      </section>
      {job && <JobProgress job={job} onLog={onLog}/>}
    </div>
    <section className="content-card calibration-history"><div className="card-heading"><h3>解析結果</h3></div>
      <div className="calibration-card-body">{runs.length > 0 && run ? <>
        <Field label="保存済みの解析"><select value={run.id} onChange={e => setSelectedRun(e.target.value)}>
          {runs.map(item => <option key={item.id} value={item.id}>{item.id} · {item.bag}</option>)}</select></Field>
        <Result key={run.id} run={run}/>
      </> : <p className="small-note">{optionsError ? '結果一覧を取得できません。「一覧を更新」から再試行してください。' : !options ? '結果一覧を取得中…' : '保存済みの解析結果はありません。'}</p>}</div>
    </section>
  </div>;
}
