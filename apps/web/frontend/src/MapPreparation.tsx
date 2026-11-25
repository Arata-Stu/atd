import type { Bag, CaptureOptions, Job, MapDocument } from './types';
import { STATUS } from './job_status';
import { Field, Icon, Modal } from './ui';

type StepState = 'complete' | 'ready' | 'blocked' | 'unknown' | 'failed' | 'running';
interface StepProps {
  index: string; name: string; state: StepState; status: string;
  job?: Job; action?: string; onAction?: () => void; disabled?: boolean; detail?: string;
}
function PreparationStep({ index, name, state, status, job, action, onAction, disabled, detail }: StepProps) {
  if (job) {
    state = job.status === 'orphaned' ? 'failed' : 'running';
    status = STATUS[job.status] ?? job.status;
  }
  return <li data-state={state}>
    <div className="preparation-step-heading">
      <span className="preparation-step-index">{index}</span>
      <span className="preparation-step-name">{name}</span>
      <span className="preparation-step-status">{state === 'complete' && <Icon name="check" size={14}/>} {status}</span>
    </div>
    {detail && <p className="preparation-step-detail">{detail}</p>}
    {action && <button type="button" className="button preparation-step-action" disabled={disabled || !!job}
      onClick={onAction}>{action}<Icon name="arrow" size={14}/></button>}
  </li>;
}

interface PreparationProps {
  doc: MapDocument | null; options: CaptureOptions | null; loading: boolean; error: string;
  pointcloudReady: boolean; dirty: boolean; busy: boolean; stageJob?: Job; offlineJob?: Job;
  onBuild: () => void; onVgl: () => void; onCapture: () => void; onEdit: () => void; onRefresh: () => void;
}
export function PreparationPanel({ doc, options, loading, error, pointcloudReady, dirty, busy, stageJob, offlineJob,
  onBuild, onVgl, onCapture, onEdit, onRefresh }: PreparationProps) {
  const workLocked = busy || !!offlineJob;
  const vglReady = options?.vgl_ready === true;
  const checked = !!options && !loading && !error;
  const vglState = !doc ? 'blocked' : error ? 'failed' : !checked ? 'unknown' : vglReady ? 'complete' : 'ready';
  const vglStatus = !doc ? '地図の作成後' : error ? '確認エラー' : !checked ? '確認中' : vglReady ? '準備済み' : '未準備';
  const canCheckCapture = checked && vglReady;
  const hasEdits = !!doc && (doc.left.length > 0 || doc.right.length > 0 || doc.custom.length > 0);
  const hdmapState = !pointcloudReady ? 'blocked' : hasEdits && !dirty ? 'complete' : 'ready';

  return <section className="property-section preparation-panel" aria-label="地図を編集する準備">
    <div className="section-top"><h3>地図を編集する準備</h3>{doc && <button className="icon-button"
      aria-label="地図の準備状態を更新" disabled={loading || busy} onClick={onRefresh}><Icon name="refresh" size={15}/></button>}</div>
    <ol className="preparation-steps">
      <PreparationStep index="01" name={doc ? 'cuVSLAM地図' : 'ステレオ地図'} state={doc ? 'complete' : 'ready'}
        status={doc ? '作成済み' : '未作成'} job={stageJob?.action === 'map_build' ? stageJob : undefined}
        action={doc ? '別の地図を作成' : 'rosbag から地図を作成'} onAction={onBuild} disabled={workLocked}/>
      <PreparationStep index="02" name="VGL地図" state={vglState} status={vglStatus}
        job={stageJob?.action === 'vgl' ? stageJob : undefined}
        action={vglState === 'failed' ? '準備状態を再確認' : vglState === 'complete' ? undefined : 'VGL地図を作成'}
        onAction={error ? onRefresh : onVgl} disabled={workLocked || (!error && (!doc || !checked))}
        detail={error || undefined}/>
      <PreparationStep index="03" name="表示用点群" state={pointcloudReady ? 'complete' : canCheckCapture ? 'ready' : 'blocked'}
        status={pointcloudReady ? '取得済み' : canCheckCapture ? '未取得' : 'VGLの準備後'}
        job={stageJob?.action === 'capture' ? stageJob : undefined}
        action={pointcloudReady ? undefined : '保存地図から点群を取得'} onAction={onCapture}
        disabled={workLocked || !canCheckCapture}
        detail={!pointcloudReady && canCheckCapture ? options?.issues[0] : undefined}/>
      <PreparationStep index="04" name="HDMap編集" state={hdmapState}
        status={!pointcloudReady ? '点群の取得後' : dirty ? '未保存' : hasEdits ? '編集点を保存済み' : '編集可能'}
        action={hasEdits ? '編集を続ける' : '編集を始める'} onAction={onEdit} disabled={busy || !pointcloudReady}/>
    </ol>
    {offlineJob && offlineJob.id !== stageJob?.id && <p className="small-note">{offlineJob.title}（{STATUS[offlineJob.status] ?? offlineJob.status}）</p>}
  </section>;
}

interface CaptureModalProps {
  doc: MapDocument; options: CaptureOptions | null; loading: boolean; error: string; bags: Bag[];
  bag: string; modelDir: string; busy: boolean; onBag: (value: string) => void; onModelDir: (value: string) => void;
  onClose: () => void; onRefresh: () => void; onStart: () => void; onVgl: () => void; onTransfer: () => void;
}
export function CaptureModal({ doc, options, loading, error, bags, bag, modelDir, busy, onBag, onModelDir, onClose, onRefresh, onStart, onVgl, onTransfer }: CaptureModalProps) {
  return <Modal title="保存地図から点群を取得" subtitle={`${doc.title} を読み込み、元bagの画像で自己位置を合わせて点群を保存します。`} onClose={onClose}>
    <div className="capture-selection"><div><span>使用するbag</span><strong>{options?.bag || options?.source_bag || doc.source?.bag || '未選択'}</strong></div><div><span>このPCのVGLモデル</span><strong>{options?.model_dir || 'VGL地図の準備後に自動選択'}</strong></div></div>
    <details><summary>bag・モデルを明示的に選び直す</summary><Field label="bag（記録ディレクトリ内）"><select value={bag} disabled={busy} onChange={e => onBag(e.target.value)}><option value="">元bagを自動選択</option>{bags.map(item => <option key={item.id} value={item.id}>{item.id}</option>)}</select></Field><Field label="このPCで作成したモデルの相対パス" hint="空欄ならVGL地図の準備で使用したモデルを使います。"><input value={modelDir} disabled={busy} placeholder={options?.model_dir || 'models/vgl/…'} onChange={e => onModelDir(e.target.value)}/></Field></details>
    <div className="capture-check-heading"><h3>開始前の確認</h3><button className="button quiet small" disabled={loading || busy} onClick={onRefresh}><Icon name="refresh" size={13}/>{loading ? '確認中' : '再確認'}</button></div>
    {loading ? <p className="small-note">地図・元bag・モデルの対応を確認しています…</p> : error ? <p className="inline-note">{error}</p> : options?.ready ? <div className="capture-ready"><Icon name="check" size={17}/>点群取得を開始できます</div> : <ul className="capture-issues">{options?.issues.map((issue, index) => <li key={index}>{issue}</li>)}</ul>}
    {options && !options.vgl_ready && <button className="button full" disabled={busy} onClick={onVgl}>先にVGL地図を準備する<Icon name="arrow" size={15}/></button>}
    {options && !bags.some(item => item.id === (options.bag || options.source_bag)) && <button className="button quiet full" disabled={busy} onClick={onTransfer}>転送画面で元bagを受信する<Icon name="arrow" size={15}/></button>}
    <p className="small-note">VGLの初期姿勢推定と保存地図での自己位置合わせを確認してから点群を保存します。取得中も車両は起動しません。</p>
    <div className="modal-actions"><button className="button primary" disabled={busy || loading || !options?.ready || !!error} onClick={onStart}><Icon name="play" size={16}/>点群の取得を開始</button></div>
  </Modal>;
}
