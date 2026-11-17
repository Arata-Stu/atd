import type { Bag, CaptureOptions, MapDocument } from './types';
import { Field, Icon, Modal } from './ui';

interface PreparationProps {
  doc: MapDocument; options: CaptureOptions | null; loading: boolean; error: string;
  pointcloudReady: boolean; busy: boolean; onVgl: () => void; onCapture: () => void; onRefresh: () => void;
}
export function PreparationPanel({ options, loading, error, pointcloudReady, busy, onVgl, onCapture, onRefresh }: PreparationProps) {
  return <div className="property-section preparation-panel">
    <div className="section-top"><h3>地図を編集する準備</h3><button className="icon-button" aria-label="地図の準備状態を更新" disabled={loading || busy} onClick={onRefresh}><Icon name="refresh" size={15}/></button></div>
    <ol className="preparation-steps"><li className="complete"><b>01</b><span>cuVSLAM地図<strong>作成済み</strong></span><Icon name="check" size={14}/></li><li className={options?.vgl_ready ? 'complete' : ''}><b>02</b><span>VGL地図<strong>{options?.vgl_ready ? '準備済み' : loading ? '確認中' : options ? '未準備' : '未確認'}</strong></span>{options?.vgl_ready && <Icon name="check" size={14}/>}</li><li className={pointcloudReady ? 'complete' : ''}><b>03</b><span>表示用点群<strong>{pointcloudReady ? '取得済み' : '未取得'}</strong></span>{pointcloudReady && <Icon name="check" size={14}/>}</li><li className={pointcloudReady ? 'complete' : ''}><b>04</b><span>HDMap編集<strong>{pointcloudReady ? '編集可能' : '点群の取得後'}</strong></span></li></ol>
    {!options?.vgl_ready && !pointcloudReady && <button className="button full" disabled={busy} onClick={onVgl}>VGL地図を作成<Icon name="arrow" size={15}/></button>}
    {!pointcloudReady && <button className="button primary full" disabled={busy || loading} onClick={onCapture}><Icon name="download" size={15}/>保存地図から点群を取得</button>}
    {error && <p className="generation-issue">{error}</p>}
    {!pointcloudReady && options?.issues.length ? <p className="small-note">{options.issues[0]}{options.issues.length > 1 ? `（ほか${options.issues.length - 1}件）` : ''}</p> : null}
  </div>;
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
