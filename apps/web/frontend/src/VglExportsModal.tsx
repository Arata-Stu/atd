import { useEffect, useState } from 'react';
import { api } from './api';
import { Field, Icon, Modal } from './ui';

interface VglExport {
  asset_id: string;
  export_id: string;
  width: number;
  height: number;
  model_dir: string;
}
interface ExportList { items: VglExport[]; result_root: string }
interface Props {
  onClose: () => void;
  preparation?: { title: string; busy: boolean; start: (modelDir: string) => void };
}

const MANUAL = 'manual';

function label(model: VglExport) {
  return `${model.asset_id} / ${model.export_id}（${model.width} × ${model.height}）`;
}

export function VglExportsModal({ onClose, preparation }: Props) {
  const [catalog, setCatalog] = useState<ExportList | null>(null);
  const [revision, setRevision] = useState(0);
  const [error, setError] = useState('');
  const [choice, setChoice] = useState<string | null>(null);
  const [manualPath, setManualPath] = useState('');

  useEffect(() => {
    const request = new AbortController();
    api<ExportList>('vgl/exports', undefined, 'GET', request.signal).then(result => {
      if (!request.signal.aborted) setCatalog(result);
    }).catch((err: Error) => {
      if (!request.signal.aborted) setError(err.message);
    });
    return () => request.abort();
  }, [revision]);

  const loading = !catalog && !error;
  const models = catalog?.items ?? [];
  const activeChoice = choice ?? (models.length === 1 ? models[0].model_dir : '');
  const manual = activeChoice === MANUAL;
  const missingChoice = !!activeChoice && !manual && !models.some(model => model.model_dir === activeChoice);
  const selected = missingChoice ? '' : activeChoice;
  const modelDir = manual ? manualPath.trim() : selected;

  return <Modal title={preparation ? 'VGL地図を準備' : 'VGLモデル一覧'} onClose={onClose}
    subtitle={preparation ? `${preparation.title} の保存画像と、Notebookでexportしたモデルを使用します。` : undefined}>
    <div className="capture-check-heading">
      <h3>ONNX export{catalog && `（${models.length}件）`}</h3>
      <button type="button" className="button small" disabled={loading || preparation?.busy} onClick={() => {
        setCatalog(null);
        setError('');
        setRevision(value => value + 1);
      }}><Icon name="refresh" size={14}/>一覧を更新</button>
    </div>
    {loading && <p role="status" className="small-note">保存済みモデルを確認中…</p>}
    {error && <p role="alert" className="inline-note">モデル一覧を取得できませんでした。{error}</p>}
    {catalog && !models.length && <p className="inline-note">保存済みのVGL exportが見つかりません。Notebookでexportした後、「一覧を更新」を押してください。</p>}
    {!preparation && !!models.length && <ul className="vgl-export-list">{models.map(model =>
      <li key={model.model_dir}><strong>{label(model)}</strong><code>{model.model_dir}</code></li>
    )}</ul>}
    {catalog && <p className="small-note vgl-export-root">保存先 <code>{catalog.result_root}/models/vgl</code></p>}
    {preparation && <form onSubmit={event => {
      event.preventDefault();
      if (modelDir && !preparation.busy) preparation.start(modelDir);
    }}>
      <Field label="使用するVGLモデル"><select required value={selected} disabled={preparation.busy}
        onChange={event => setChoice(event.target.value)}>
        <option value="">モデルを選択</option>
        {models.map(model => <option key={model.model_dir} value={model.model_dir}>{label(model)}</option>)}
        <option value={MANUAL}>パスを手入力</option>
      </select></Field>
      {catalog && missingChoice && <p role="status" className="inline-note">選択していたモデルが一覧にありません。使用するモデルを選び直してください。</p>}
      {manual && <Field label="VGL export bundle（結果保存先からの相対パス）">
        <input required disabled={preparation.busy} value={manualPath} onChange={event => setManualPath(event.target.value)}
          placeholder="models/vgl/asset/exported/export_id"/>
      </Field>}
      <p className="small-note">このNotebookのGPU用engineをビルドし、VGL地図を作成します。</p>
      <div className="modal-actions"><button className="button primary" disabled={preparation.busy || !modelDir}>
        <Icon name="play" size={16}/>VGL地図の準備を開始
      </button></div>
    </form>}
  </Modal>;
}
