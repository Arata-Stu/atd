import { useState } from 'react';
import { Field, Modal } from './workspace_ui';

export type MapScope = 'hdmap' | 'map';
export interface GazeboMapEntry { id: string; gazebo_manifest_sha256: string }

interface Props {
  doc: { id: string; revision?: number };
  action: 'duplicate' | 'delete';
  busy: boolean;
  onClose: () => void;
  onSubmit: (scope: MapScope, destination: string) => void;
}

export default function MapLibraryDialog({ doc, action, busy, onClose, onSubmit }: Props) {
  const duplicate = action === 'duplicate';
  const hasHdmap = doc.revision !== undefined;
  const [scope, setScope] = useState<MapScope>(duplicate || !hasHdmap ? 'map' : 'hdmap');
  const [destination, setDestination] = useState(doc.id.slice(0, 91) + '-copy');
  const target = scope === 'map' ? '地図全体' : 'HDMap編集内容';
  const description = duplicate
    ? scope === 'map' ? '地図フォルダー内の全資産を新しいIDにコピーします。' : '境界・走行ライン・校正制限と表示用点群を、新しい手動編集用HDMapにコピーします。'
    : scope === 'map' ? 'このNotebookにある地図フォルダーを削除します。自己位置推定用の地図、点群、HDMap、書き出し履歴が対象です。' : '境界・Custom点・生成ライン・採用した校正制限と書き出し履歴を削除します。地図ID・背景点群・自己位置推定用の地図は引き続き使えます。';
  return <Modal title={duplicate ? '地図を複製' : '地図の削除'} subtitle={`対象地図　${doc.id}（${hasHdmap ? 'rev ' + doc.revision : 'Gazebo 保存地図'}）`} onClose={() => !busy && onClose()}>
    <form onSubmit={e => { e.preventDefault(); onSubmit(scope, destination); }}>
      {hasHdmap && <Field label={duplicate ? '複製する範囲' : '削除する範囲'}><select value={scope} onChange={e => setScope(e.target.value as MapScope)} disabled={busy}>
        <option value="hdmap">HDMap編集内容</option><option value="map">地図全体</option>
      </select></Field>}
      <p className="inline-note">{hasHdmap ? description : `Notebook に保存された Gazebo の cuVSLAM 地図と mapping.json を含むフォルダー全体を${duplicate ? '複製' : '削除'}します。`}</p>
      {duplicate && <Field label="複製先の地図ID" hint="新しいIDを指定してください。"><input required maxLength={96} pattern="[A-Za-z0-9][A-Za-z0-9_.\-]*" value={destination} disabled={busy} onChange={e => setDestination(e.target.value)}/></Field>}
      <div className="modal-actions">
        <button type="button" className="button" disabled={busy} onClick={onClose}>キャンセル</button>
        <button className={duplicate ? 'button primary' : 'button danger'} disabled={busy || (duplicate && (!destination || destination === doc.id))}>
          {target}を{duplicate ? '複製' : '削除'}
        </button>
      </div>
    </form>
  </Modal>;
}
