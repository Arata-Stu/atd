import { useState } from 'react';
import { api } from './api';

export function DeleteAnalysisButton({path, body, detail, onDeleted, label = '結果を削除', disabled}: {
  path: string; body?: unknown; detail: string; onDeleted: () => void; label?: string; disabled?: boolean;
}) {
  const [pending, setPending] = useState(false), [error, setError] = useState('');
  async function remove() {
    if (pending || !window.confirm(detail)) return;
    setPending(true); setError('');
    try {
      await api(path, body, 'DELETE');
      onDeleted();
    } catch (exception) {
      setError((exception as Error).message);
    } finally { setPending(false); }
  }
  return <>
    <button type="button" className="button small danger" disabled={disabled || pending} onClick={() => void remove()}>
      {pending ? '削除中…' : label}
    </button>
    {error && <span className="inline-note" role="alert">{error}</span>}
  </>;
}
