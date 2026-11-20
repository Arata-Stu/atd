import { useEffect, useLayoutEffect, useRef, useState, type ReactNode } from 'react';
import { api } from './api';

type LogPage = { text: string; start: number; offset: number };

export function JobLog({ jobId, title, children }: { jobId: string; title: string; children: ReactNode }) {
  const [log, setLog] = useState<LogPage | null>(null);
  const [following, setFollowing] = useState(true);
  const [loadingOlder, setLoadingOlder] = useState(false);
  const [error, setError] = useState('');
  const viewport = useRef<HTMLPreElement>(null);
  const end = useRef(0);
  const heightBeforePrepend = useRef<number | null>(null);
  const olderRequest = useRef<AbortController | null>(null);
  const path = `jobs/${encodeURIComponent(jobId)}/log?tail=true`;

  useEffect(() => {
    if (!jobId || !following) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout>;
    async function poll() {
      try {
        const page = await api<LogPage>(`${path}&offset=${end.current}`, undefined, undefined, controller.signal);
        if (controller.signal.aborted) return;
        if (page.text || page.offset !== end.current || end.current === 0) {
          end.current = page.offset;
          setLog(page);
        }
        setError('');
      } catch (e) {
        if (!controller.signal.aborted) setError((e as Error).message);
      }
      if (!controller.signal.aborted) timer = setTimeout(poll, 1800);
    }
    void poll();
    return () => { controller.abort(); clearTimeout(timer); };
  }, [jobId, path, following]);

  useEffect(() => () => olderRequest.current?.abort(), []);

  useLayoutEffect(() => {
    const pre = viewport.current;
    if (!pre) return;
    if (following) pre.scrollTop = pre.scrollHeight;
    else if (heightBeforePrepend.current !== null) pre.scrollTop += pre.scrollHeight - heightBeforePrepend.current;
    heightBeforePrepend.current = null;
  }, [log, following]);

  function followLatest() {
    olderRequest.current?.abort();
    setFollowing(true);
  }

  async function loadOlder() {
    if (!log || log.start === 0 || olderRequest.current) return;
    const controller = new AbortController();
    olderRequest.current = controller;
    setLoadingOlder(true);
    try {
      const page = await api<LogPage>(`${path}&before=${log.start}`, undefined, undefined, controller.signal);
      if (controller.signal.aborted || !viewport.current) return;
      if (page.offset !== log.start) throw new Error('ログの内容が変わりました。「最新へ」で再読み込みしてください。');
      heightBeforePrepend.current = viewport.current.scrollHeight;
      setLog({ ...log, start: page.start, text: page.text + log.text });
      setError('');
    } catch (e) {
      if (!controller.signal.aborted) setError((e as Error).message);
    } finally {
      olderRequest.current = null;
      setLoadingOlder(false);
    }
  }

  return <div className="job-log">
    <div className="job-log-header">
      <strong>{title}</strong>
      <div className="job-log-actions">
        {children}
        {!following && !!log?.start && <button className="button small" disabled={loadingOlder} onClick={loadOlder}>前のログを表示</button>}
        {jobId && <button className="button small" disabled={following} onClick={followLatest}>
          {following ? '自動追従中' : '最新へ'}
        </button>}
      </div>
    </div>
    {error && <p className="inline-note" role="status">{error}</p>}
    <pre ref={viewport} tabIndex={0} aria-label="実行ログ" onScroll={event => {
      const pre = event.currentTarget;
      if (pre.scrollHeight - pre.clientHeight - pre.scrollTop <= 2) followLatest();
      else setFollowing(false);
    }}>{log?.text || 'ログはまだありません。'}</pre>
  </div>;
}
