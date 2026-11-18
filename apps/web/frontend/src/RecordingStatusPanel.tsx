import { useEffect, useState } from 'react';
import { api } from './api';
import { recordingAvailability, recordingDuration, recordingPhaseText, recordingReasonText, recordingRequestText, recordingResultText, recordingSampleAge, recordingSize, recordingSourceText, recordingStatusText } from './recording_status';
import type { RecordingEnvelope } from './recording_status';

const observedText = (value: unknown) => typeof value === 'string' && value ? value : '—';
const observedCount = (value: unknown) => typeof value === 'number' && Number.isSafeInteger(value) && value >= 0 ? value.toLocaleString() : '—';

/** 観測のみ。記録の開始・停止は Jetson が受け取ったコントローラ操作で行う。 */
export function RecordingStatusPanel() {
  const [envelope, setEnvelope] = useState<RecordingEnvelope | null>(null), [error, setError] = useState('');
  const [receivedAt, setReceivedAt] = useState(0), [now, setNow] = useState(Date.now());
  useEffect(() => {
    let cancelled = false, timer: ReturnType<typeof setTimeout>;
    const clock = setInterval(() => setNow(Date.now()), 250);
    async function poll() {
      try {
        const result = await api<RecordingEnvelope>('recording/status');
        if (!cancelled) { setEnvelope(result); setReceivedAt(Date.now()); setNow(Date.now()); setError(''); }
      } catch (e) { if (!cancelled) setError((e as Error).message); }
      if (!cancelled) timer = setTimeout(poll, 750);
    }
    void poll();
    return () => { cancelled = true; clearTimeout(timer); clearInterval(clock); };
  }, []);
  const availability = recordingAvailability(envelope, receivedAt, now, !!error), sample = envelope?.status ?? null;
  const fresh = availability === 'fresh', age = recordingSampleAge(sample, receivedAt, now);
  const request = sample?.last_request, result = sample?.last_result;
  const snapshotLabel = fresh ? '' : '（最終受信時）';
  const currentPath = typeof sample?.bag_path === 'string' && !!sample.bag_path;
  const displayedPath = currentPath ? sample.bag_path : result?.bag_path;
  return <section className="content-card recording-status-card" aria-label="rosbag 記録状態" data-state={fresh ? sample?.state : availability}>
    <div className="recording-status-heading"><h3>rosbag 記録</h3><span className={'recording-state-tag ' + (fresh ? sample?.state : availability)} role="status"><i/>{recordingStatusText(availability, sample?.state)}</span><small>{age === null ? '状態の受信待ち' : `${age.toFixed(1)} 秒前の状態`}</small></div>
    <div className="recording-status-body">
      {!fresh && <p className="recording-observation-note">{availability === 'loading' ? 'Jetson の記録状態を取得しています。' : availability === 'disconnected' ? 'Jetson への監視接続がありません。記録が停止したかは、この表示から判断できません。' : availability === 'missing' ? '記録ノードから状態を受信していません。Jetson の記録ノードの起動を確認してください。' : '現在の記録状態は未確認です。新しい状態を受信するまで、前回の表示を現在の状態として扱いません。'}{sample && ` 最終受信時: ${recordingPhaseText(sample.state)}。`}</p>}
      <div className="recording-observation-grid"><div className="recording-bag-path"><span>{!currentPath && result?.bag_path ? '前回の保存先（Jetson）' : '保存先（Jetson）'}{snapshotLabel}</span><strong>{observedText(displayedPath)}</strong></div><div><span>経過・状態時点{snapshotLabel}</span><strong className="recording-elapsed">{recordingDuration(sample?.elapsed_s)}</strong></div><div><span>記録プロファイル{snapshotLabel}</span><strong>{observedText(sample?.profile)}</strong></div></div>
      {typeof sample?.message === 'string' && sample.message && <p className={'recording-state-message' + (fresh && sample.state === 'error' ? ' failed' : '')}>{fresh ? '' : '最終受信時の説明: '}{sample.message}</p>}
      <div className="recording-request-line"><span>直近の操作{snapshotLabel}</span><strong>{recordingRequestText(request ?? null)}</strong>{request && <small>{recordingSourceText(request.source)} · {recordingReasonText(request.reason)}</small>}</div>
      <div className="recording-result-line"><span>前回の終了{snapshotLabel}</span><strong className={result?.success === false ? 'failed' : ''}>{recordingResultText(result ?? null)}</strong>{result && <small>{recordingSize(result.size_bytes)} · {observedCount(result.message_count)} メッセージ</small>}</div>
      {result && <p className="recording-result-reason">{recordingReasonText(result.reason)}</p>}
      {error && <p className="inline-note" role="alert">{error}</p>}
      <details className="recording-status-details"><summary>詳しい受信情報と前回の保存先</summary>
        <dl><dt>状態元 stamp（ROS 秒）</dt><dd>{typeof sample?.stamp === 'number' && Number.isFinite(sample.stamp) ? sample.stamp.toFixed(3) : '—'}</dd><dt>受信元</dt><dd>{observedText(envelope?.identity?.hostname)}</dd><dt>直近操作の入力元</dt><dd>{observedText(request?.source)}</dd><dt>直近操作の理由コード</dt><dd>{observedText(request?.reason)}</dd><dt>前回の bag 保存先</dt><dd>{observedText(result?.bag_path)}</dd><dt>前回の経過時間</dt><dd>{recordingDuration(result?.elapsed_s)}</dd><dt>前回終了の理由コード</dt><dd>{observedText(result?.reason)}</dd></dl>
      </details>
      <p className="recording-status-footnote">開始・停止は割り当てた Joy／プロポのボタンで行います。停止要求の受付と、bag の保存確定は別の状態です。</p>
    </div>
  </section>;
}
