import { useEffect, useRef, useState } from 'react';
import { api } from './api';
import { matchesTuningSession, tuningNodes } from './drive_tuning';
import type { TuningSnapshot } from './drive_tuning';
import { gazeboHoldStatus } from './gazebo_hold';
import type { LiveStatus } from './workspace_types';

export interface GazeboOperatorStatus {
  instance: string; data: LiveStatus['gazebo']; pollStart: number;
}

export function GazeboVehicleControls({status}: {status: GazeboOperatorStatus | null}) {
  const data = status?.data;
  const start = status?.pollStart ?? 0;
  const [, refresh] = useState(0);
  const request = useRef<AbortController | null>(null);
  const [sending, setSending] = useState<boolean | null>(null);
  const [accepted, setAccepted] = useState<{pause: boolean; time: number} | null>(null);
  const [error, setError] = useState('');
  const expiry = start + (data?.vehicle_hold?.remaining_s ?? 0) * 1000;
  useEffect(() => {
    const remaining = expiry - performance.now();
    if (remaining <= 0) return;
    const timer = window.setTimeout(() => refresh(n => n + 1), Math.ceil(remaining));
    return () => window.clearTimeout(timer);
  }, [expiry]);
  useEffect(() => () => request.current?.abort(), []);

  // Only a poll that started after readback can confirm this command.
  const state = data?.current?.state;
  useEffect(() => {
    if (accepted && start >= accepted.time && state &&
        (state === 'stopped' || (state === 'paused') === accepted.pause)) setAccepted(null);
  }, [accepted, start, state]);
  useEffect(() => {
    if (!accepted) return;
    const timer = window.setTimeout(() => {
      setAccepted(null);
      setError('制御状態の更新を確認できません。表示中の車両状態を確認してください。');
    }, 5000);
    return () => window.clearTimeout(timer);
  }, [accepted]);

  const view = gazeboHoldStatus(data, start, performance.now());
  async function apply(pause: boolean) {
    const current = gazeboHoldStatus(data, start, performance.now());
    if (!status?.instance || request.current || accepted || !(pause ? current.canPause : current.canResume)) return;
    const controller = new AbortController();
    request.current = controller;
    setSending(pause); setError('');
    try {
      const result = await api<TuningSnapshot & {applied: boolean}>('live/parameters?target=gazebo',
        {node: tuningNodes.gazebo, instance_id: status.instance, values: {operator_paused: pause}}, 'POST', controller.signal);
      if (controller.signal.aborted) return;
      if (!matchesTuningSession(result, 'gazebo', status.instance) || result.applied !== true ||
          result.items.find(item => item.name === 'operator_paused')?.value !== pause)
        throw new Error('同じ Gazebo 走行からの設定の読戻しを確認できません。');
      setAccepted({pause, time: performance.now()});
    } catch (e) {
      if (!controller.signal.aborted) setError((e as Error).message);
    } finally {
      if (!controller.signal.aborted) { request.current = null; setSending(null); }
    }
  }
  const busy = sending !== null || accepted !== null;
  const action = sending ?? accepted?.pause;
  const feedback = busy ? (action ? '一時停止' : '再開') +
    (sending !== null ? 'を要求中…' : '要求を受付済み・制御状態を確認中') : view.text;
  return <div className="gazebo-vehicle-controls" role="group" aria-label="Gazebo 車両の操作">
    <div className="button-row">
      <button type="button" className="button danger" disabled={busy || !status || !view.canPause}
        onClick={() => void apply(true)}>車両を一時停止</button>
      <button type="button" className="button" disabled={busy || !status || !view.canResume}
        onClick={() => void apply(false)}>走行を再開</button>
    </div>
    <span role="status">{feedback}</span>
    {error && <span role="alert">{error}</span>}
  </div>;
}
