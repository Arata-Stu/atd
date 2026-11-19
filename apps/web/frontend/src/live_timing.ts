import type { AutoObservation } from './auto_drive_view';

export const clock = new Intl.DateTimeFormat('ja-JP', {hour:'2-digit', minute:'2-digit', second:'2-digit',
  fractionalSecondDigits:3, hourCycle:'h23'});

export function clockText(stamp?: number) {
  return typeof stamp === 'number' && Number.isFinite(stamp) ? clock.format(stamp * 1000) : '—';
}

export function captureTiming(data: AutoObservation | null | undefined, roundTripMs: number | null): {
  roundTripMs: number | null;
  boardAgeMs: [number, number] | null;
} {
  if (roundTripMs === null || !Number.isFinite(roundTripMs) || roundTripMs < 0) {
    return {roundTripMs: null, boardAgeMs: null};
  }

  const measured = Math.round(roundTripMs);
  const stamp = data?.current?.board?.stamp_s;
  if (!data || data.publishers !== 1 || !data.session_id || typeof stamp !== 'number'
      || !Number.isFinite(stamp) || stamp <= 0 || !Number.isFinite(data.now_s) || data.now_s < stamp) {
    return {roundTripMs: measured, boardAgeMs: null};
  }

  // Jetson の採取はこの要求の往復中。Notebook に届くまでの追加時間は 0〜往復時間の間にある。
  const ageMs = (data.now_s - stamp) * 1000;
  return {roundTripMs: measured, boardAgeMs: [Math.floor(ageMs), Math.ceil(ageMs + roundTripMs)]};
}
