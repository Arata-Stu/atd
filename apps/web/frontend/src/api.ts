export async function api<T>(path: string, body?: unknown, method?: string, signal?: AbortSignal): Promise<T> {
  const response = await fetch('/api/' + path, {
    method: method ?? (body === undefined ? 'GET' : 'POST'), signal,
    ...(body === undefined ? {} : { headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }),
  });
  const contentType = response.headers.get('content-type') || '';
  const result = contentType.includes('application/json') ? await response.json() : await response.text();
  if (!response.ok) {
    const detail = typeof result === 'string' ? result : result.error ?? result.detail ?? '処理を完了できませんでした';
    throw new Error(Array.isArray(detail) ? detail.map(x => x.msg || JSON.stringify(x)).join(' / ') : typeof detail === 'string' ? detail : JSON.stringify(detail));
  }
  return result as T;
}
export function bytes(value: number = 0): string {
  if (value < 1024) return `${value} B`;
  const unit = Math.min(3, Math.floor(Math.log(value) / Math.log(1024)));
  return `${(value / 1024 ** unit).toFixed(1)} ${['B', 'KB', 'MB', 'GB'][unit]}`;
}
export function availabilityText(value: unknown): string {
  if (typeof value === 'string') return value;
  if (value && typeof value === 'object') {
    const data = value as Record<string, unknown>;
    if (typeof data.reason === 'string') return data.reason;
    if (typeof data.message === 'string') return data.message;
  }
  return '';
}
