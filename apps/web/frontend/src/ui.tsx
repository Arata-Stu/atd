import type { ReactNode } from 'react';

export function Icon({ name, size = 20 }: { name: string; size?: number }) {
  const paths: Record<string, ReactNode> = {
    map: <><path d="m3 5 6-2 6 2 6-2v16l-6 2-6-2-6 2z"/><path d="M9 3v16M15 5v16"/></>,
    transfer: <><path d="M4 7h15m-4-4 4 4-4 4M20 17H5m4-4-4 4 4 4"/></>,
    chart: <><path d="M3 3v17h18"/><path d="m6 15 4-5 4 2 6-8"/></>,
    live: <><path d="M3 12h4l3-8 4 16 3-8h4"/></>,
    controller: <><path d="M7 7h10a4 4 0 0 1 4 4l1 6a2 2 0 0 1-3 2l-4-3H9l-4 3a2 2 0 0 1-3-2l1-6a4 4 0 0 1 4-4Z"/><path d="M6 10v5m-2.5-2.5h5M16 11h.01M19 14h.01"/></>,
    arrow: <><path d="M5 12h14m-5-5 5 5-5 5"/></>,
    plus: <path d="M12 5v14M5 12h14"/>,
    refresh: <><path d="M20 7v5h-5M4 17v-5h5"/><path d="M6 6a8 8 0 0 1 13 3M5 15a8 8 0 0 0 13 3"/></>,
    check: <path d="m5 12 4 4L19 6"/>,
    close: <path d="m6 6 12 12M18 6 6 18"/>,
    folder: <path d="M3 6a2 2 0 0 1 2-2h5l2 3h7a2 2 0 0 1 2 2v10H3Z"/>,
    download: <><path d="M12 3v12m-5-5 5 5 5-5M4 16v5h16v-5"/></>,
    upload: <><path d="M12 17V5m-5 5 5-5 5 5M4 16v5h16v-5"/></>,
    expand: <><path d="M9 3H3v6m12-6h6v6M3 15v6h6m12-6v6h-6"/></>,
    save: <><path d="M4 3h13l4 4v14H3V3z"/><path d="M7 3v6h9V3M7 21v-8h10v8"/></>,
    play: <path d="m8 4 12 8-12 8Z"/>,
    undo: <><path d="m8 3-5 5 5 5M3 8h10a7 7 0 0 1 0 14"/></>,
    redo: <><path d="m16 3 5 5-5 5m5-5H11a7 7 0 0 0 0 14"/></>,
    list: <><path d="M8 6h13M8 12h13M8 18h13M3 6h.01M3 12h.01M3 18h.01"/></>,
    chevron: <path d="m9 5 7 7-7 7"/>,
    settings: <><path d="M4 7h16M4 17h16"/><circle cx="9" cy="7" r="3"/><circle cx="15" cy="17" r="3"/></>,
    info: <><circle cx="12" cy="12" r="9"/><path d="M12 11v6M12 7h.01"/></>,
  };
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">{paths[name] ?? paths.info}</svg>;
}
export function Field({ label, children, hint }: { label: string; children: ReactNode; hint?: string }) {
  return <label className="field"><span>{label}</span>{children}{hint && <small>{hint}</small>}</label>;
}
export function Empty({ title, text, icon = 'map', children }: { title: string; text: string; icon?: string; children?: ReactNode }) {
  return <div className="empty"><div className="empty-icon"><Icon name={icon} size={27}/></div><h3>{title}</h3><p>{text}</p>{children}</div>;
}
export function Tag({ children, good = false }: { children: ReactNode; good?: boolean }) {
  return <span className={'tag' + (good ? ' good' : '')}><span className="dot"/>{children}</span>;
}
export function Modal({ title, subtitle, children, onClose }: { title: string; subtitle?: string; children: ReactNode; onClose: () => void }) {
  return <div className="modal-backdrop" onMouseDown={e => { if (e.target === e.currentTarget) onClose(); }}><section className="modal" role="dialog" aria-modal="true" aria-label={title}><header><div><h2>{title}</h2>{subtitle && <p>{subtitle}</p>}</div><button className="icon-button" aria-label="閉じる" onClick={onClose}><Icon name="close"/></button></header>{children}</section></div>;
}
