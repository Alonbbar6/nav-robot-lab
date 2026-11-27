import type { ReactNode } from 'react';
import readme from '../../README.md?raw';

function inline(t: string, key: string): ReactNode[] {
  const parts = t.split(/(`[^`]+`|\*\*[^*]+\*\*|\[[^\]]+\]\([^)]+\))/g);
  return parts.map((p, i) => {
    const k = `${key}-${i}`;
    if (p.startsWith('`') && p.endsWith('`')) return <code key={k} className="mono text-[0.85em] bg-muted px-1 py-0.5 rounded text-primary">{p.slice(1, -1)}</code>;
    if (p.startsWith('**')) return <strong key={k} className="text-foreground">{p.slice(2, -2)}</strong>;
    const m = p.match(/^\[([^\]]+)\]\(([^)]+)\)$/);
    if (m) return <a key={k} href={m[2]} className="text-accent underline underline-offset-2" target="_blank" rel="noreferrer">{m[1]}</a>;
    return p;
  });
}

function Markdown({ src }: { src: string }) {
  const lines = src.replace(/\r/g, '').split('\n');
  const out: ReactNode[] = [];
  let i = 0;
  while (i < lines.length) {
    const l = lines[i];
    const k = `b${i}`;
    if (l.startsWith('```')) {
      const buf: string[] = []; i++;
      while (i < lines.length && !lines[i].startsWith('```')) buf.push(lines[i++]);
      i++;
      out.push(<pre key={k} className="mono text-[12px] bg-background border border-border rounded p-3 overflow-x-auto my-3">{buf.join('\n')}</pre>);
      continue;
    }
    const h = l.match(/^(#{1,4})\s+(.*)/);
    if (h) {
      const lvl = h[1].length;
      const cls = lvl === 1 ? 'text-2xl font-semibold mt-2 mb-3' : lvl === 2 ? 'lbl !text-[11px] !text-primary mt-8 mb-2 pb-1 border-b border-border' : 'text-sm font-semibold mt-5 mb-1.5';
      out.push(<div key={k} className={cls}>{inline(h[2], k)}</div>); i++; continue;
    }
    if (/^\s*([-*]|\d+\.)\s+/.test(l)) {
      const items: string[] = [];
      const ordered = /^\s*\d+\./.test(l);
      while (i < lines.length && /^\s*([-*]|\d+\.)\s+/.test(lines[i])) items.push(lines[i++].replace(/^\s*([-*]|\d+\.)\s+/, ''));
      const Tag = ordered ? 'ol' : 'ul';
      out.push(<Tag key={k} className={`${ordered ? 'list-decimal' : 'list-disc'} pl-5 space-y-1 my-2 marker:text-muted-foreground`}>{items.map((it, j) => <li key={j}>{inline(it, `${k}-${j}`)}</li>)}</Tag>);
      continue;
    }
    if (l.startsWith('|')) {
      const rows: string[][] = [];
      while (i < lines.length && lines[i].startsWith('|')) { const r = lines[i++]; if (!/^\|[\s:-|]+\|$/.test(r)) rows.push(r.split('|').slice(1, -1).map((c) => c.trim())); }
      out.push(<table key={k} className="my-3 text-[12px] w-full"><tbody>{rows.map((r, j) => <tr key={j} className="border-b border-border">{r.map((c, n) => <td key={n} className={`py-1.5 pr-3 ${j === 0 ? 'lbl' : ''}`}>{inline(c, `${k}-${j}-${n}`)}</td>)}</tr>)}</tbody></table>);
      continue;
    }
    if (!l.trim()) { i++; continue; }
    const para: string[] = [];
    while (i < lines.length && lines[i].trim() && !/^(#|```|\s*[-*]\s|\s*\d+\.\s|\|)/.test(lines[i])) para.push(lines[i++]);
    if (!para.length) { para.push(l); i++; }
    out.push(<p key={k} className="my-2.5">{inline(para.join(' '), k)}</p>);
  }
  return <>{out}</>;
}

export default function About() {
  return (
    <article className="panel max-w-3xl mx-auto px-6 md:px-10 py-8 text-[14px] leading-relaxed text-foreground/85 rise" data-testid="text-readme">
      <Markdown src={readme} />
    </article>
  );
}
