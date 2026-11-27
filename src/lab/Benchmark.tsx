import { useEffect, useRef, useState } from 'react';
import { Download, Play, Square } from 'lucide-react';
import { defaultConfig, normalizeConfig, type BenchmarkRow, type BenchmarkSummary, type Config, type Layout } from '../robotics/types';

type Msg = { type: 'progress'; completed: number; total: number } | { type: 'done'; rows: BenchmarkRow[]; summary: BenchmarkSummary } | { type: 'error'; message: string };

export default function Benchmark() {
  const [cfg, setCfg] = useState<Config>({ ...defaultConfig, showLidar: false, showCostmap: false });
  const [trials, setTrials] = useState('20');
  const [busy, setBusy] = useState(false);
  const [prog, setProg] = useState({ completed: 0, total: 0 });
  const [rows, setRows] = useState<BenchmarkRow[] | null>(null);
  const [sum, setSum] = useState<BenchmarkSummary | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [ranCfg, setRanCfg] = useState<Config | null>(null);
  const w = useRef<Worker | null>(null);

  useEffect(() => () => w.current?.terminate(), []);

  const n = Math.max(1, Math.min(200, Math.floor(Number(trials)) || 20));
  const run = () => {
    setErr(null); setRows(null); setSum(null); setBusy(true); setProg({ completed: 0, total: n });
    try {
      const worker = new Worker(new URL('../robotics/benchmark.worker.ts', import.meta.url), { type: 'module' });
      w.current = worker;
      const snapshot = normalizeConfig({ ...cfg, movingObstacles: true });
      setCfg(snapshot);
      worker.onmessage = (e: MessageEvent<Msg>) => {
        const d = e.data;
        if (d.type === 'progress') setProg({ completed: d.completed, total: d.total });
        else if (d.type === 'done') { setRows(d.rows); setSum(d.summary); setRanCfg(snapshot); setBusy(false); worker.terminate(); w.current = null; }
        else if (d.type === 'error') { setErr(d.message); setBusy(false); worker.terminate(); w.current = null; }
      };
      worker.onerror = (e) => { setErr(e.message || 'Worker failed to start'); setBusy(false); worker.terminate(); w.current = null; };
      worker.postMessage({ config: snapshot, trials: n });
    } catch (e) { setErr(e instanceof Error ? e.message : String(e)); setBusy(false); }
  };
  const cancel = () => { w.current?.terminate(); w.current = null; setBusy(false); setErr('Cancelled by user.'); };

  const csv = () => {
    if (!rows || !ranCfg || !sum) return;
    const escape=(v:unknown)=>`"${String(v).replace(/"/g,'""')}"`;
    const lines = ['# config', ...Object.entries(ranCfg).map(([k, v]) => `# ${k},${v}`),
      '# summary', ...Object.entries(sum).map(([k,v])=>`# ${k},${v}`),
      'trial,seed,success,time,distance,straightLine,ratio,replans,collisions,staticContacts,dynamicContacts,movingContacts,stoppedContacts,contactEvents',
      ...rows.map((r) => [r.trial, r.seed, r.success, r.time, r.distance, r.straightLine, r.ratio, r.replans, r.collisions,
        r.staticContacts,r.dynamicContacts,r.movingContacts,r.stoppedContacts,JSON.stringify(r.contacts)].map(escape).join(','))];
    const url = URL.createObjectURL(new Blob([lines.join('\n')], { type: 'text/csv' }));
    const a = document.createElement('a'); a.href = url; a.download = `nav-benchmark-${ranCfg.layout}-seed${ranCfg.seed}.csv`; a.click();
    URL.revokeObjectURL(url);
  };

  const num = (k: keyof Config, step: number) => (
    <label className="block"><span className="lbl">{k}</span>
      <input className="num mt-1" type="number" step={step} value={cfg[k] as number} data-testid={`bench-input-${k}`} disabled={busy}
        onChange={(e) => setCfg({ ...cfg, [k]: Number(e.target.value) })} /></label>
  );
  const pct = prog.total ? (prog.completed / prog.total) * 100 : 0;

  return (
    <div className="grid gap-3 lg:grid-cols-[300px_1fr]">
      <section className="panel p-4 space-y-3 h-fit">
        <h3 className="lbl">Benchmark config</h3>
        <p className="text-[12px] text-muted-foreground leading-relaxed">Headless trials run in a Web Worker. Each trial uses a derived seed with random start and goal; nothing is rendered.</p>
        <div className="seg">
          {(['simple', 'office', 'warehouse'] as Layout[]).map((l) => (
            <button key={l} data-on={cfg.layout === l} disabled={busy} onClick={() => setCfg({ ...cfg, layout: l })} data-testid={`bench-layout-${l}`}>{l}</button>
          ))}
        </div>
        <div className="grid grid-cols-2 gap-2">
          <label className="block"><span className="lbl">trials</span><input className="num mt-1" type="number" min={1} max={200} value={trials} disabled={busy} onChange={(e) => setTrials(e.target.value)} data-testid="input-trials" /></label>
          {num('seed', 1)}{num('maxSpeed', 0.05)}{num('lookahead', 0.05)}{num('inflation', 0.01)}{num('obstacleSpeed', 0.05)}{num('beams', 10)}{num('range', 0.5)}{num('fov', 10)}
        </div>
        <p className="text-[11px] text-muted-foreground">Moving obstacles are always enabled. Inputs are clamped to safe simulator limits on run.</p>
        {(['odometryNoise'] as const).map((k) => (
          <label key={k} className="flex items-center gap-2 text-[12px] text-muted-foreground"><input type="checkbox" checked={cfg[k]} disabled={busy} onChange={(e) => setCfg({ ...cfg, [k]: e.target.checked })} data-testid={`bench-check-${k}`} />{k}</label>
        ))}
        <div className="flex gap-2 pt-1">
          <button className="btn btn-p flex-1 justify-center" disabled={busy} onClick={run} data-testid="button-run-benchmark"><Play size={14} />Run {n} trials</button>
          <button className="btn" disabled={!busy} onClick={cancel} data-testid="button-cancel-benchmark"><Square size={13} />Cancel</button>
        </div>
      </section>

      <div className="flex flex-col gap-3 min-w-0">
        {busy && (
          <div className="panel p-4 rise">
            <div className="flex justify-between mono text-[12px] mb-2"><span>running</span><span data-testid="text-progress">{prog.completed} / {prog.total}</span></div>
            <div className="h-1.5 rounded bg-muted overflow-hidden"><div className="h-full bg-primary origin-left transition-transform duration-300" style={{ transform: `scaleX(${pct / 100})` }} /></div>
          </div>
        )}
        {err && <div className="panel p-4 border-destructive/50 text-[13px] flex items-center justify-between rise" data-testid="text-bench-error"><span className="text-destructive">{err}</span><button className="btn" onClick={run}>Retry</button></div>}
        {!busy && !rows && !err && (
          <div className="panel p-10 text-center">
            <div className="mono text-sm text-foreground">No results yet</div>
            <p className="text-[12px] text-muted-foreground mt-1">Configure on the left and run a batch to see success rate, timing and path efficiency.</p>
          </div>
        )}
        {busy && !rows && <div className="panel p-4 space-y-2">{[0, 1, 2, 3].map((i) => <div key={i} className="h-6 rounded bg-muted/60 animate-pulse" />)}</div>}
        {sum && rows && (
          <>
            <div className="grid grid-cols-2 md:grid-cols-3 xl:grid-cols-6 gap-px bg-border rounded-md overflow-hidden rise">
              {[
                ['success', `${sum.successRate.toFixed(1)}%`], ['mean time', `${sum.meanTime.toFixed(2)}s`], ['median time', `${sum.medianTime.toFixed(2)}s`],
                ['path / straight', sum.meanRatio.toFixed(3)], ['mean replans', sum.meanReplans.toFixed(2)], ['collisions', String(sum.totalCollisions)],
                ['static contacts',String(sum.staticContacts)],['dynamic contacts',String(sum.dynamicContacts)],
                ['while moving',String(sum.movingContacts)],['while stopped',String(sum.stoppedContacts)],
                ['trials with contact',String(sum.trialsWithContacts)],
              ].map(([l, v]) => <div key={l} className="bg-card p-3" data-testid={`summary-${l.replace(/\W+/g, '-')}`}><div className="lbl">{l}</div><div className="mono text-xl mt-1">{v}</div></div>)}
            </div>
            <section className="panel overflow-hidden">
              <div className="flex justify-between items-center px-3 py-2 border-b border-border">
                <h3 className="lbl">Per-trial results</h3>
                <button className="btn" onClick={csv} data-testid="button-download-csv"><Download size={13} />CSV</button>
              </div>
              <div className="overflow-auto max-h-[60vh]">
                <table className="w-full mono text-[12px]">
                  <thead className="sticky top-0 bg-card text-muted-foreground text-[10px] uppercase tracking-wider">
                    <tr>{['trial', 'seed', 'result', 'time s', 'dist m', 'straight m', 'ratio', 'replans', 'coll', 'static', 'dynamic', 'moving', 'stopped'].map((h) => <th key={h} className="px-3 py-2 text-right first:text-left font-normal">{h}</th>)}</tr>
                  </thead>
                  <tbody>
                    {rows.map((r) => (
                      <tr key={r.trial} className="border-t border-border hover:bg-muted/40 transition-colors" data-testid={`row-trial-${r.trial}`}>
                        <td className="px-3 py-1.5">{r.trial}</td><td className="px-3 text-right">{r.seed}</td>
                        <td className={`px-3 text-right ${r.success ? 'text-accent' : 'text-destructive'}`}>{r.success ? 'reached' : 'failed'}</td>
                        <td className="px-3 text-right">{r.time.toFixed(2)}</td><td className="px-3 text-right">{r.distance.toFixed(2)}</td><td className="px-3 text-right">{r.straightLine.toFixed(2)}</td>
                        <td className="px-3 text-right">{r.ratio.toFixed(3)}</td><td className="px-3 text-right">{r.replans}</td><td className={`px-3 text-right ${r.collisions ? 'text-destructive' : ''}`}>{r.collisions}</td>
                        <td className="px-3 text-right">{r.staticContacts}</td><td className="px-3 text-right">{r.dynamicContacts}</td>
                        <td className="px-3 text-right">{r.movingContacts}</td><td className="px-3 text-right">{r.stoppedContacts}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
            </section>
          </>
        )}
      </div>
    </div>
  );
}
