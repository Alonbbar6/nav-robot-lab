import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react';
import { Pause, Play, RotateCcw, StepForward, Crosshair, Compass, Gamepad2 } from 'lucide-react';
import { Simulation } from '../robotics/simulation';
import { defaultConfig, type Config, type Layout, type Mode, type Point } from '../robotics/types';
import { render, pixelToWorld, type Frame, type View } from './renderer';

const DT = 1 / 60;
const MAX_STEPS = 12;

interface Snap {
  time: number; status: string; safety: string; mode: Mode; localPlannerState: string;
  pose: { x: number; y: number; theta: number }; odom: { x: number; y: number; theta: number };
  metrics: Record<string, number>; rates: { topic: string; rate: number; count: number }[];
  goal: Point | null; frontiers: number; pathPts: number;
}

function snap(sim: Simulation): Snap {
  let rates: Snap['rates'] = [];
  try { rates = sim.bus?.rates(sim.time) ?? []; } catch { rates = []; }
  return {
    time: sim.time, status: sim.status, safety: sim.safety, mode: sim.mode, localPlannerState: sim.localPlannerState,
    pose: { ...sim.pose }, odom: { ...sim.odom }, metrics: { ...sim.metrics } as unknown as Record<string, number>,
    rates, goal: sim.goal ? { ...sim.goal } : null, frontiers: sim.frontiers.length, pathPts: sim.path.poses.length,
  };
}

function Canvas({ simRef, view, onPick, label, sub }: { simRef: React.MutableRefObject<Simulation>; view: View; onPick: (p: Point) => void; label: string; sub: string }) {
  const ref = useRef<HTMLCanvasElement>(null);
  const frame = useRef<Frame | null>(null);
  const hover = useRef<Point | null>(null);
  useEffect(() => {
    let id = 0;
    const loop = (t: number) => {
      if (ref.current) frame.current = render(ref.current, simRef.current, view, hover.current, t / 1000);
      id = requestAnimationFrame(loop);
    };
    id = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(id);
  }, [simRef, view]);
  const toWorld = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!frame.current || !ref.current) return null;
    const r = ref.current.getBoundingClientRect();
    return pixelToWorld(frame.current, e.clientX - r.left, e.clientY - r.top);
  };
  return (
    <div className="panel relative flex flex-col overflow-hidden" data-testid={`canvas-panel-${view}`}>
      <div className="flex items-baseline justify-between px-3 py-2 border-b border-border">
        <span className="lbl" style={{ color: 'hsl(var(--foreground))' }}>{label}</span>
        <span className="lbl">{sub}</span>
      </div>
      <canvas
        ref={ref}
        data-testid={`canvas-${view}`}
        className="w-full aspect-[4/3] cursor-crosshair block"
        onMouseMove={(e) => { hover.current = toWorld(e); }}
        onMouseLeave={() => { hover.current = null; }}
        onClick={(e) => { const p = toWorld(e); if (p) onPick(p); }}
      />
    </div>
  );
}

function Section({ title, children, right }: { title: string; children: ReactNode; right?: ReactNode }) {
  return (
    <section className="panel p-3 rise">
      <div className="flex items-center justify-between mb-2.5"><h3 className="lbl">{title}</h3>{right}</div>
      {children}
    </section>
  );
}

function Seg<T extends string | number>({ value, options, onChange, id }: { value: T; options: { v: T; l: ReactNode }[]; onChange: (v: T) => void; id: string }) {
  return (
    <div className="seg">
      {options.map((o) => (
        <button key={String(o.v)} data-on={o.v === value} onClick={() => onChange(o.v)} data-testid={`${id}-${o.v}`} className="inline-flex items-center justify-center gap-1.5">{o.l}</button>
      ))}
    </div>
  );
}

function Slider({ label, k, min, max, step, unit, cfg, set }: { label: string; k: keyof Config; min: number; max: number; step: number; unit: string; cfg: Config; set: (p: Partial<Config>) => void }) {
  const v = cfg[k] as number;
  return (
    <label className="block">
      <div className="flex justify-between text-[11px] mb-1"><span className="text-muted-foreground">{label}</span><span className="mono text-foreground">{v}{unit}</span></div>
      <input type="range" min={min} max={max} step={step} value={v} data-testid={`slider-${k}`} onChange={(e) => set({ [k]: Number(e.target.value) } as Partial<Config>)} />
    </label>
  );
}

function Toggle({ label, k, cfg, set, hint }: { label: string; k: keyof Config; cfg: Config; set: (p: Partial<Config>) => void; hint?: string }) {
  const on = cfg[k] as boolean;
  return (
    <button onClick={() => set({ [k]: !on } as Partial<Config>)} data-testid={`toggle-${k}`} className="flex w-full items-center justify-between py-1 text-left group" title={hint}>
      <span className="text-[12px] text-muted-foreground group-hover:text-foreground transition-colors">{label}</span>
      <span className={`relative h-4 w-7 rounded-full transition-colors ${on ? 'bg-accent' : 'bg-muted border border-input'}`}>
        <span className={`absolute top-0.5 h-3 w-3 rounded-full bg-background transition-transform ${on ? 'translate-x-3.5' : 'translate-x-0.5'}`} />
      </span>
    </button>
  );
}

const f2 = (n: number | undefined, d = 2) => (n === undefined || !isFinite(n) ? '--' : n.toFixed(d));

const NODES: { name: string; pubs: string[] }[] = [
  { name: 'lidar_driver', pubs: ['/scan'] },
  { name: 'odometry', pubs: ['/odom'] },
  { name: 'mapper', pubs: ['/map'] },
  { name: 'obstacle_tracker', pubs: ['/tracked_obstacles'] },
  { name: 'goal_source', pubs: ['/goal_pose'] },
  { name: 'planner', pubs: ['/plan'] },
  { name: 'pursuit / DWA / monitor', pubs: ['/cmd_vel'] },
];

export default function Simulator() {
  const simRef = useRef<Simulation>(null as unknown as Simulation);
  if (!simRef.current) simRef.current = new Simulation(defaultConfig);
  const [cfg, setCfg] = useState<Config>(() => ({ ...simRef.current.config }));
  const [playing, setPlaying] = useState(true);
  const [speed, setSpeed] = useState(1);
  const [mode, setModeS] = useState<Mode>(simRef.current.mode);
  const [s, setS] = useState<Snap>(() => snap(simRef.current));
  const [seedDraft, setSeedDraft] = useState(String(cfg.seed));
  const playRef = useRef(playing); playRef.current = playing;
  const speedRef = useRef(speed); speedRef.current = speed;
  const keys = useRef(new Set<string>());

  // physics loop
  useEffect(() => {
    let id = 0, last = performance.now(), acc = 0;
    const loop = (now: number) => {
      const el = Math.min(0.25, (now - last) / 1000); last = now;
      if (playRef.current) {
        acc += el * speedRef.current;
        let n = 0;
        while (acc >= DT && n < MAX_STEPS * speedRef.current) { simRef.current.step(DT); acc -= DT; n++; }
        if (n >= MAX_STEPS * speedRef.current) acc = 0;
      } else acc = 0;
      id = requestAnimationFrame(loop);
    };
    id = requestAnimationFrame(loop);
    const iv = setInterval(() => setS(snap(simRef.current)), 125);
    return () => { cancelAnimationFrame(id); clearInterval(iv); };
  }, []);

  // manual drive
  useEffect(() => {
    const apply = () => {
      const k = keys.current;
      const fwd = (k.has('w') || k.has('arrowup') ? 1 : 0) - (k.has('s') || k.has('arrowdown') ? 1 : 0);
      const turn = (k.has('a') || k.has('arrowleft') ? 1 : 0) - (k.has('d') || k.has('arrowright') ? 1 : 0);
      simRef.current.setManual({ linear: fwd * simRef.current.config.maxSpeed, angular: turn * 1.6 });
    };
    const drive = ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'];
    const down = (e: KeyboardEvent) => {
      const t = e.target as HTMLElement;
      if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA')) return;
      const k = e.key.toLowerCase();
      if (simRef.current.mode !== 'manual' || !drive.includes(k)) return;
      e.preventDefault(); keys.current.add(k); apply();
    };
    const up = (e: KeyboardEvent) => { if (keys.current.delete(e.key.toLowerCase())) apply(); };
    const blur = () => { keys.current.clear(); simRef.current.setManual({ linear: 0, angular: 0 }); };
    window.addEventListener('keydown', down); window.addEventListener('keyup', up); window.addEventListener('blur', blur);
    return () => { window.removeEventListener('keydown', down); window.removeEventListener('keyup', up); window.removeEventListener('blur', blur); };
  }, []);

  const refresh = () => setS(snap(simRef.current));
  const update = useCallback((p: Partial<Config>) => {
    simRef.current.updateConfig(p);
    setCfg({ ...simRef.current.config });
  }, []);
  const resetWith = (p: Partial<Config> = {}) => {
    simRef.current.reset({ ...simRef.current.config, ...p });
    setCfg({ ...simRef.current.config });
    setModeS(simRef.current.mode);
    keys.current.clear();
    refresh();
  };
  const setMode = (m: Mode) => { simRef.current.setMode(m); setModeS(m); keys.current.clear(); simRef.current.setManual({ linear: 0, angular: 0 }); refresh(); };
  const pick = (p: Point) => {
    const w = simRef.current.world;
    if (p.x < 0 || p.y < 0 || p.x > w.width || p.y > w.height) return;
    if (simRef.current.mode !== 'goal') { simRef.current.setMode('goal'); setModeS('goal'); }
    simRef.current.setGoal(p); refresh();
  };
  const step = () => { setPlaying(false); simRef.current.step(DT); refresh(); };

  const safetyColor = s.safety === 'stop' ? 'hsl(var(--destructive))' : s.safety === 'slow' ? 'hsl(var(--primary))' : 'hsl(var(--accent))';
  const drift = Math.hypot(s.pose.x - s.odom.x, s.pose.y - s.odom.y);
  const m = s.metrics;
  const rateOf = (t: string) => s.rates.find((r) => r.topic === t);

  return (
    <div className="grid gap-3 xl:grid-cols-[1fr_300px]">
      <div className="flex flex-col gap-3 min-w-0">
        {/* transport bar */}
        <div className="panel flex flex-wrap items-center gap-2 px-3 py-2">
          <button className="btn btn-p w-[84px] justify-center" onClick={() => setPlaying((p) => !p)} data-testid="button-play">
            {playing ? <Pause size={14} /> : <Play size={14} />}{playing ? 'Pause' : 'Play'}
          </button>
          <button className="btn" onClick={step} data-testid="button-step"><StepForward size={14} />Step</button>
          <button className="btn" onClick={() => resetWith()} data-testid="button-reset"><RotateCcw size={14} />Reset</button>
          <div className="w-36"><Seg<number> id="speed" value={speed} onChange={setSpeed} options={[1, 2, 4].map((v) => ({ v, l: `${v}x` }))} /></div>
          <div className="w-[270px]">
            <Seg id="mode" value={mode} onChange={setMode} options={[
              { v: 'goal', l: <><Crosshair size={12} />goal</> },
              { v: 'explore', l: <><Compass size={12} />explore</> },
              { v: 'manual', l: <><Gamepad2 size={12} />manual</> },
            ]} />
          </div>
          <div className="ml-auto flex items-center gap-3 mono text-[11px]">
            <span className="text-muted-foreground">t <span className="text-foreground" data-testid="text-time">{f2(s.time, 1)}s</span></span>
            <span className="flex items-center gap-1.5" data-testid="status-safety" style={{ color: safetyColor }}>
              <span className="h-2 w-2 rounded-full live-dot" style={{ background: safetyColor }} />{s.safety.toUpperCase()}
            </span>
          </div>
        </div>

        <div className="panel px-3 py-2 flex items-center gap-3 text-[12px]">
          <span className="lbl shrink-0">status</span>
          <span className="mono truncate" data-testid="text-status">{s.status || 'idle'}</span>
          <span className="mono text-[11px] text-accent" data-testid="text-local-planner">{s.localPlannerState}</span>
          <span className="ml-auto text-muted-foreground text-[11px] shrink-0 hidden md:block">
            {mode === 'manual' ? 'Drive with W A S D or arrow keys' : 'Click either view to place a goal'}
          </span>
        </div>

        <div className="grid gap-3 md:grid-cols-2">
          <Canvas simRef={simRef} view="truth" onPick={pick} label="Ground truth" sub="world / true pose" />
          <Canvas simRef={simRef} view="belief" onPick={pick} label="Belief" sub="log-odds map / odom pose" />
        </div>

        <div className="flex flex-wrap gap-x-4 gap-y-1 px-1 text-[11px] text-muted-foreground">
          {[
            ['#f2b33d', 'robot / A* path'], ['#ffd98a', 'lookahead'], ['#5ee6d6', 'lidar hits'], ['#e2566b', 'dynamic / inflated'],
            ['#a98bff', 'frontiers'], ['#ff7a59', 'goal'], ['#41c7b9', 'odom ghost'],
            ['#4d5660', 'unknown'], ['#c8d2d7', 'free'], ['#131b22', 'occupied'],
            ['#47a7ff', 'estimated tracks / velocity'],
          ].map(([c, l]) => <span key={l} className="flex items-center gap-1.5"><span className="h-2 w-2 rounded-sm" style={{ background: c }} />{l}</span>)}
        </div>

        <div className="grid gap-3 lg:grid-cols-[1.1fr_1fr]">
          <Section title="Metrics">
            <div className="grid grid-cols-2 sm:grid-cols-4 gap-px bg-border rounded overflow-hidden">
              {[
                ['planned', f2(m.plannedLength), 'm'], ['traveled', f2(m.distance), 'm'], ['nav time', f2(m.time, 1), 's'], ['replans', String(m.replans ?? '--'), ''],
                ['min clear', f2(m.minClearance), 'm'], ['collisions', String(m.collisions ?? '--'), ''], ['explored', f2(m.explored ?? 0, 1), '%'], ['frontiers', String(s.frontiers), ''],
                ['static contacts', String(m.staticContacts ?? 0), ''], ['dynamic contacts', String(m.dynamicContacts ?? 0), ''],
                ['while moving', String(m.movingContacts ?? 0), ''], ['while stopped', String(m.stoppedContacts ?? 0), ''],
              ].map(([l, v, u]) => (
                <div key={l} className="bg-card p-2.5" data-testid={`metric-${l.replace(' ', '-')}`}>
                  <div className="lbl">{l}</div>
                  <div className={`mono text-lg leading-tight mt-0.5 ${l === 'collisions' && Number(v) > 0 ? 'text-destructive' : ''}`}>{v}<span className="text-[11px] text-muted-foreground ml-0.5">{u}</span></div>
                </div>
              ))}
            </div>
          </Section>

          <Section title="Pose: truth vs odometry">
            <table className="w-full mono text-[12px]">
              <thead><tr className="text-muted-foreground text-[10px]"><th className="text-left font-normal"></th><th className="text-right font-normal">x</th><th className="text-right font-normal">y</th><th className="text-right font-normal">theta</th></tr></thead>
              <tbody>
                <tr><td className="text-[#f2b33d]">truth</td><td className="text-right">{f2(s.pose.x)}</td><td className="text-right">{f2(s.pose.y)}</td><td className="text-right">{f2((s.pose.theta * 180) / Math.PI, 1)}°</td></tr>
                <tr><td className="text-[#41c7b9]">odom</td><td className="text-right">{f2(s.odom.x)}</td><td className="text-right">{f2(s.odom.y)}</td><td className="text-right">{f2((s.odom.theta * 180) / Math.PI, 1)}°</td></tr>
              </tbody>
            </table>
            <div className="mt-2 flex items-center justify-between text-[11px]">
              <span className="text-muted-foreground">position drift</span>
              <span className="mono" data-testid="text-drift" style={{ color: drift > 0.1 ? 'hsl(var(--destructive))' : 'hsl(var(--accent))' }}>{f2(drift, 3)} m</span>
            </div>
            <p className="mt-2 text-[11px] leading-relaxed text-muted-foreground">
              Mapping uses known poses: {cfg.odometryNoise ? 'odometry noise is on, so the map is built from the drifting odom estimate with no SLAM correction. Expect smeared walls.' : 'odometry is exact, so the belief map is registered against the true pose. This is mapping, not SLAM.'}
            </p>
          </Section>
        </div>

        <Section title="Node graph" right={<span className="lbl flex items-center gap-1.5"><span className="h-1.5 w-1.5 rounded-full bg-accent live-dot" />measured sim hz</span>}>
          <div className="flex flex-wrap items-stretch gap-2">
            {NODES.map((n, i) => (
              <div key={n.name} className="flex items-center gap-2">
                <div className="rounded border border-input bg-background px-2.5 py-2 min-w-[132px]" data-testid={`node-${n.name}`}>
                  <div className="mono text-[11px] text-foreground">{n.name}</div>
                  {n.pubs.map((t) => {
                    const r = rateOf(t);
                    return <div key={t} className="flex justify-between gap-3 mono text-[10px] mt-1"><span className="text-accent">{t}</span><span className="text-muted-foreground">{r ? `${r.rate.toFixed(1)} Hz` : '--'}</span></div>;
                  })}
                </div>
                {i < NODES.length - 1 && <span className="text-muted-foreground mono text-xs">→</span>}
              </div>
            ))}
          </div>
          <div className="mt-3 grid grid-cols-2 sm:grid-cols-3 lg:grid-cols-4 gap-1.5">
            {s.rates.length === 0 && <div className="text-[11px] text-muted-foreground col-span-full">No messages published yet. Press play.</div>}
            {s.rates.map((r) => (
              <div key={r.topic} className="flex justify-between rounded bg-muted/50 px-2 py-1 mono text-[10.5px]" data-testid={`rate-${r.topic}`}>
                <span className="text-accent truncate">{r.topic}</span>
                <span>{r.rate.toFixed(1)}<span className="text-muted-foreground"> Hz · {r.count}</span></span>
              </div>
            ))}
          </div>
        </Section>
      </div>

      <aside className="flex flex-col gap-3">
        <Section title="World">
          <Seg id="layout" value={cfg.layout} onChange={(l: Layout) => resetWith({ layout: l })} options={(['simple', 'office', 'warehouse'] as Layout[]).map((v) => ({ v, l: v }))} />
          <div className="mt-2.5 flex gap-2 items-center">
            <span className="lbl shrink-0">seed</span>
            <input className="num" type="number" value={seedDraft} data-testid="input-seed" onChange={(e) => setSeedDraft(e.target.value)}
              onKeyDown={(e) => { if (e.key === 'Enter') resetWith({ seed: Math.floor(Number(seedDraft)) || 0 }); }} />
            <button className="btn shrink-0" data-testid="button-apply-seed" onClick={() => resetWith({ seed: Math.floor(Number(seedDraft)) || 0 })}>Apply</button>
          </div>
        </Section>
        <Section title="Environment">
          <Toggle label="Moving obstacles" k="movingObstacles" cfg={cfg} set={update} />
          <div className="mt-2"><Slider label="Obstacle speed" k="obstacleSpeed" min={0} max={1.2} step={0.05} unit=" m/s" cfg={cfg} set={update} /></div>
          <div className="mt-2"><Toggle label="Odometry drift" k="odometryNoise" cfg={cfg} set={update} hint="Noisy wheel odometry; map is built from odom" /></div>
        </Section>
        <Section title="Lidar">
          <div className="space-y-2.5">
            <Slider label="Beams" k="beams" min={180} max={360} step={10} unit="" cfg={cfg} set={update} />
            <Slider label="Range" k="range" min={1} max={10} step={0.5} unit=" m" cfg={cfg} set={update} />
            <Slider label="Field of view" k="fov" min={60} max={360} step={10} unit="°" cfg={cfg} set={update} />
          </div>
        </Section>
        <Section title="Planning & control">
          <div className="space-y-2.5">
            <Slider label="Max speed" k="maxSpeed" min={0.2} max={1.5} step={0.05} unit=" m/s" cfg={cfg} set={update} />
            <Slider label="Pure pursuit lookahead" k="lookahead" min={0.2} max={1.5} step={0.05} unit=" m" cfg={cfg} set={update} />
            <Slider label="Inflation margin (+ 0.22 m robot)" k="inflation" min={0} max={0.5} step={0.01} unit=" m" cfg={cfg} set={update} />
            <p className="text-[10px] text-muted-foreground">Global route margin above. DWA uses a separate local footprint costmap with a 4 cm hard margin and scores global inflation as a soft cost.</p>
          </div>
        </Section>
        <Section title="Display">
          <Toggle label="Lidar rays" k="showLidar" cfg={cfg} set={update} />
          <Toggle label="Costmap inflation" k="showCostmap" cfg={cfg} set={update} />
        </Section>
        <div className="px-1 text-[10.5px] text-muted-foreground mono leading-relaxed">
          goal {s.goal ? `(${f2(s.goal.x)}, ${f2(s.goal.y)})` : 'none'} · path {s.pathPts} pts
        </div>
      </aside>
    </div>
  );
}
