import type { Simulation } from '../robotics/simulation';
import type { Point, Pose } from '../robotics/types';

export type View = 'truth' | 'belief';

export const C = {
  bg: '#0b1117',
  floor: '#101a22',
  grid: 'rgba(120,160,180,0.07)',
  gridMajor: 'rgba(120,160,180,0.14)',
  wall: '#4a5c6a',
  wallEdge: '#7f97a8',
  dyn: '#e2566b',
  robot: '#f2b33d',
  odom: '#41c7b9',
  lidar: 'rgba(65,199,185,0.22)',
  hit: '#5ee6d6',
  path: '#f2b33d',
  look: '#ffd98a',
  trail: 'rgba(160,140,230,0.7)',
  goal: '#ff7a59',
  frontier: '#a98bff',
};

export interface Frame { s: number; ox: number; oy: number; W: number; H: number }

export function frameFor(cw: number, ch: number, ww: number, wh: number): Frame {
  const pad = 14;
  const s = Math.min((cw - pad * 2) / ww, (ch - pad * 2) / wh);
  return { s, ox: (cw - ww * s) / 2, oy: (ch - wh * s) / 2, W: ww, H: wh };
}
const px = (f: Frame, x: number) => f.ox + x * f.s;
const py = (f: Frame, y: number) => f.oy + (f.H - y) * f.s;

export function pixelToWorld(f: Frame, cx: number, cy: number): Point {
  return { x: (cx - f.ox) / f.s, y: f.H - (cy - f.oy) / f.s };
}

const rasterCache = new WeakMap<HTMLCanvasElement, HTMLCanvasElement>();

function drawRaster(ctx: CanvasRenderingContext2D, host: HTMLCanvasElement, sim: Simulation, f: Frame) {
  const m = sim.map;
  let off = rasterCache.get(host);
  if (!off) { off = document.createElement('canvas'); rasterCache.set(host, off); }
  if (off.width !== m.width || off.height !== m.height) { off.width = m.width; off.height = m.height; }
  const octx = off.getContext('2d');
  if (!octx) return;
  const img = octx.createImageData(m.width, m.height);
  const d = img.data;
  const cost = sim.config.showCostmap ? sim.costmap : null;
  for (let j = 0; j < m.height; j++) {
    const row = m.height - 1 - j;
    for (let i = 0; i < m.width; i++) {
      const k = j * m.width + i;
      const o = (row * m.width + i) * 4;
      const l = m.data[k];
      let r: number, g: number, b: number;
      if (l > 0.7) { r = 19; g = 27; b = 34; }
      else if (l < -0.1) { const t = Math.min(1, -l / 2); r = 130 + 70 * t; g = 145 + 65 * t; b = 155 + 60 * t; }
      else { r = 77; g = 86; b = 96; }
      if (cost) {
        const c = cost[k];
        if (c >= 8 && c <= 100) { const t = 0.15 + 0.45 * (c / 100); r = r * (1 - t) + 226 * t; g = g * (1 - t) + 86 * t; b = b * (1 - t) + 107 * t; }
        else if (c === 255 && l <= 0.7) { r = 150; g = 60; b = 76; }
      }
      d[o] = r; d[o + 1] = g; d[o + 2] = b; d[o + 3] = 255;
    }
  }
  octx.putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = false;
  const x0 = px(f, m.origin.x), y1 = py(f, m.origin.y + m.height * m.resolution);
  ctx.drawImage(off, x0, y1, m.width * m.resolution * f.s, m.height * m.resolution * f.s);
}

function drawGrid(ctx: CanvasRenderingContext2D, f: Frame) {
  ctx.lineWidth = 1;
  for (let x = 0; x <= f.W + 1e-6; x += 0.5) {
    ctx.strokeStyle = Math.abs(x - Math.round(x)) < 1e-6 ? C.gridMajor : C.grid;
    ctx.beginPath(); ctx.moveTo(Math.round(px(f, x)) + 0.5, py(f, 0)); ctx.lineTo(Math.round(px(f, x)) + 0.5, py(f, f.H)); ctx.stroke();
  }
  for (let y = 0; y <= f.H + 1e-6; y += 0.5) {
    ctx.strokeStyle = Math.abs(y - Math.round(y)) < 1e-6 ? C.gridMajor : C.grid;
    ctx.beginPath(); ctx.moveTo(px(f, 0), Math.round(py(f, y)) + 0.5); ctx.lineTo(px(f, f.W), Math.round(py(f, y)) + 0.5); ctx.stroke();
  }
  ctx.fillStyle = 'rgba(140,170,190,0.45)';
  ctx.font = '9px JetBrains Mono, monospace';
  for (let x = 0; x <= f.W; x += 2) ctx.fillText(`${x}`, px(f, x) + 2, py(f, 0) - 3);
  for (let y = 2; y <= f.H; y += 2) ctx.fillText(`${y}`, px(f, 0) + 2, py(f, y) + 10);
}

function drawRobot(ctx: CanvasRenderingContext2D, f: Frame, p: Pose, color: string, ghost = false) {
  const r = 0.18 * f.s;
  const x = px(f, p.x), y = py(f, p.y);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-p.theta);
  ctx.globalAlpha = ghost ? 0.55 : 1;
  ctx.fillStyle = ghost ? 'transparent' : 'rgba(242,179,61,0.18)';
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.6;
  if (ghost) ctx.setLineDash([3, 3]);
  ctx.beginPath(); ctx.arc(0, 0, r, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = color;
  if (!ghost) { ctx.fillRect(-r * 0.35, -r - 2, r * 0.7, 3); ctx.fillRect(-r * 0.35, r - 1, r * 0.7, 3); }
  ctx.beginPath(); ctx.moveTo(r * 1.35, 0); ctx.lineTo(r * 0.3, -r * 0.45); ctx.lineTo(r * 0.3, r * 0.45); ctx.closePath(); ctx.fill();
  ctx.restore();
}

function polyline(ctx: CanvasRenderingContext2D, f: Frame, pts: Point[]) {
  ctx.beginPath();
  pts.forEach((p, i) => (i ? ctx.lineTo(px(f, p.x), py(f, p.y)) : ctx.moveTo(px(f, p.x), py(f, p.y))));
  ctx.stroke();
}

export function render(canvas: HTMLCanvasElement, sim: Simulation, view: View, hover: Point | null, t: number): Frame {
  const dpr = window.devicePixelRatio || 1;
  const cw = canvas.clientWidth, ch = canvas.clientHeight;
  if (canvas.width !== Math.round(cw * dpr) || canvas.height !== Math.round(ch * dpr)) {
    canvas.width = Math.round(cw * dpr); canvas.height = Math.round(ch * dpr);
  }
  const ctx = canvas.getContext('2d')!;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = C.bg; ctx.fillRect(0, 0, cw, ch);
  const w = sim.world;
  const f = frameFor(cw, ch, w.width, w.height);

  if (view === 'truth') {
    ctx.fillStyle = C.floor; ctx.fillRect(px(f, 0), py(f, w.height), w.width * f.s, w.height * f.s);
    drawGrid(ctx, f);
    for (const r of w.walls) {
      ctx.fillStyle = C.wall; ctx.strokeStyle = C.wallEdge; ctx.lineWidth = 1;
      ctx.fillRect(px(f, r.x), py(f, r.y + r.h), r.w * f.s, r.h * f.s);
      ctx.strokeRect(px(f, r.x) + 0.5, py(f, r.y + r.h) + 0.5, r.w * f.s - 1, r.h * f.s - 1);
    }
  } else {
    drawRaster(ctx, canvas, sim, f);
    drawGrid(ctx, f);
  }

  for (const o of sim.config.movingObstacles ? w.obstacles : []) {
    if (view === 'belief') continue;
    ctx.fillStyle = 'rgba(226,86,107,0.25)'; ctx.strokeStyle = C.dyn; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(px(f, o.x), py(f, o.y), o.radius * f.s, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(px(f, o.x), py(f, o.y)); ctx.lineTo(px(f, o.x + o.vx * 0.8), py(f, o.y + o.vy * 0.8)); ctx.stroke();
  }

  // Raw sensor geometry in truth view; estimated scan placement in belief view.
  const scan = sim.scan;
  if (scan && sim.config.showLidar) {
    const o = view === 'truth' ? sim.pose : sim.odom;
    ctx.lineWidth = 1;
    ctx.strokeStyle = C.lidar;
    ctx.beginPath();
    const hits: Point[] = [];
    for (let i = 0; i < scan.ranges.length; i++) {
      const r = scan.ranges[i];
      if (!isFinite(r)) continue;
      const a = o.theta + scan.angle_min + i * scan.angle_increment;
      const rr = Math.min(r, scan.range_max);
      const hx = o.x + Math.cos(a) * rr, hy = o.y + Math.sin(a) * rr;
      ctx.moveTo(px(f, o.x), py(f, o.y)); ctx.lineTo(px(f, hx), py(f, hy));
      if (r < scan.range_max - 1e-3) hits.push({ x: hx, y: hy });
    }
    ctx.stroke();
    ctx.fillStyle = C.hit;
    for (const h of hits) ctx.fillRect(px(f, h.x) - 1.2, py(f, h.y) - 1.2, 2.4, 2.4);
  }

  if(view==='belief')for(const t of sim.tracks){
    const age=Math.max(0,sim.time-t.stamp),x=t.x+t.vx*age,y=t.y+t.vy*age;
    const ax=px(f,x),ay=py(f,y),bx=px(f,x+t.vx),by=py(f,y+t.vy);
    ctx.strokeStyle='#47a7ff';ctx.fillStyle='rgba(71,167,255,0.12)';ctx.lineWidth=1.5;
    ctx.setLineDash(t.missed>0?[3,3]:[]);
    ctx.beginPath();ctx.arc(ax,ay,t.radius*f.s,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.setLineDash([]);
    ctx.beginPath();ctx.moveTo(ax,ay);ctx.lineTo(bx,by);ctx.stroke();
    if(Math.hypot(bx-ax,by-ay)>2){
      const a=Math.atan2(by-ay,bx-ax);
      ctx.beginPath();ctx.moveTo(bx-5*Math.cos(a-0.5),by-5*Math.sin(a-0.5));ctx.lineTo(bx,by);ctx.lineTo(bx-5*Math.cos(a+0.5),by-5*Math.sin(a+0.5));ctx.stroke();
    }
    ctx.fillStyle='#47a7ff';ctx.font='9px monospace';
    ctx.fillText(`#${t.id} ${Math.hypot(t.vx,t.vy).toFixed(2)} m/s`,ax+5,ay-t.radius*f.s-4);
  }
  if (sim.trail.length > 1) { ctx.strokeStyle = C.trail; ctx.lineWidth = 1.4; ctx.setLineDash([2, 3]); polyline(ctx, f, sim.trail); ctx.setLineDash([]); }

  for (const fr of sim.frontiers) {
    ctx.fillStyle = C.frontier;
    ctx.beginPath(); ctx.arc(px(f, fr.x), py(f, fr.y), 2.6, 0, Math.PI * 2); ctx.fill();
  }

  if (sim.path.poses.length > 1) {
    ctx.strokeStyle = 'rgba(242,179,61,0.25)'; ctx.lineWidth = 6; polyline(ctx, f, sim.path.poses);
    ctx.strokeStyle = C.path; ctx.lineWidth = 2; polyline(ctx, f, sim.path.poses);
  }

  if (sim.lookahead) {
    const l = sim.lookahead;
    ctx.strokeStyle = 'rgba(255,217,138,0.5)'; ctx.setLineDash([4, 4]); ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(px(f, sim.odom.x), py(f, sim.odom.y)); ctx.lineTo(px(f, l.x), py(f, l.y)); ctx.stroke(); ctx.setLineDash([]);
    ctx.strokeStyle = C.look; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(px(f, l.x), py(f, l.y), 5, 0, Math.PI * 2); ctx.stroke();
  }

  if (sim.goal) {
    const g = sim.goal, gx = px(f, g.x), gy = py(f, g.y);
    const pulse = 8 + 4 * (0.5 + 0.5 * Math.sin(t * 4));
    ctx.strokeStyle = C.goal; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(gx, gy, pulse, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(gx - 12, gy); ctx.lineTo(gx + 12, gy); ctx.moveTo(gx, gy - 12); ctx.lineTo(gx, gy + 12); ctx.stroke();
  }

  if (view === 'truth') {
    if (sim.config.odometryNoise) drawRobot(ctx, f, sim.odom, C.odom, true);
    drawRobot(ctx, f, sim.pose, C.robot);
  } else {
    drawRobot(ctx, f, sim.odom, C.robot);
  }

  if (hover && hover.x >= 0 && hover.y >= 0 && hover.x <= w.width && hover.y <= w.height) {
    const hx = px(f, hover.x), hy = py(f, hover.y);
    ctx.strokeStyle = 'rgba(255,122,89,0.6)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(hx, hy, 6, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = 'rgba(230,240,245,0.85)'; ctx.font = '10px JetBrains Mono, monospace';
    ctx.fillText(`${hover.x.toFixed(2)}, ${hover.y.toFixed(2)} m`, hx + 9, hy - 8);
  }
  return f;
}
