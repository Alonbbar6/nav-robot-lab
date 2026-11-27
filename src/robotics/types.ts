export interface Point { x: number; y: number }
export interface Pose extends Point { theta: number }
export interface Twist { linear: number; angular: number }
export interface LaserScan { angle_min: number; angle_max: number; angle_increment: number; range_min: number; range_max: number; ranges: number[] }
export interface Odometry { pose: Pose; twist: Twist; stamp: number }
export interface OccupancyGrid { resolution: number; width: number; height: number; origin: Point; data: Float32Array }
export interface Path { poses: Point[]; length: number }
export interface Rect extends Point { w: number; h: number }
export interface MovingObstacle extends Point { radius: number; vx: number; vy: number }
/** Estimated in odom coordinates; never populated from ground truth. */
export interface TrackedObstacle extends Point {
  id: number; vx: number; vy: number; radius: number; stamp: number;
  age: number; missed: number; moving: boolean;
}
export interface TrackedObstacles { stamp: number; tracks: TrackedObstacle[] }
export interface ContactEvent {
  stamp: number; kind: 'static' | 'dynamic'; obstacle: string;
  moving: boolean; linear: number; angular: number;
}
export interface ContactCounts {
  staticContacts: number; dynamicContacts: number;
  movingContacts: number; stoppedContacts: number;
}
export interface World { width: number; height: number; walls: Rect[]; obstacles: MovingObstacle[] }
export type Layout = 'simple' | 'office' | 'warehouse';
export type Mode = 'goal' | 'explore' | 'manual';
export interface Config {
  layout: Layout; seed: number; movingObstacles: boolean; odometryNoise: boolean;
  showLidar: boolean; showCostmap: boolean; beams: number; range: number; fov: number;
  maxSpeed: number; lookahead: number; inflation: number; obstacleSpeed: number;
}
export const defaultConfig: Config = {
  layout: 'office', seed: 42, movingObstacles: true, odometryNoise: false,
  showLidar: true, showCostmap: true, beams: 180, range: 6, fov: 360,
  maxSpeed: 0.8, lookahead: 0.6, inflation: 0.12, obstacleSpeed: 0.35,
};
export function normalizeConfig(input: Partial<Config>): Config {
  const c={...defaultConfig,...input};
  const clamp=(n:number,min:number,max:number,fallback:number)=>Number.isFinite(n)?Math.max(min,Math.min(max,n)):fallback;
  return {...c,seed:Math.trunc(clamp(c.seed,0,4294967295,42)),
    beams:Math.round(clamp(c.beams,180,360,180)),range:clamp(c.range,1,10,6),
    fov:clamp(c.fov,60,360,360),maxSpeed:clamp(c.maxSpeed,0.1,1.5,0.8),
    lookahead:clamp(c.lookahead,0.2,2,0.6),inflation:clamp(c.inflation,0,0.6,0.12),
    obstacleSpeed:clamp(c.obstacleSpeed,0,1.5,0.35)};
}
export interface Metrics extends ContactCounts { plannedLength: number; distance: number; time: number; replans: number; minClearance: number; collisions: number; explored: number; contacts: ContactEvent[] }
export interface BenchmarkRow extends ContactCounts { trial: number; seed: number; success: boolean; time: number; distance: number; straightLine: number; ratio: number; replans: number; collisions: number; contacts: ContactEvent[] }
export interface BenchmarkSummary extends ContactCounts { successRate: number; meanTime: number; medianTime: number; meanRatio: number; meanReplans: number; totalCollisions: number; trialsWithContacts: number }
