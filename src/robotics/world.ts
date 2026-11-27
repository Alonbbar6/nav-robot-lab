import type { Layout, Point, Pose, Rect, Twist, World } from './types';
import { Random } from './random';
export const ROBOT_RADIUS = 0.22;
export const distance = (a: Point,b: Point) => Math.hypot(a.x-b.x,a.y-b.y);
export const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
export function rectClearance(p: Point, r: Rect): number {
  return Math.hypot(Math.max(r.x-p.x,0,p.x-r.x-r.w),Math.max(r.y-p.y,0,p.y-r.y-r.h));
}
export function clearance(world: World,p: Point,moving = true): number {
  let d = Math.min(p.x,p.y,world.width-p.x,world.height-p.y);
  for (const r of world.walls) d = Math.min(d,rectClearance(p,r));
  if (moving) for (const c of world.obstacles) d = Math.min(d,distance(p,c)-c.radius);
  return d - ROBOT_RADIUS;
}
export function makeWorld(layout: Layout, rng: Random): World {
  const walls: Rect[] = [
    {x:0,y:0,w:12,h:0.12},{x:0,y:8.88,w:12,h:0.12},
    {x:0,y:0,w:0.12,h:9},{x:11.88,y:0,w:0.12,h:9},
  ];
  if (layout === 'simple') walls.push({x:5,y:3,w:1.4,h:1},{x:8.5,y:6,w:1.7,h:1.3},{x:2,y:6,w:1.2,h:1});
  if (layout === 'office') walls.push(
    {x:4,y:0,w:0.16,h:3.5},{x:4,y:5,w:0.16,h:4},
    {x:8,y:0,w:0.16,h:2.4},{x:8,y:3.9,w:0.16,h:5.1},
    {x:0,y:5.5,w:1.7,h:0.16},{x:3,y:5.5,w:1,h:0.16},
    {x:5.4,y:6,w:1.3,h:1.5},{x:9.4,y:2,w:1.2,h:0.8},
    {x:1.5,y:7,w:1.3,h:0.75},{x:9.5,y:6.6,w:1.3,h:1});
  if (layout === 'warehouse') for (let x=3;x<=9;x+=3)
    for (let y=1.8;y<8;y+=3.2) walls.push({x,y,w:1.1,h:2});
  const world: World = {width:12,height:9,walls,obstacles:[]};
  for (let i=0;i<3;i++) {
    let p: Point = {x:6,y:4};
    for (let n=0;n<1000;n++) {
      p={x:0.8+rng.next()*10.4,y:0.8+rng.next()*7.4};
      if (clearance(world,p)>0.5) break;
    }
    const a=rng.next()*Math.PI*2;
    world.obstacles.push({...p,radius:0.23+i*0.025,vx:Math.cos(a),vy:Math.sin(a)});
  }
  return world;
}
export function moveObstacles(world: World,dt: number,speed: number): void {
  for (const c of world.obstacles) {
    const p={x:c.x+c.vx*speed*dt,y:c.y+c.vy*speed*dt};
    if (world.walls.some(r=>rectClearance(p,r)<c.radius+0.08)) { c.vx=-c.vx;c.vy=-c.vy; }
    else { c.x=p.x;c.y=p.y; }
  }
}
/** Exact unicycle integration with velocity and acceleration constraints. */
export function integrate(pose: Pose, velocity: Twist, command: Twist,dt: number,maxSpeed: number): { pose: Pose; velocity: Twist } {
  const approach=(a:number,b:number,delta:number)=>a+Math.max(-delta,Math.min(delta,b-a));
  const v=approach(velocity.linear,Math.max(-maxSpeed,Math.min(maxSpeed,command.linear)),0.9*dt);
  const w=approach(velocity.angular,Math.max(-2.2,Math.min(2.2,command.angular)),4*dt);
  const theta=pose.theta+w*dt;
  return {pose:{x:pose.x+(Math.abs(w)<1e-7?v*dt*Math.cos(pose.theta):v/w*(Math.sin(theta)-Math.sin(pose.theta))),
    y:pose.y+(Math.abs(w)<1e-7?v*dt*Math.sin(pose.theta):v/w*(Math.cos(pose.theta)-Math.cos(theta))),theta:wrap(theta)},
    velocity:{linear:v,angular:w}};
}
