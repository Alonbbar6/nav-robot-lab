import type { Config, LaserScan, Point, Pose, Rect, World } from './types';
import { Random } from './random';
/** Slab intersection, returning first positive entry into an axis-aligned rectangle. */
export function rayRect(origin: Point,dx: number,dy: number,rect: Rect,maxRange: number): number {
  let near=0,far=maxRange;
  for (const [o,d,min,max] of [[origin.x,dx,rect.x,rect.x+rect.w],[origin.y,dy,rect.y,rect.y+rect.h]]) {
    if (Math.abs(d)<1e-10) { if (o<min||o>max) return maxRange; }
    else { const a=(min-o)/d,b=(max-o)/d; near=Math.max(near,Math.min(a,b));far=Math.min(far,Math.max(a,b)); }
  }
  return far>=near ? near : maxRange;
}
export function sense(world: World,pose: Pose,config: Config,rng: Random): LaserScan {
  const fov=config.fov*Math.PI/180;
  const scan:LaserScan={angle_min:-fov/2,angle_max:fov/2,angle_increment:fov/Math.max(1,config.beams-1),range_min:0.04,range_max:config.range,ranges:[]};
  for (let i=0;i<config.beams;i++) {
    const a=pose.theta+scan.angle_min+i*scan.angle_increment,dx=Math.cos(a),dy=Math.sin(a);
    let range=config.range;
    for (const r of world.walls) range=Math.min(range,rayRect(pose,dx,dy,r,range));
    if (config.movingObstacles) for (const c of world.obstacles) {
      const ox=pose.x-c.x,oy=pose.y-c.y,b=ox*dx+oy*dy,disc=b*b-(ox*ox+oy*oy-c.radius*c.radius);
      if (disc>=0) { const t=-b-Math.sqrt(disc); if (t>=0) range=Math.min(range,t); }
    }
    scan.ranges.push(range>=config.range ? config.range : Math.max(scan.range_min,Math.min(config.range,range+rng.gaussian()*0.008)));
  }
  return scan;
}
