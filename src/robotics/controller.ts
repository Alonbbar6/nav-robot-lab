import type { Path, Point, Pose, Twist } from './types';
import { distance, wrap } from './world';
export function purePursuit(pose:Pose,path:Path,lookahead:number,maxSpeed:number,clearance=Infinity):{command:Twist;target:Point|null} {
  if(!path.poses.length)return {command:{linear:0,angular:0},target:null};
  let nearest=0;for(let i=1;i<path.poses.length;i++)if(distance(pose,path.poses[i])<distance(pose,path.poses[nearest]))nearest=i;
  let target=path.poses[path.poses.length-1],remaining=lookahead+maxSpeed*0.25;
  // Project onto path segments then walk forward, rather than chasing vertices.
  let best=Infinity,segment=0,projection:Point=path.poses[0];
  for(let i=0;i<path.poses.length-1;i++){
    const a=path.poses[i],b=path.poses[i+1],dx=b.x-a.x,dy=b.y-a.y;
    const t=Math.max(0,Math.min(1,((pose.x-a.x)*dx+(pose.y-a.y)*dy)/(dx*dx+dy*dy||1)));
    const p={x:a.x+t*dx,y:a.y+t*dy},d=distance(pose,p);
    if(d<best){best=d;segment=i;projection=p;}
  }
  for(let i=segment;i<path.poses.length-1;i++){
    const a=i===segment?projection:path.poses[i],b=path.poses[i+1],d=distance(a,b);
    if(d>=remaining){const t=remaining/d;target={x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t};break;}remaining-=d;
  }
  const error=wrap(Math.atan2(target.y-pose.y,target.x-pose.x)-pose.theta);
  if(Math.abs(error)>0.8)return {command:{linear:0,angular:Math.max(-1.8,Math.min(1.8,2.5*error))},target};
  const curvature=2*Math.sin(error)/Math.max(0.1,distance(pose,target));
  const goalDistance=distance(pose,path.poses[path.poses.length-1]);
  const linear=Math.min(maxSpeed/(1+1.2*Math.abs(curvature)),Math.max(0.08,goalDistance*1.2),Math.max(0.06,clearance*1.4));
  return {command:{linear,angular:Math.max(-2.2,Math.min(2.2,linear*curvature))},target};
}
