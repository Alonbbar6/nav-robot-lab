import type { LaserScan, Pose, TrackedObstacle, Twist } from './types';
import { ROBOT_RADIUS } from './world';

/** Earliest circle-circle contact under relative constant velocity. */
export function timeToCollision(rx:number,ry:number,vx:number,vy:number,radius:number):number {
  const c=rx*rx+ry*ry-radius*radius;
  if(c<=0)return 0;
  const a=vx*vx+vy*vy,b=rx*vx+ry*vy,disc=b*b-a*c;
  if(a<1e-10||b>=0||disc<0)return Infinity;
  return Math.max(0,(-b-Math.sqrt(disc))/a);
}
/** 360-degree circular footprint monitor. Raw scan hits cover untracked objects;
 * tracks supply relative-velocity TTC, including hazards from side/rear.
 * Emergency braking must not cancel an evasive command that increases TTC. */
export function collisionMonitor(
  scan:LaserScan|null,cmd:Twist,pose:Pose={x:0,y:0,theta:0},
  tracks:TrackedObstacle[]=[],stamp=0,
):{command:Twist;state:'clear'|'slow'|'stop';clearance:number;ttc:number} {
  let nearest=Infinity,ttc=Infinity,stopTtc=Infinity,allAround=Infinity;
  const vx=cmd.linear*Math.cos(pose.theta),vy=cmd.linear*Math.sin(pose.theta);
  if(scan)scan.ranges.forEach((r,i)=>{
    if(!Number.isFinite(r)||r>=scan.range_max-0.02)return;
    const a=scan.angle_min+i*scan.angle_increment,x=r*Math.cos(a),y=r*Math.sin(a);
    allAround=Math.min(allAround,r-ROBOT_RADIUS);
    // Any beam can intersect the swept circle; there is no front-only filter.
    const hit=timeToCollision(x,y,-cmd.linear,0,ROBOT_RADIUS+0.07);
    // Points associated with tracks are handled by dynamic prediction, not frozen.
    const wx=pose.x+Math.cos(pose.theta)*x-Math.sin(pose.theta)*y;
    const wy=pose.y+Math.sin(pose.theta)*x+Math.cos(pose.theta)*y;
    const dynamic=tracks.some(t=>Math.hypot(wx-t.x-t.vx*(stamp-t.stamp),wy-t.y-t.vy*(stamp-t.stamp))<t.radius+0.12);
    if(!dynamic){
      ttc=Math.min(ttc,hit);
      const toward=cmd.linear>=0?x:-x;
      if(toward>0&&Math.abs(y)<ROBOT_RADIUS+0.08)nearest=Math.min(nearest,toward-ROBOT_RADIUS);
    }
  });
  for(const t of tracks){
    const age=Math.max(0,stamp-t.stamp),x=t.x+t.vx*age-pose.x,y=t.y+t.vy*age-pose.y,radius=ROBOT_RADIUS+t.radius+0.07;
    ttc=Math.min(ttc,timeToCollision(x,y,t.vx-vx,t.vy-vy,radius));
    stopTtc=Math.min(stopTtc,timeToCollision(x,y,t.vx,t.vy,radius));
  }
  const closingNear=nearest<0.13&&Math.abs(cmd.linear)>0.001;
  // Stop only if braking is no worse than continuing; allow safe reverse/escape.
  const danger=ttc<0.35&&stopTtc>=ttc;
  const state=closingNear||danger?'stop':ttc<0.9||nearest<0.45?'slow':'clear';
  // Do not scale a track-aware evasive trajectory: that can turn a safe escape
  // into a collision. Slow scaling is reserved for static/scan-only hazards.
  const scale=state==='stop'?0:state==='slow'&&!tracks.length?Math.max(0.1,Math.min(1,(nearest-0.13)/0.32)):1;
  return {command:{linear:scale===0?0:cmd.linear*scale,angular:cmd.angular},state,clearance:Math.min(nearest,allAround),ttc};
}
