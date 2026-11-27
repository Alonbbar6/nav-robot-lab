import type { OccupancyGrid, Point, Pose, TrackedObstacle, Twist } from './types';
import { cellOf, inGrid } from './mapper';
import { distance, integrate, ROBOT_RADIUS } from './world';

export interface LocalPlannerInput {
  pose: Pose; velocity: Twist; preferred: Twist; target: Point | null;
  map: OccupancyGrid; costmap: Uint8Array; tracks: TrackedObstacle[];
  inflationCosts?: Uint8Array;
  maxSpeed: number; stamp: number;
}
export interface TrajectoryEvaluation {
  safe: boolean; clearance: number; progress: number; end: Pose; collisionTime: number;
}
const HORIZON=1.5, INTERVAL=0.1, MARGIN=0.06;
/** Test inflated static costmap at robot center (footprint is ALREADY inflated),
 * and circular swept footprint against constant-velocity tracks. No truth inputs. */
export function evaluateTrajectory(input:LocalPlannerInput,command:Twist):TrajectoryEvaluation {
  let p={...input.pose},v={...input.velocity},minimum=2,collisionTime=Infinity;
  const initialDistance=input.target?distance(p,input.target):0;
  for(let t=INTERVAL;t<=HORIZON+1e-8;t+=INTERVAL){
    const before=p;
    ({pose:p,velocity:v}=integrate(p,v,command,INTERVAL,input.maxSpeed));
    // Half-step checks bound translational sampling gaps to < 8 cm at 1.5 m/s.
    for(const f of [0.5,1]){
      const q={x:before.x+(p.x-before.x)*f,y:before.y+(p.y-before.y)*f};
      const c=cellOf(input.map,q);
      if(!inGrid(input.map,c.x,c.y)||input.costmap[c.y*input.map.width+c.x]===255)
        return {safe:false,clearance:Math.min(minimum,-MARGIN),progress:0,end:p,collisionTime:t};
    }
    for(const track of input.tracks){
      const age=Math.max(0,input.stamp-track.stamp);
      const uncertainty=0.03+0.025*t+Math.min(0.15,track.missed*0.15);
      const q={x:track.x+track.vx*(t+age),y:track.y+track.vy*(t+age)};
      const separation=distance(p,q)-ROBOT_RADIUS-track.radius-uncertainty;
      minimum=Math.min(minimum,separation);
      if(separation<MARGIN){collisionTime=t;return {safe:false,clearance:minimum,progress:0,end:p,collisionTime};}
    }
  }
  return {safe:true,clearance:minimum,progress:input.target?initialDistance-distance(p,input.target):-distance(input.pose,p),end:p,collisionTime};
}
/** DWB-style acceleration-limited rollout generator: sample velocities reachable
 * within the horizon, simulate the acceleration ramp, emit only the first
 * controller-period velocity. Sampling only +/- one tick and then holding that
 * tiny velocity for 1.5s falsely labels every escape from rest as impossible. */
export function dynamicWindow(input:LocalPlannerInput):{command:Twist;trajectoryCommand:Twist;safe:boolean;clearance:number;rejected:number} {
  const dt=0.05,dv=0.9*dt,dw=4*dt;
  const lo=Math.max(-0.3,input.velocity.linear-0.9*HORIZON),hi=Math.min(input.maxSpeed,input.velocity.linear+0.9*HORIZON);
  const wl=Math.max(-2.2,input.velocity.angular-4*HORIZON),wh=Math.min(2.2,input.velocity.angular+4*HORIZON);
  const clip=(n:number,a:number,b:number)=>Math.max(a,Math.min(b,n));
  const vs=[lo,clip(lo/2,lo,hi),clip(hi/2,lo,hi),hi,clip(0,lo,hi),clip(input.preferred.linear,lo,hi),clip(input.velocity.linear,lo,hi)];
  const ws=[wl,wl/2,wh/2,wh,clip(0,wl,wh),clip(input.preferred.angular,wl,wh)];
  const threatened=input.tracks.some(t=>{
    const age=Math.max(0,input.stamp-t.stamp),x=t.x+t.vx*age-input.pose.x,y=t.y+t.vy*age-input.pose.y;
    const speed2=t.vx*t.vx+t.vy*t.vy;
    const closest=speed2>0.01?clip(-(x*t.vx+y*t.vy)/speed2,0,HORIZON):0;
    return speed2>0.01&&Math.hypot(x+t.vx*closest,y+t.vy*closest)-ROBOT_RADIUS-t.radius<0.8;
  });
  let best=-Infinity,result:Twist={linear:0,angular:0},clearance=-Infinity,rejected=0;
  for(const linear of new Set(vs))for(const angular of new Set(ws)){
    const command={linear,angular},e=evaluateTrajectory(input,command);
    if(!e.safe){rejected++;continue;}
    const preference=0.55*Math.abs(linear-input.preferred.linear)+0.16*Math.abs(angular-input.preferred.angular);
    let terminalClearance=1.2;
    if(threatened)for(const t of input.tracks){
      const future=HORIZON+Math.max(0,input.stamp-t.stamp);
      terminalClearance=Math.min(terminalClearance,Math.hypot(e.end.x-t.x-t.vx*future,e.end.y-t.y-t.vy*future)-ROBOT_RADIUS-t.radius);
    }
    // Under threat, escaping toward clear space beats stopping exactly at a
    // lookahead/goal point. Minimum clearance alone is dominated by t≈0 and
    // otherwise rewards freezing in front of an approaching person.
    const endCell=cellOf(input.map,e.end);
    const inflationCost=input.inflationCosts?.[endCell.y*input.map.width+endCell.x]??0;
    const score=(input.target?(threatened?0.45:2.5):0.15)*e.progress-preference+
      (threatened?4:0.65)*Math.min(1,e.clearance)+(threatened?1.8*terminalClearance:0)-Math.min(0.4,inflationCost*0.004);
    if(score>best){best=score;result=command;clearance=e.clearance;}
  }
  // No feasible velocity: bounded braking, rather than returning a known-colliding
  // trajectory as "safe". The independent monitor can emergency-brake if needed.
  const command={linear:clip(result.linear,input.velocity.linear-dv,input.velocity.linear+dv),
    angular:clip(result.angular,input.velocity.angular-dw,input.velocity.angular+dw)};
  return {command,trajectoryCommand:result,safe:best!==-Infinity,clearance,rejected};
}
