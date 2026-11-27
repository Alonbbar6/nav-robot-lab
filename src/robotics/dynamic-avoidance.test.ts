import { describe, expect, it } from 'vitest';
import { ObstacleTracker } from './tracker';
import { createMap } from './mapper';
import { sense } from './lidar';
import { Random } from './random';
import { defaultConfig, type TrackedObstacle } from './types';
import { dynamicWindow, evaluateTrajectory, type LocalPlannerInput } from './dwa';
import { collisionMonitor } from './safety';
import { Simulation } from './simulation';
import { clearance } from './world';
import { summarize } from './metrics';
import { trialResult } from './benchmark';
import { inflationEscape } from './recovery';

const track=(partial:Partial<TrackedObstacle>={}):TrackedObstacle=>({
  id:1,x:3.3,y:2,vx:-0.35,vy:0,radius:0.25,stamp:0,age:1,missed:0,moving:true,...partial,
});
const input=():LocalPlannerInput=>{
  const map=createMap();
  return {pose:{x:2,y:2,theta:0},velocity:{linear:0,angular:0},preferred:{linear:0.8,angular:0},
    target:{x:5,y:2},map,costmap:new Uint8Array(map.data.length).fill(1),tracks:[track()],maxSpeed:0.8,stamp:0};
};
describe('scan-derived obstacle tracker',()=>{
  it('estimates a moving cluster velocity to within 0.15 m/s',()=>{
    const tracker=new ObstacleTracker(),map=createMap(),pose={x:2,y:2,theta:0};
    let tracks:TrackedObstacle[]=[];
    for(let i=0;i<=20;i++){
      const world={width:12,height:9,walls:[],obstacles:[{x:4+i*0.04,y:3,radius:0.28,vx:1,vy:0}]};
      const scan=sense(world,pose,{...defaultConfig,beams:360},new Random(i+1));
      tracks=tracker.update(scan,pose,map,i*0.1).tracks;
    }
    const moving=tracks.find(t=>t.moving);
    expect(moving).toBeDefined();
    expect(moving!.vx).toBeCloseTo(0.4,0);
    expect(Math.abs(moving!.vx-0.4)).toBeLessThan(0.15);
    expect(Math.abs(moving!.vy)).toBeLessThan(0.15);
    expect(moving!.radius).toBeGreaterThan(0.2);
  });
  it('does not create tracks from confidently occupied static returns',()=>{
    const map=createMap();map.data.fill(4);
    const pose={x:2,y:2,theta:0};
    const scan=sense({width:12,height:9,walls:[{x:4,y:0,w:0.2,h:8}],obstacles:[]},pose,defaultConfig,new Random(1));
    expect(new ObstacleTracker().update(scan,pose,map,0).tracks).toHaveLength(0);
  });
});
describe('dynamic window trajectory selection',()=>{
  it('rejects a predicted collision and selects an acceleration-reachable safe command',()=>{
    const args=input();
    expect(evaluateTrajectory(args,{linear:0.8,angular:0}).safe).toBe(false);
    const selected=dynamicWindow(args);
    expect(selected.safe).toBe(true);
    expect(evaluateTrajectory(args,selected.trajectoryCommand).safe).toBe(true);
    expect(Math.abs(selected.command.linear-args.velocity.linear)).toBeLessThanOrEqual(0.9*0.05+1e-8);
    expect(Math.abs(selected.command.angular-args.velocity.angular)).toBeLessThanOrEqual(4*0.05+1e-8);
  });
  it('rejects lethal static costmap cells',()=>{
    const args=input();args.tracks=[];
    for(let y=0;y<args.map.height;y++)args.costmap[y*args.map.width+50]=255;
    expect(evaluateTrajectory(args,{linear:0.8,angular:0}).safe).toBe(false);
  });
  it('can choose reverse motion to evade a closing obstacle',()=>{
    const args=input();args.tracks=[track({x:3.15,vx:-0.3})];args.preferred={linear:0,angular:0};args.target=null;
    const chosen=dynamicWindow(args);
    expect(chosen.safe).toBe(true);
    expect(chosen.command.linear).toBeLessThan(0);
  });
});
describe('360-degree safety monitor',()=>{
  it('stops reverse travel when a hit is behind the robot',()=>{
    const scan={angle_min:Math.PI,angle_max:Math.PI,angle_increment:0,range_min:0.04,range_max:6,ranges:[0.3]};
    const safe=collisionMonitor(scan,{linear:-0.2,angular:0});
    expect(safe.state).toBe('stop');expect(safe.command.linear).toBe(0);
  });
  it('uses side-approach velocity, not a front-only zone',()=>{
    const side=track({x:0,y:0.8,vx:0,vy:-0.5});
    const safe=collisionMonitor(null,{linear:0,angular:0},{x:0,y:0,theta:0},[side]);
    expect(safe.ttc).toBeLessThan(0.9);
    expect(safe.state).not.toBe('clear');
  });
  it('does not brake a safer reverse escape into an oncoming obstacle',()=>{
    const args=input(),coming=track({x:2.7,vx:-0.4});
    const result=collisionMonitor(null,{linear:-0.25,angular:0},args.pose,[coming]);
    expect(result.command.linear).toBeLessThan(0);
  });
});
describe('physics and contact accounting',()=>{
  it('finds a collision-free recovery pose when global inflation grows under the robot',()=>{
    const map=createMap(3,3,0.1);map.data.fill(-4);
    const local=new Uint8Array(map.data.length).fill(1),global=new Uint8Array(local.length).fill(1);
    for(let y=0;y<map.height;y++)for(let x=12;x<18;x++)global[y*map.width+x]=255;
    const point=inflationEscape(map,global,local,{x:1.5,y:1.5});
    expect(point).not.toBeNull();
    expect(Math.abs(point!.x-1.5)).toBeGreaterThan(0.25);
    local.fill(255);
    expect(inflationEscape(map,global,local,{x:1.5,y:1.5})).toBeNull();
  });
  it('never overlaps static walls during a two-minute commanded drive',()=>{
    const sim=new Simulation({movingObstacles:false,layout:'office',seed:7});
    sim.setMode('manual');
    for(let tick=0;tick<120*60;tick++){
      sim.setManual({linear:0.8,angular:tick%1800<1200?0:0.7});
      sim.step();
      expect(clearance(sim.world,sim.pose,false)).toBeGreaterThanOrEqual(-1e-7);
    }
    expect(sim.time).toBeCloseTo(120);
  },30000);
  it('keeps patrollers blind to the robot pose',()=>{
    const a=new Simulation({seed:7},{x:1,y:1,theta:0});
    const b=new Simulation({seed:7},{x:10,y:7,theta:0});
    for(let tick=0;tick<60;tick++){a.step();b.step();}
    expect(a.world.obstacles).toEqual(b.world.obstacles);
  });
  it('includes contact categories and per-event motion in benchmark results',()=>{
    const sim=new Simulation({seed:7});
    sim.metrics.collisions=1;sim.metrics.dynamicContacts=1;sim.metrics.stoppedContacts=1;
    sim.metrics.contacts=[{stamp:1,kind:'dynamic',obstacle:'moving:0',moving:false,linear:0,angular:0}];
    const row=trialResult(sim,1,7,5),summary=summarize([row]);
    expect(summary.dynamicContacts).toBe(1);expect(summary.staticContacts).toBe(0);
    expect(summary.stoppedContacts).toBe(1);expect(summary.trialsWithContacts).toBe(1);
    expect(row.contacts[0].moving).toBe(false);
  });
});
