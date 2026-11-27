import type { BenchmarkRow, Config, Point, Pose } from './types';
import { Simulation } from './simulation';
import { Random } from './random';
import { clearance, distance, makeWorld, ROBOT_RADIUS } from './world';
import { createMap, pointOf } from './mapper';
import { inflate } from './costmap';
import { astar } from './planner';
/** Ground truth is used ONLY to sample statically reachable pairs, never fed to navigation. */
export function createTrial(config:Config,trial:number):{sim:Simulation;straightLine:number;seed:number} {
  const seed=(config.seed+trial*7919)>>>0,rng=new Random(seed);
  const world=makeWorld(config.layout,new Random(seed));
  const oracle=createMap(world.width,world.height,0.15);
  for(let y=0;y<oracle.height;y++)for(let x=0;x<oracle.width;x++)
    if(clearance(world,pointOf(oracle,x,y),false)<-ROBOT_RADIUS+0.02)oracle.data[y*oracle.width+x]=4;
  const oracleCost=inflate(oracle,ROBOT_RADIUS+config.inflation+0.06);
  const sample=():Point=>{
    for(let i=0;i<5000;i++){const p={x:0.7+rng.next()*10.6,y:0.7+rng.next()*7.6};if(clearance(world,p,true)>config.inflation+0.25)return p;}
    throw new Error('Could not sample a collision-free position');
  };
  for(let tries=0;tries<500;tries++){
    const start=sample(),goal=sample(),d=distance(start,goal);
    if(d<3||!astar(oracle,oracleCost,start,goal))continue;
    const pose:Pose={...start,theta:rng.next()*Math.PI*2};
    const sim=new Simulation({...config,seed,movingObstacles:true},pose);
    sim.setGoal(goal);
    return {sim,straightLine:d,seed};
  }
  throw new Error('No reachable start/goal pair found; reduce inflation');
}
export function trialResult(sim:Simulation,trial:number,seed:number,straightLine:number):BenchmarkRow {
  return {trial,seed,success:sim.status==='Goal reached',time:sim.metrics.time,distance:sim.metrics.distance,
    straightLine,ratio:sim.metrics.distance/straightLine,replans:sim.metrics.replans,collisions:sim.metrics.collisions,
    staticContacts:sim.metrics.staticContacts,dynamicContacts:sim.metrics.dynamicContacts,
    movingContacts:sim.metrics.movingContacts,stoppedContacts:sim.metrics.stoppedContacts,
    contacts:sim.metrics.contacts.map(c=>({...c}))};
}
