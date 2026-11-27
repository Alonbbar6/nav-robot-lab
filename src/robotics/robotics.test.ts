import { describe, expect, it } from 'vitest';
import { rayRect, sense } from './lidar';
import { bresenham, createMap, logOddsUpdate, mapScan } from './mapper';
import { astar } from './planner';
import { inflate } from './costmap';
import { purePursuit } from './controller';
import { defaultConfig, type LaserScan } from './types';
import { Random } from './random';
import { collisionMonitor } from './safety';
import { MessageBus } from './bus';
import { Simulation } from './simulation';
import { summarize } from './metrics';

describe('ray-cast lidar',()=>{
  it('intersects a known vertical wall at two meters',()=>{
    expect(rayRect({x:1,y:2},1,0,{x:3,y:0,w:0.1,h:4},6)).toBeCloseTo(2);
    expect(rayRect({x:1,y:5},1,0,{x:3,y:0,w:0.1,h:4},6)).toBe(6);
  });
  it('produces LaserScan angles and seeded noisy ranges',()=>{
    const config={...defaultConfig,beams:3,fov:90,movingObstacles:false};
    const scan=sense({width:12,height:9,walls:[{x:3,y:0,w:0.1,h:9}],obstacles:[]},{x:1,y:4,theta:0},config,new Random(42));
    expect(scan.ranges[1]).toBeCloseTo(2,1);
    expect(scan.angle_increment).toBeCloseTo(Math.PI/4);
    expect(scan.ranges).toHaveLength(3);
  });
});
describe('Bresenham traversal',()=>{
  it('includes both endpoints in every octant',()=>{
    expect(bresenham(0,0,3,1)).toEqual([[0,0],[1,0],[2,1],[3,1]]);
    expect(bresenham(3,1,0,0)).toEqual([[3,1],[2,1],[1,0],[0,0]]);
    expect(bresenham(1,1,1,1)).toEqual([[1,1]]);
  });
});
describe('occupancy log odds',()=>{
  it('clamps repeated updates',()=>{
    expect(logOddsUpdate(3.8,0.85)).toBe(4);
    expect(logOddsUpdate(-3.8,-0.4)).toBe(-4);
  });
  it('marks traversed cells free and a hit occupied',()=>{
    const map=createMap(5,5,1);
    const scan:LaserScan={angle_min:0,angle_max:0,angle_increment:0,range_min:0.04,range_max:6,ranges:[2]};
    mapScan(map,{x:0.5,y:2.5,theta:0},scan);
    expect(map.data[10]).toBeLessThan(0);expect(map.data[11]).toBeLessThan(0);
    expect(map.data[12]).toBeGreaterThan(0);expect(map.data[13]).toBe(0);
    for(let i=0;i<30;i++)mapScan(map,{x:0.5,y:2.5,theta:0},scan);
    expect(map.data[12]).toBe(4);expect(map.data[10]).toBe(-4);
  });
});
describe('A* global planner',()=>{
  it('finds the shortest 8-connected path',()=>{
    const map=createMap(5,5,1),cost=new Uint8Array(25).fill(1);
    const path=astar(map,cost,{x:0.5,y:0.5},{x:4.5,y:4.5},false)!;
    expect(path.length).toBeCloseTo(4*Math.SQRT2);
    expect(path.poses).toHaveLength(5);
  });
  it('returns none for a completely blocked passage',()=>{
    const map=createMap(5,5,1),cost=new Uint8Array(25).fill(1);
    for(let y=0;y<5;y++)cost[y*5+2]=255;
    expect(astar(map,cost,{x:0.5,y:2.5},{x:4.5,y:2.5})).toBeNull();
  });
  it('does not cut diagonal corners',()=>{
    const map=createMap(2,2,1),cost=new Uint8Array([1,255,255,1]);
    expect(astar(map,cost,{x:0.5,y:0.5},{x:1.5,y:1.5})).toBeNull();
  });
});
describe('costmap inflation',()=>{
  it('blocks cells within the robot plus margin radius',()=>{
    const map=createMap(9,9,1);map.data[4*9+4]=4;
    const cost=inflate(map,1.1);
    expect(cost[40]).toBe(255);expect(cost[41]).toBe(255);
    expect(cost[42]).not.toBe(255);
    expect(cost[3*9+3]).toBeLessThan(255);
  });
});
describe('pure pursuit',()=>{
  it('steers counterclockwise left and clockwise right',()=>{
    const pose={x:0,y:0,theta:0};
    const left=purePursuit(pose,{poses:[{x:0,y:0},{x:2,y:1}],length:2.24},0.6,0.8);
    const right=purePursuit(pose,{poses:[{x:0,y:0},{x:2,y:-1}],length:2.24},0.6,0.8);
    expect(left.command.angular).toBeGreaterThan(0);
    expect(right.command.angular).toBeLessThan(0);
    expect(left.command.linear).toBeGreaterThan(0);
  });
});
describe('collision monitor and topic bus',()=>{
  it('stops on a forward obstacle and preserves turning',()=>{
    const scan:LaserScan={angle_min:0,angle_max:0,angle_increment:0,range_min:0.04,range_max:6,ranges:[0.3]};
    const safe=collisionMonitor(scan,{linear:0.8,angular:1});
    expect(safe.state).toBe('stop');expect(safe.command).toEqual({linear:0,angular:1});
  });
  it('delivers typed messages and unsubscribes',()=>{
    const bus=new MessageBus();let received=0;
    const unsubscribe=bus.subscribe('/cmd_vel',msg=>{received+=msg.linear;});
    bus.publish('/cmd_vel',{linear:1,angular:0},1);unsubscribe();
    bus.publish('/cmd_vel',{linear:2,angular:0},2);
    expect(received).toBe(1);expect(bus.rates(2).find(r=>r.topic==='/cmd_vel')?.count).toBe(2);
  });
});
describe('determinism and summary semantics',()=>{
  it('replays seeded sensor readings and resets without leaked listeners',()=>{
    const a=new Simulation({seed:123,movingObstacles:false}),b=new Simulation({seed:123,movingObstacles:false});
    for(let i=0;i<12;i++){a.step();b.step();}
    expect(a.scan?.ranges).toEqual(b.scan?.ranges);expect(a.map.data).toEqual(b.map.data);
    a.reset();
    const fresh=new Simulation({seed:123,movingObstacles:false});
    expect(a.scan?.ranges).toEqual(fresh.scan?.ranges);expect(a.map.data).toEqual(fresh.map.data);
    a.step();expect(a.time).toBeCloseTo(1/60);
    expect(a.bus.rates(a.time).find(r=>r.topic==='/odom')?.count).toBe(1);
  });
  it('reports no successful times as zero rather than NaN',()=>{
    expect(summarize([]).meanTime).toBe(0);expect(summarize([]).successRate).toBe(0);
  });
});
