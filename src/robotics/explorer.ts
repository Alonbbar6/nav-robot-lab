import type { OccupancyGrid, Path, Point } from './types';
import { pointOf } from './mapper';
import { astar } from './planner';
import { distance } from './world';
export function findFrontiers(map:OccupancyGrid,cost:Uint8Array):Point[] {
  const points:Point[]=[],w=map.width;
  // Spatially sampled frontier representatives reduce planner workload.
  const buckets=new Set<string>();
  for(let y=1;y<map.height-1;y++)for(let x=1;x<w-1;x++){
    const i=y*w+x;if(map.data[i]>-0.3||cost[i]===255)continue;
    if([i-1,i+1,i-w,i+w].some(j=>Math.abs(map.data[j])<0.08)){
      const key=`${Math.floor(x/8)},${Math.floor(y/8)}`;
      if(!buckets.has(key)){buckets.add(key);points.push(pointOf(map,x,y));}
    }
  }return points;
}
export function chooseFrontier(map:OccupancyGrid,cost:Uint8Array,pose:Point,frontiers:Point[],excluded:Point[]=[]):{goal:Point;path:Path}|null {
  const candidates=frontiers.filter(p=>distance(p,pose)>0.45&&!excluded.some(e=>distance(p,e)<0.5)).sort((a,b)=>distance(a,pose)-distance(b,pose));
  for(const goal of candidates){const path=astar(map,cost,pose,goal);if(path)return {goal,path};}return null;
}
