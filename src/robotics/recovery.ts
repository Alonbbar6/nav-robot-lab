import type { OccupancyGrid, Point } from './types';
import { cellOf, inGrid, pointOf } from './mapper';
/** Find the nearest known-free pose outside global inflation, traversing only
 * locally collision-free cells. Needed when new observations inflate under a
 * physically clear robot. Never clears obstacles or accesses world geometry. */
export function inflationEscape(map:OccupancyGrid,global:Uint8Array,local:Uint8Array,pose:Point):Point|null {
  const s=cellOf(map,pose),w=map.width;
  if(!inGrid(map,s.x,s.y))return null;
  const start=s.y*w+s.x;
  if(global[start]!==255||local[start]===255)return null;
  const queue=[start],seen=new Set<number>([start]);
  for(let j=0;j<queue.length;j++){
    const i=queue[j],x=i%w,y=Math.floor(i/w);
    if(global[i]<24&&map.data[i]<-0.2)return pointOf(map,x,y);
    for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){
      const nx=x+dx,ny=y+dy,ni=ny*w+nx;
      if(!inGrid(map,nx,ny)||seen.has(ni)||local[ni]===255||Math.hypot(nx-s.x,ny-s.y)*map.resolution>1)continue;
      if(dx&&dy&&(local[y*w+nx]===255||local[ny*w+x]===255))continue;
      seen.add(ni);queue.push(ni);
    }
  }return null;
}
