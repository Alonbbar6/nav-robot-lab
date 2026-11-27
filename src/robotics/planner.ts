import type { OccupancyGrid, Path, Point } from './types';
import { bresenham, cellOf, inGrid, pointOf } from './mapper';
import { distance } from './world';
/** Binary heap avoids quadratic open-set scans on the 43,200-cell map. */
class MinHeap {
  a:{id:number;f:number}[]=[];
  push(v:{id:number;f:number}) {let i=this.a.length;this.a.push(v);while(i){const p=(i-1)>>1;if(this.a[p].f<=v.f)break;this.a[i]=this.a[p];i=p;}this.a[i]=v;}
  pop() {const root=this.a[0],last=this.a.pop()!;if(this.a.length){let i=0;while(i*2+1<this.a.length){let c=i*2+1;if(c+1<this.a.length&&this.a[c+1].f<this.a[c].f)c++;if(last.f<=this.a[c].f)break;this.a[i]=this.a[c];i=c;}this.a[i]=last;}return root;}
}
export function lineFree(map:OccupancyGrid,cost:Uint8Array,a:Point,b:Point):boolean {
  const p=cellOf(map,a),q=cellOf(map,b),cells=bresenham(p.x,p.y,q.x,q.y);
  return cells.every(([x,y],i)=>{
    if(!inGrid(map,x,y)||cost[y*map.width+x]===255)return false;
    if(i){const [px,py]=cells[i-1];if(px!==x&&py!==y&&(cost[py*map.width+x]===255||cost[y*map.width+px]===255))return false;}
    return true;
  });
}
export function astar(map:OccupancyGrid,cost:Uint8Array,start:Point,goal:Point,smooth=true):Path|null {
  const s=cellOf(map,start),t=cellOf(map,goal),w=map.width;
  if(!inGrid(map,s.x,s.y)||!inGrid(map,t.x,t.y))return null;
  const si=s.y*w+s.x,ti=t.y*w+t.x;
  if(cost[si]===255||cost[ti]===255)return null;
  const g=new Float64Array(cost.length).fill(Infinity),parent=new Int32Array(cost.length).fill(-1),closed=new Uint8Array(cost.length),heap=new MinHeap();
  const heuristic=(x:number,y:number)=>{const dx=Math.abs(x-t.x),dy=Math.abs(y-t.y);return Math.max(dx,dy)+(Math.SQRT2-1)*Math.min(dx,dy);};
  g[si]=0;heap.push({id:si,f:heuristic(s.x,s.y)});
  while(heap.a.length){
    const {id}=heap.pop();if(closed[id])continue;
    if(id===ti){
      const cells:Point[]=[];let n=id;while(n!==-1){cells.push(pointOf(map,n%w,Math.floor(n/w)));n=parent[n];}cells.reverse();
      cells[0]={...start};cells[cells.length-1]={...goal};
      const poses:Point[]=[cells[0]];
      if(smooth){let i=0;while(i<cells.length-1){let j=Math.min(cells.length-1,i+60);while(j>i+1&&!lineFree(map,cost,cells[i],cells[j]))j--;poses.push(cells[j]);i=j;}}
      else poses.splice(0,1,...cells);
      return {poses,length:poses.reduce((sum,p,i)=>sum+(i?distance(p,poses[i-1]):0),0)};
    }
    closed[id]=1;const x=id%w,y=Math.floor(id/w);
    for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){
      if(!dx&&!dy)continue;const nx=x+dx,ny=y+dy;
      if(!inGrid(map,nx,ny))continue;const ni=ny*w+nx;
      if(closed[ni]||cost[ni]===255)continue;
      if(dx&&dy&&(cost[y*w+nx]===255||cost[ny*w+x]===255))continue;
      const next=g[id]+(dx&&dy?Math.SQRT2:1)*(1+(cost[ni]-1)*0.12);
      if(next<g[ni]){g[ni]=next;parent[ni]=id;heap.push({id:ni,f:next+heuristic(nx,ny)});}
    }
  }return null;
}
