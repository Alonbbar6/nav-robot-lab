import type { OccupancyGrid } from './types';
/** Two-pass chamfer distance transform, in meters, O(grid size). */
export function obstacleDistances(map:OccupancyGrid):Float32Array {
  const {width:w,height:h,data}=map,d=new Float32Array(data.length);
  for(let i=0;i<d.length;i++)d[i]=data[i]>0.7?0:1e5;
  const diagonal=Math.SQRT2;
  for(let y=0;y<h;y++)for(let x=0;x<w;x++){
    const i=y*w+x;
    if(x)d[i]=Math.min(d[i],d[i-1]+1);
    if(y)d[i]=Math.min(d[i],d[i-w]+1);
    if(x&&y)d[i]=Math.min(d[i],d[i-w-1]+diagonal);
    if(x+1<w&&y)d[i]=Math.min(d[i],d[i-w+1]+diagonal);
  }
  for(let y=h-1;y>=0;y--)for(let x=w-1;x>=0;x--){
    const i=y*w+x;
    if(x+1<w)d[i]=Math.min(d[i],d[i+1]+1);
    if(y+1<h)d[i]=Math.min(d[i],d[i+w]+1);
    if(x+1<w&&y+1<h)d[i]=Math.min(d[i],d[i+w+1]+diagonal);
    if(x&&y+1<h)d[i]=Math.min(d[i],d[i+w-1]+diagonal);
  }
  for(let i=0;i<d.length;i++)d[i]*=map.resolution;
  return d;
}
export function inflate(map:OccupancyGrid,radius:number):Uint8Array {
  const d=obstacleDistances(map),cost=new Uint8Array(d.length);
  for(let y=0;y<map.height;y++)for(let x=0;x<map.width;x++){
    const i=y*map.width+x,boundary=Math.min(x+0.5,y+0.5,map.width-x-0.5,map.height-y-0.5)*map.resolution;
    const dist=Math.min(boundary,d[i]);
    cost[i]=dist<=radius?255:dist<radius+0.4?Math.round(8+50*(1-(dist-radius)/0.4)):Math.abs(map.data[i])<0.1?4:1;
  } return cost;
}
