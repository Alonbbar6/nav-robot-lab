import type { LaserScan, OccupancyGrid, Point, Pose } from './types';
export function bresenham(x0:number,y0:number,x1:number,y1:number): [number,number][] {
  const cells:[number,number][]=[]; const dx=Math.abs(x1-x0),sx=x0<x1?1:-1,dy=-Math.abs(y1-y0),sy=y0<y1?1:-1;
  let err=dx+dy;
  for (;;) {
    cells.push([x0,y0]); if (x0===x1&&y0===y1) break;
    const e=2*err;if(e>=dy){err+=dy;x0+=sx;}if(e<=dx){err+=dx;y0+=sy;}
  } return cells;
}
export const logOddsUpdate=(value:number,delta:number)=>Math.max(-4,Math.min(4,value+delta));
export const cellOf=(map:OccupancyGrid,p:Point)=>({x:Math.floor((p.x-map.origin.x)/map.resolution),y:Math.floor((p.y-map.origin.y)/map.resolution)});
export const inGrid=(map:OccupancyGrid,x:number,y:number)=>x>=0&&y>=0&&x<map.width&&y<map.height;
export const pointOf=(map:OccupancyGrid,x:number,y:number):Point=>({x:map.origin.x+(x+0.5)*map.resolution,y:map.origin.y+(y+0.5)*map.resolution});
export function createMap(width=12,height=9,resolution=0.05): OccupancyGrid {
  const w=Math.ceil(width/resolution),h=Math.ceil(height/resolution);
  return {resolution,width:w,height:h,origin:{x:0,y:0},data:new Float32Array(w*h)};
}
export function mapScan(map:OccupancyGrid,pose:Pose,scan:LaserScan,dynamicBeams:ReadonlySet<number>=new Set()):void {
  const from=cellOf(map,pose);
  // Each scan only applies one free or occupied update to a cell, avoiding beam-density bias.
  const observed=new Int8Array(map.data.length);
  scan.ranges.forEach((range,i)=>{
    const a=pose.theta+scan.angle_min+i*scan.angle_increment;
    const to=cellOf(map,{x:pose.x+Math.cos(a)*range,y:pose.y+Math.sin(a)*range});
    const cells=bresenham(from.x,from.y,to.x,to.y),hit=range<scan.range_max-0.015&&!dynamicBeams.has(i);
    cells.forEach(([x,y],j)=>{
      if(!inGrid(map,x,y))return;
      const index=y*map.width+x;
      if(hit&&j===cells.length-1) observed[index]=2;
      else if(!observed[index]) observed[index]=1;
    });
  });
  for(let i=0;i<observed.length;i++)if(observed[i])map.data[i]=logOddsUpdate(map.data[i],observed[i]===2?0.85:-0.4);
}
export function decayMap(map:OccupancyGrid,seconds:number):void {
  const factor=Math.exp(-seconds/16);
  for(let i=0;i<map.data.length;i++)map.data[i]*=factor;
}
