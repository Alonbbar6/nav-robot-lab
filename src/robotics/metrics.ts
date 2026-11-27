import type { BenchmarkRow, BenchmarkSummary, Metrics, OccupancyGrid } from './types';
export const freshMetrics=():Metrics=>({plannedLength:0,distance:0,time:0,replans:0,minClearance:Infinity,collisions:0,explored:0,
  staticContacts:0,dynamicContacts:0,movingContacts:0,stoppedContacts:0,contacts:[]});
/** Coverage is observed cells / total map area, not an oracle of reachable free space. */
export function exploredPercent(map:OccupancyGrid):number {
  let n=0;for(const v of map.data)if(Math.abs(v)>0.1)n++;return n/map.data.length*100;
}
export function summarize(rows:BenchmarkRow[]):BenchmarkSummary {
  const successful=rows.filter(r=>r.success),times=successful.map(r=>r.time).sort((a,b)=>a-b);
  const mean=(values:number[])=>values.length?values.reduce((a,b)=>a+b,0)/values.length:0;
  return {successRate:rows.length?successful.length/rows.length*100:0,meanTime:mean(times),
    medianTime:times.length?(times[Math.floor((times.length-1)/2)]+times[Math.floor(times.length/2)])/2:0,
    meanRatio:mean(successful.map(r=>r.ratio)),meanReplans:mean(rows.map(r=>r.replans)),totalCollisions:rows.reduce((a,r)=>a+r.collisions,0),
    staticContacts:rows.reduce((a,r)=>a+r.staticContacts,0),dynamicContacts:rows.reduce((a,r)=>a+r.dynamicContacts,0),
    movingContacts:rows.reduce((a,r)=>a+r.movingContacts,0),stoppedContacts:rows.reduce((a,r)=>a+r.stoppedContacts,0),
    trialsWithContacts:rows.filter(r=>r.collisions>0).length};
}
