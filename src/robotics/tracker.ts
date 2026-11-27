import type { LaserScan, OccupancyGrid, Point, Pose, TrackedObstacle, TrackedObstacles } from './types';
import { cellOf, inGrid } from './mapper';
import { distance } from './world';

interface Hit extends Point { beam: number }
interface Detection extends Point { radius: number; beams: number[] }

/** Least-squares circle on a visible lidar arc. Ill-conditioned fits use a
 * conservative surface-centroid offset, never a ground-truth shape or radius. */
function detection(points: Hit[], pose: Pose): Detection | null {
  if (points.length < 3) return null;
  const center = points.reduce((a,p)=>({x:a.x+p.x/points.length,y:a.y+p.y/points.length}),{x:0,y:0});
  let xx=0,xy=0,yy=0,bx=0,by=0;
  for(const p of points){
    const x=p.x-center.x,y=p.y-center.y,z=x*x+y*y;
    xx+=x*x;xy+=x*y;yy+=y*y;bx+=x*z/2;by+=y*z/2;
  }
  const det=xx*yy-xy*xy;
  let x=center.x,y=center.y,radius=0.2;
  if(det>1e-10){
    x+=(bx*yy-by*xy)/det;y+=(by*xx-bx*xy)/det;
    radius=points.reduce((a,p)=>a+Math.hypot(p.x-x,p.y-y)/points.length,0);
    const residual=points.reduce((a,p)=>a+Math.abs(Math.hypot(p.x-x,p.y-y)-radius)/points.length,0);
    if(radius>0.65||radius<0.07||residual>0.045)return null;
  }else{
    // Flat walls are not moving-circle hypotheses.
    return null;
  }
  if(distance(points[0],points[points.length-1])>1.2)return null;
  // A short arc may fit inside the physical surface. Keep a conservative bound.
  radius=Math.max(0.16,Math.min(0.6,radius+0.035));
  if(distance({x,y},pose)<radius)return null;
  return {x,y,radius,beams:points.map(p=>p.beam)};
}

/** Scan/odom/map-only nearest-neighbor tracker. Call BEFORE mapping this scan,
 * so freshly observed moving hits cannot explain themselves as static cells. */
export class ObstacleTracker {
  private tracks: TrackedObstacle[]=[];
  private nextId=1;
  private measurements=new Map<number,{x:number;y:number;stamp:number;start:number;samples:{x:number;y:number;stamp:number}[]}>();
  dynamicBeams=new Set<number>();
  update(scan:LaserScan,pose:Pose,map:OccupancyGrid,stamp:number):TrackedObstacles {
    const clusters:Hit[][]=[];let cluster:Hit[]=[];
    const flush=()=>{if(cluster.length)clusters.push(cluster);cluster=[];};
    scan.ranges.forEach((r,i)=>{
      if(!Number.isFinite(r)||r>=scan.range_max-0.02){flush();return;}
      const a=pose.theta+scan.angle_min+i*scan.angle_increment;
      const p:Hit={x:pose.x+r*Math.cos(a),y:pose.y+r*Math.sin(a),beam:i};
      const c=cellOf(map,p);
      // A confident neighbor accommodates centimeter range noise / grid aliasing.
      let known=false;
      for(let dy=-1;dy<=1&&!known;dy++)for(let dx=-1;dx<=1;dx++)
        if(inGrid(map,c.x+dx,c.y+dy)&&map.data[(c.y+dy)*map.width+c.x+dx]>2.5)known=true;
      const trackedMoving=this.tracks.some(t=>t.moving&&distance(t,p)<t.radius+0.3);
      if(known&&!trackedMoving){flush();return;}
      if(cluster.length&&distance(p,cluster[cluster.length-1])>Math.max(0.13,r*scan.angle_increment*2.8))flush();
      cluster.push(p);
    });flush();
    // Join the +/- pi wrap-around cluster of a full-circle scanner.
    if(scan.angle_max-scan.angle_min>6.2&&clusters.length>1){
      const first=clusters[0],last=clusters[clusters.length-1];
      if(distance(first[0],last[last.length-1])<0.18){clusters[0]=last.concat(first);clusters.pop();}
    }
    const detections=clusters.map(c=>detection(c,pose)).filter((d):d is Detection=>!!d);
    const used=new Set<number>(),next:TrackedObstacle[]=[];
    this.dynamicBeams=new Set();
    for(const d of detections){
      let previous:TrackedObstacle|undefined,best=0.6;
      for(const t of this.tracks){
        if(used.has(t.id))continue;
        const dt=stamp-t.stamp,err=distance(d,{x:t.x+t.vx*dt,y:t.y+t.vy*dt});
        if(err<best){best=err;previous=t;}
      }
      const id=previous?.id??this.nextId++;
      let vx=0,vy=0,age=0,moving=false;
      const last=this.measurements.get(id);
      const samples=[...(last?.samples??[]),{x:d.x,y:d.y,stamp}].filter(s=>stamp-s.stamp<=0.5+1e-8);
      if(previous&&last){
        used.add(id);
        // Short-window linear regression avoids turning centimeter-level circle-fit
        // noise into large single-scan velocity jumps.
        const mean=samples.reduce((a,s)=>({x:a.x+s.x/samples.length,y:a.y+s.y/samples.length,t:a.t+s.stamp/samples.length}),{x:0,y:0,t:0});
        let denom=0,nx=0,ny=0;
        for(const s of samples){const dt=s.stamp-mean.t;denom+=dt*dt;nx+=dt*(s.x-mean.x);ny+=dt*(s.y-mean.y);}
        const mx=nx/Math.max(1e-8,denom),my=ny/Math.max(1e-8,denom),speed=Math.hypot(mx,my);
        const scale=speed>1.6?1.6/speed:1;
        vx=previous.vx*0.3+mx*scale*0.7;vy=previous.vy*0.3+my*scale*0.7;
        age=stamp-last.start;moving=previous.moving||Math.hypot(vx,vy)>0.12;
      }
      const t:TrackedObstacle={id,x:d.x,y:d.y,vx,vy,radius:previous?0.65*previous.radius+0.35*d.radius:d.radius,stamp,age,missed:0,moving};
      next.push(t);
      this.measurements.set(id,{x:d.x,y:d.y,stamp,start:last?.start??stamp,samples});
      if(moving)for(const i of d.beams)this.dynamicBeams.add(i);
    }
    for(const t of this.tracks)if(!used.has(t.id)&&!next.some(n=>n.id===t.id)){
      const missed=stamp-(this.measurements.get(t.id)?.stamp??t.stamp);
      if(missed<=0.6&&t.moving)next.push({...t,missed});
      else this.measurements.delete(t.id);
    }
    this.tracks=next;
    return {stamp,tracks:next.map(t=>({...t}))};
  }
}
