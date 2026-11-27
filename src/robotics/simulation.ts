import { normalizeConfig, type Config, type LaserScan, type Mode, type Path, type Point, type Pose, type Twist, type World, type OccupancyGrid, type Metrics, type TrackedObstacle, type ContactEvent } from './types';
import { MessageBus } from './bus';
import { Random } from './random';
import { clearance, distance, integrate, makeWorld, moveObstacles, ROBOT_RADIUS, rectClearance } from './world';
import { sense } from './lidar';
import { updateOdometry } from './odometry';
import { createMap, decayMap, mapScan } from './mapper';
import { inflate } from './costmap';
import { astar, lineFree } from './planner';
import { purePursuit } from './controller';
import { collisionMonitor } from './safety';
import { chooseFrontier, findFrontiers } from './explorer';
import { exploredPercent, freshMetrics } from './metrics';
import { ObstacleTracker } from './tracker';
import { dynamicWindow } from './dwa';
import { inflationEscape } from './recovery';

/** Orchestrates ROS-shaped nodes at deterministic simulation-time rates.
 * Physics/odometry:60 Hz, lidar/mapper:10 Hz, costmap/controller:5/20 Hz.
 * UI owns a fixed-step accumulator; this class has no DOM or React dependency.
 */
export class Simulation {
  config:Config;
  world:World;
  pose:Pose;
  odom:Pose;
  scan:LaserScan|null=null;
  tracks:TrackedObstacle[]=[];
  localPlannerState='Following';
  map:OccupancyGrid;
  costmap:Uint8Array;
  localCostmap:Uint8Array;
  path:Path={poses:[],length:0};
  goal:Point|null=null;
  lookahead:Point|null=null;
  trail:Point[]=[];
  frontiers:Point[]=[];
  metrics:Metrics=freshMetrics();
  time=0;
  status='Awaiting goal';
  safety:'clear'|'slow'|'stop'='clear';
  mode:Mode='goal';
  bus=new MessageBus();
  private rng:Random;
  private velocity:Twist={linear:0,angular:0};
  private command:Twist={linear:0,angular:0};
  private manual:Twist={linear:0,angular:0};
  private scanClock=0;
  private controlClock=0;
  private costClock=0;
  private stopped=0;
  private lastPlan=-10;
  private contactPairs=new Set<string>();
  private tracker=new ObstacleTracker();
  private goalStarted=0;
  private excluded:Point[]=[];
  private lastExplore=-10;
  constructor(config:Partial<Config>={},start?:Pose) {
    this.config=normalizeConfig(config);
    this.rng=new Random(this.config.seed);
    this.world=makeWorld(this.config.layout,this.rng);
    this.pose=start?{...start}:{x:1.2,y:1.2,theta:0.25};
    this.odom={...this.pose};
    this.map=createMap(this.world.width,this.world.height);
    this.costmap=inflate(this.map,ROBOT_RADIUS+this.config.inflation);
    this.localCostmap=inflate(this.map,ROBOT_RADIUS+0.04);
    this.connectNodes();
    this.publishScan();
    this.refreshCostmap();
  }
  private connectNodes() {
    this.bus.subscribe('/scan',scan=>{
      this.scan=scan;
      const tracked=this.tracker.update(scan,this.odom,this.map,this.time);
      this.bus.publish('/tracked_obstacles',tracked,this.time);
      mapScan(this.map,this.odom,scan,this.tracker.dynamicBeams);
      this.bus.publish('/map',this.map,this.time);
    });
    this.bus.subscribe('/tracked_obstacles',msg=>{this.tracks=msg.tracks;});
    this.bus.subscribe('/odom',msg=>{this.odom=msg.pose;});
    this.bus.subscribe('/cmd_vel',cmd=>{this.command=cmd;});
    this.bus.subscribe('/plan',path=>{this.path=path;});
    this.bus.subscribe('/goal_pose',goal=>{this.goal={...goal};});
  }
  reset(config:Partial<Config>={}) {
    // Copy initialized instance fields, with subscriptions rebound to THIS instance.
    const next=new Simulation({...this.config,...config});
    Object.assign(this,next);
    this.bus=new MessageBus();
    this.connectNodes();
    this.bus.publish('/map',this.map,0);
  }
  updateConfig(config:Partial<Config>) {
    if((config.layout!==undefined&&config.layout!==this.config.layout)||(config.seed!==undefined&&config.seed!==this.config.seed)){
      this.reset(config);return;
    }
    this.config=normalizeConfig({...this.config,...config});
    if(config.inflation!==undefined){this.refreshCostmap();if(this.goal)this.plan(true);}
  }
  setManual(command:Twist) { this.manual={...command}; }
  setMode(mode:Mode) {
    this.mode=mode;this.goal=null;this.path={poses:[],length:0};this.command={linear:0,angular:0};
    this.manual={linear:0,angular:0};this.excluded=[];this.lastExplore=-10;
    this.status=mode==='manual'?'Manual control':mode==='explore'?'Finding frontiers':'Awaiting goal';
  }
  setGoal(point:Point) {
    if(this.mode!=='goal')this.setMode('goal');
    this.beginGoal(point);
  }
  private beginGoal(point:Point) {
    this.bus.publish('/goal_pose',{...point},this.time);
    this.metrics=freshMetrics();this.metrics.explored=exploredPercent(this.map);
    this.goalStarted=this.time;this.stopped=0;
    this.plan(false);
  }
  private publishScan() {
    this.bus.publish('/scan',sense(this.world,this.pose,this.config,this.rng),this.time);
  }
  private refreshCostmap() {
    this.costmap=inflate(this.map,ROBOT_RADIUS+this.config.inflation);
    this.localCostmap=inflate(this.map,ROBOT_RADIUS+0.04);
    this.metrics.explored=exploredPercent(this.map);
    if(this.mode==='explore')this.frontiers=findFrontiers(this.map,this.costmap);
  }
  private plan(replan:boolean) {
    if(!this.goal)return;
    this.lastPlan=this.time;
    if(replan)this.metrics.replans++;
    const path=astar(this.map,this.costmap,this.odom,this.goal);
    this.bus.publish('/plan',path??{poses:[],length:0},this.time);
    this.lookahead=null;
    if(path){this.metrics.plannedLength=path.length;this.status='Navigating';}
    else {this.status='Goal unreachable';this.command={linear:0,angular:0};}
  }
  private explore() {
    if(this.mode!=='explore'||this.time-this.lastExplore<1)return;
    if(this.goal&&this.status==='Navigating')return;
    this.lastExplore=this.time;
    if(this.goal&&this.status==='Goal unreachable')this.excluded.push(this.goal);
    const target=chooseFrontier(this.map,this.costmap,this.odom,this.frontiers,this.excluded);
    if(target){
      this.beginGoal(target.goal);
    } else {
      this.goal=null;this.path={poses:[],length:0};
      this.status=this.excluded.length?'Exploration paused: no reachable frontiers':'Exploration complete';
    }
  }
  step(dt=1/60) {
    // Public stepping is bounded so callers cannot tunnel through obstacles.
    if(!Number.isFinite(dt)||dt<=0)return;
    if(dt>1/60+1e-8){const steps=Math.ceil(dt/(1/60));for(let i=0;i<steps;i++)this.step(dt/steps);return;}
    this.time+=dt;
    if(this.config.movingObstacles)moveObstacles(this.world,dt,this.config.obstacleSpeed);
    const before={...this.pose};
    const next=integrate(this.pose,this.velocity,this.command,dt,this.config.maxSpeed);
    // Ground truth is confined to world physics and metric recording. Neither
    // tracker, DWA nor monitor receives world.obstacles or wall geometry.
    const contacts=new Set<string>();
    const motion={...this.velocity};
    const record=(key:string,kind:ContactEvent['kind'])=>{
      contacts.add(key);
      if(this.contactPairs.has(key))return;
      const moving=Math.abs(motion.linear)>0.02||Math.abs(motion.angular)>0.05;
      this.metrics.collisions++;
      if(kind==='static')this.metrics.staticContacts++;else this.metrics.dynamicContacts++;
      if(moving)this.metrics.movingContacts++;else this.metrics.stoppedContacts++;
      this.metrics.contacts.push({stamp:this.time,kind,obstacle:key,moving,...motion});
    };
    let staticContact=false,dynamicContact=false;
    this.world.walls.forEach((r,i)=>{
      const d=Math.min(rectClearance(before,r),rectClearance(next.pose,r))-ROBOT_RADIUS,key=`wall:${i}`;
      if(d<=0){record(key,'static');staticContact=true;}
      else if(d<0.015&&this.contactPairs.has(key))contacts.add(key);
    });
    if(this.config.movingObstacles)this.world.obstacles.forEach((o,i)=>{
      const d=Math.min(distance(before,o),distance(next.pose,o))-ROBOT_RADIUS-o.radius,key=`moving:${i}`;
      if(d<=0){record(key,'dynamic');if(distance(next.pose,o)<distance(before,o))dynamicContact=true;}
      else if(d<0.015&&this.contactPairs.has(key))contacts.add(key);
    });
    if(staticContact||dynamicContact){
      this.velocity={linear:0,angular:next.velocity.angular};
      this.pose={...this.pose,theta:next.pose.theta};
    }else{this.pose=next.pose;this.velocity=next.velocity;}
    this.contactPairs=contacts;
    this.metrics.minClearance=Math.min(this.metrics.minClearance,Math.max(0,clearance(this.world,this.pose,this.config.movingObstacles)));
    const traveled=distance(before,this.pose);
    if(this.goal&&this.status!=='Goal reached'){this.metrics.distance+=traveled;this.metrics.time=this.time-this.goalStarted;}
    const odom=updateOdometry(this.odom,before,this.pose,this.velocity,dt,this.config.odometryNoise,this.rng);
    this.bus.publish('/odom',{pose:odom,twist:this.velocity,stamp:this.time},this.time);
    this.scanClock+=dt;this.costClock+=dt;this.controlClock+=dt;
    if(this.scanClock>=0.1-1e-8){this.scanClock-=0.1;decayMap(this.map,0.1);this.publishScan();}
    if(this.costClock>=0.2-1e-8){
      this.costClock-=0.2;this.refreshCostmap();
      if(!this.trail.length||distance(this.odom,this.trail[this.trail.length-1])>0.035){this.trail.push({...this.odom});if(this.trail.length>6000)this.trail.shift();}
      if(this.goal&&this.status==='Navigating'&&this.time-this.lastPlan>0.5){
        // Check the remaining path, not already-traversed segments.
        let closest=0,best=Infinity;
        this.path.poses.forEach((p,i)=>{const d=distance(this.odom,p);if(d<best){best=d;closest=i;}});
        const blocked=this.path.poses.slice(Math.max(1,closest)).some((p,i,arr)=>!lineFree(this.map,this.costmap,i?arr[i-1]:this.odom,p));
        if(blocked)this.plan(true);
      }else if(this.goal&&this.status==='Goal unreachable'&&this.time-this.lastPlan>1.5)this.plan(true);
      this.explore();
    }
    if(this.controlClock>=0.05-1e-8){
      this.controlClock-=0.05;
      let cmd:Twist={linear:0,angular:0};
      if(this.mode==='manual')cmd=this.manual;
      else if(this.goal&&this.status==='Navigating'){
        if(distance(this.odom,this.goal)<0.2){this.status='Goal reached';this.path={poses:[],length:0};this.lookahead=null;}
        else{
          const c=purePursuit(this.odom,this.path,this.config.lookahead,this.config.maxSpeed);
          cmd=c.command;this.lookahead=c.target;
        }
      }
      let recovering=false;
      if(this.goal&&this.status==='Goal unreachable'){
        const escape=inflationEscape(this.map,this.costmap,this.localCostmap,this.odom);
        if(escape){
          const path=astar(this.map,this.localCostmap,this.odom,escape);
          if(path){
            const recovery=purePursuit(this.odom,path,0.25,0.25);
            cmd=recovery.command;this.lookahead=recovery.target;recovering=true;
          }
        }
      }
      const preferred=cmd;
      const target=this.mode==='manual'?{x:this.odom.x+cmd.linear*Math.cos(this.odom.theta),y:this.odom.y+cmd.linear*Math.sin(this.odom.theta)}:this.lookahead;
      const local=dynamicWindow({pose:this.odom,velocity:this.velocity,preferred,target,
        map:this.map,costmap:this.localCostmap,inflationCosts:this.costmap,tracks:this.tracks,maxSpeed:this.config.maxSpeed,stamp:this.time});
      cmd=local.command;
      this.localPlannerState=!local.safe?'Braking: no safe trajectory':cmd.linear<-0.01?'Backing off':
        Math.abs(cmd.linear)<0.025&&preferred.linear>0.05?'Yielding':recovering?'Recovering global clearance':'Following';
      const safe=collisionMonitor(this.scan,cmd,this.odom,this.tracks,this.time);this.safety=safe.state;
      this.stopped=(safe.state==='stop'||Math.abs(cmd.linear)<0.025)&&Math.abs(preferred.linear)>0.05?this.stopped+0.05:0;
      if(this.stopped>1&&this.goal&&this.time-this.lastPlan>1){this.plan(true);this.stopped=0;}
      // Emergency stop cancels forward inertia immediately (not merely cmd_vel).
      if(safe.state==='stop')this.velocity.linear=0;
      this.bus.publish('/cmd_vel',safe.command,this.time);
    }
  }
}
