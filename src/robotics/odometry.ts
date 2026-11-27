import type { Pose, Twist } from './types';
import { Random } from './random';
import { wrap } from './world';
/** Dead-reckoning displacement, optionally with biased wheel/heading estimates. No truth-map access. */
export function updateOdometry(odom: Pose,before: Pose,after: Pose,velocity: Twist,dt: number,noisy: boolean,rng: Random): Pose {
  if (!noisy) return {...after};
  const travel=Math.hypot(after.x-before.x,after.y-before.y)*Math.sign(velocity.linear);
  const turn=wrap(after.theta-before.theta);
  const theta=wrap(odom.theta+turn*1.025+travel*0.012+rng.gaussian()*Math.sqrt(dt)*0.001);
  const d=travel*(1.015+rng.gaussian()*0.008);
  return {x:odom.x+d*Math.cos((odom.theta+theta)/2),y:odom.y+d*Math.sin((odom.theta+theta)/2),theta};
}
