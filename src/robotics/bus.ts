import type { LaserScan, Odometry, OccupancyGrid, Path, Point, Twist, TrackedObstacles } from './types';
interface Topics { '/scan': LaserScan; '/odom': Odometry; '/map': OccupancyGrid; '/plan': Path; '/cmd_vel': Twist; '/goal_pose': Point; '/tracked_obstacles': TrackedObstacles }
export class MessageBus {
  private listeners = new Map<keyof Topics, Set<(message: never) => void>>();
  private history = new Map<keyof Topics, { count: number; stamps: number[] }>();
  constructor() {
    for (const topic of ['/scan','/odom','/map','/plan','/cmd_vel','/goal_pose','/tracked_obstacles'] as (keyof Topics)[])
      this.history.set(topic, { count: 0, stamps: [] });
  }
  subscribe<K extends keyof Topics>(topic: K, handler: (message: Topics[K]) => void): () => void {
    if (!this.listeners.has(topic)) this.listeners.set(topic, new Set());
    this.listeners.get(topic)!.add(handler as (message: never) => void);
    return () => { this.listeners.get(topic)?.delete(handler as (message: never) => void); };
  }
  publish<K extends keyof Topics>(topic: K, message: Topics[K], time: number): void {
    const h = this.history.get(topic)!; h.count++; h.stamps.push(time);
    while (h.stamps.length && h.stamps[0] < time - 2) h.stamps.shift();
    this.listeners.get(topic)?.forEach(fn => fn(message as never));
  }
  rates(time: number) {
    return [...this.history].map(([topic, h]) => ({
      topic, count: h.count, rate: h.stamps.filter(t => t > time - 2).length / Math.min(2, Math.max(time, 0.1)),
    }));
  }
}
