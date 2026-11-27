# Nav Robot Lab

A simulated differential-drive robot with a ray-cast lidar explores an unknown room in your browser. It builds an occupancy grid as it goes, plans to the goal you click with A*, follows the plan with pure pursuit, and dodges moving obstacles with a lidar-based tracker and a DWA-style local planner. Everything is drawn live, and a benchmark tab runs repeatable headless trials.

Built as a robotics software portfolio project. Pure TypeScript, React and HTML canvas, no backend, no API keys, no ROS install needed. The modules map one to one onto ROS 2 / Nav2 nodes (table below).

## Quick start

Needs Node.js 20 or newer.

```bash
npm install
npm run dev        # open the URL Vite prints, then click the map to set a goal
npm test           # Vitest unit tests for the robotics modules
npm run typecheck  # TypeScript check
npm run build      # static build in dist/
```

## Results at a glance

50 trials per layout, base seed 7, moving obstacles on (full method in Benchmark below):

| Layout | Goals reached | Contacts (trials with any) | Wall contacts |
| --- | --- | --- | --- |
| Simple room | 50/50 | 2 (1/50) | 0 |
| Office | 50/50 | 4 (3/50) | 0 |
| Warehouse | 45/50 | 3 (3/50) | 0 |
| Total | 145/150 | 9 (7/150) | 0 |

Before the moving-obstacle work (tracker, DWA, 360 degree monitor) the same benchmark had 32 contacts across 27/150 trials. All 9 remaining contacts happened with the robot already stopped, and they still count as failures.

## Project layout

- `src/robotics/`: the robotics stack, framework free (world, lidar, odometry, mapper, costmap, planner, controller, tracker, dwa, recovery, safety, explorer, metrics, bus, simulation, benchmark)
- `src/lab/`: React UI and canvas renderer (Simulator, Benchmark, About)
- `src/robotics/*.test.ts`: unit tests

## Features

- Live split view: ground truth versus the robot's occupancy-grid belief.
- Three indoor environments: simple room, office with doorways, and cluttered warehouse.
- Differential-drive/unicycle kinematics with velocity and acceleration limits.
- Ray-cast lidar with 180–360 configurable beams, field of view, range, and seeded Gaussian range noise.
- **Mapping with known poses**, not full SLAM. Optional noisy dead reckoning demonstrates odometry drift.
- Five-centimeter log-odds occupancy mapping with Bresenham ray traversal and slow decay.
- Robot-footprint and safety-margin costmap inflation; unknown space remains traversable but expensive.
- Eight-connected A* with octile heuristic, a binary-heap open set, and collision-checked path pruning.
- Adaptive-lookahead pure pursuit provides preferred commands to an acceleration-limited DWA-style local planner.
- Lidar-only clustering, nearest-neighbor obstacle tracking, constant-velocity prediction, yielding and reverse escape.
- Full-circle, direction-aware collision monitoring with tracked-obstacle time to collision.
- Automatic replanning when the path becomes blocked or the safety monitor stays stopped.
- Frontier-based autonomous exploration and WASD/arrow manual control.
- Three seeded moving people/cart-like circles with configurable speed.
- Pause, single fixed-timestep stepping, reset, and 1×/2×/4× simulation speed.
- Goal metrics, static/dynamic and moving/stopped contact counts, timestamped contact events, live ROS-style topics and measured publish rates.
- Worker-based repeatable navigation benchmarks, per-trial results, and CSV export.

## Try it

On the Simulator tab, click either map to set a goal. The robot initially knows nothing about the room; its first lidar scan reveals nearby surfaces. Play advances the simulation, Pause freezes simulation time, and Step advances one 1/60-second physics tick. Select Explore to repeatedly navigate to the nearest reachable frontier, or Manual to drive with WASD / arrow keys. Release keys to stop. Avoid typing into inputs while driving.

Changing the layout or seed resets the simulation. Display switches only affect rendering. Changing sensor or robot parameters affects the next simulation updates. Inflation is a **safety margin added to the 0.22 m robot radius**, not the total robot footprint. Use Reset for a repeatable run after changing parameters.

## Stack and why

React + TypeScript + HTML Canvas, bundled with Vite. All simulation and navigation run in pure TypeScript inside the browser: zero simulator installation for viewers, no backend latency, no database, and no login. Seeded per-instance random generators make identical inputs and fixed timesteps repeatable. No real robot, ROS installation, cloud API, or AI model is required.

Rendering uses requestAnimationFrame, targeting approximately 60 fps. Physics runs at a fixed 60 Hz independently of rendering; sensor, map, costmap, and controller updates use lower simulation-time rates. Typed arrays and a heap-based planner keep grid operations bounded. Actual frame rate depends on the laptop, beam count, browser, and simulation multiplier; 4× advances simulated time rather than promising four times the rendered frames.

## Tests

The Vitest suite covers ray intersections, LaserScan ranges, Bresenham endpoints and octants, log-odds updates and clamping, A* shortest paths and unreachable/corner cases, inflation, left/right pure pursuit steering, typed message delivery, deterministic resets, and benchmark statistics. Dynamic-avoidance tests cover moving-cluster velocity accuracy, rejection of predicted collisions, acceleration-bounded safe command selection, reverse escape, rear and lateral safety, blind-patroller invariance, contact classification, and a two-minute static-wall non-overlap run.

## Robotics pipeline

### World and physics

The ground-truth world contains axis-aligned wall/furniture rectangles and three patrolling circles. A unicycle model integrates ẋ = v cos θ, ẏ = v sin θ, θ̇ = ω, which is the planar equivalent of differential drive. Commands are limited to the configured linear speed and 2.2 rad/s angular speed, with 0.9 m/s² linear and 4 rad/s² angular acceleration limits. **Patrollers remain blind**: they only reverse near static walls and never avoid or react to the robot. Tracking, planning, and safety receive no ground-truth obstacle geometry or velocity.

World physics rejects penetrating static motion and inward dynamic-contact motion, but allows the robot to separate from an overlapping moving obstacle instead of pinning it in place. True geometry is used only for simulated sensors, physical contact handling, benchmark pair sampling, and metrics. It is not an avoidance oracle.

### Lidar and odometry

Each beam intersects true rectangles analytically and moving circles with a ray-circle quadratic. A small seeded Gaussian error is added to hit ranges. The sensor publishes LaserScan-shaped messages containing angle_min, angle_max, angle_increment, range_min, range_max, and ranges. The odometry node publishes pose, twist, and simulation timestamp. With noise disabled it uses the known pose. With noise enabled it accumulates biased/noisy motion estimates to demonstrate drift; no pose correction is performed.

### Known-pose occupancy mapping

The mapper never reads world geometry. It consumes lidar and odometry through the message bus. Bresenham traversal marks cells before a hit free (−0.4 log odds) and the endpoint occupied (+0.85). Each cell receives at most one update per scan, with occupied evidence taking precedence. Evidence is clamped to [−4, 4]. No-hit max-range returns clear free space without inventing a wall. Evidence decays exponentially toward unknown with a 16-second time constant, while new free observations clear moved obstacles sooner. Hits associated with moving tracks clear traversed space but are not inserted as permanent occupied endpoints; DWA checks their predicted circular footprints separately.

### Costmap and A*

A two-pass chamfer distance transform approximates obstacle distance in O(grid cells). The global costmap inflates obstacles by robot radius plus the configured route margin, with an additional soft-cost band. The local controller has a separate footprint costmap with a fixed 0.04 m hard margin; global inflation is an additional soft critic. This global/local distinction is explicit in the controls: route preference is not the same as physical footprint collision. Unknown cells have a higher traversal cost than known free cells. A* searches eight neighbors with an admissible octile heuristic and disallows diagonal corner cutting. The resulting path is pruned with collision-checked line-of-sight segments. This is safe geometric pruning, not a curvature-constrained trajectory optimizer.

If newly observed global inflation grows under an otherwise locally collision-free robot, a bounded known-free search finds a recovery pose outside that band and A* verifies the local escape. This lets the robot regain global clearance rather than freezing forever in a physically clear location. It never erases static obstacles, disables footprint checks, or uses world geometry.

### Obstacle tracker

The tracker processes each scan **before** that scan updates the occupancy grid. Hit points matching confidently occupied map cells (log odds > 2.5, including neighboring cells for grid/range noise) are static background, except where an existing moving track explains them. Remaining adjacent hits are clustered, including the full-circle angle wrap. A small least-squares circle fit estimates center and radius; implausible fits and long wall-like clusters are rejected.

Detections are associated one-to-one with predicted tracks by nearest-neighbor distance. A short, 0.5-second position-history regression estimates velocity, with smoothing and an association-speed cap to limit lidar fitting jitter. Tracks coast through missed detections for up to 0.6 seconds with growing uncertainty. Output contains estimated center, velocity, radius, timestamp, age, motion flag, and missed-detection time on `/tracked_obstacles`. Belief-view circles and arrows show these estimates; dashed circles indicate coasting. These are sensor-derived hypotheses, not the simulator's true obstacle IDs.

### Pure pursuit, DWA, and collision monitor

Pure pursuit projects onto the current path and selects an adaptive lookahead point, producing a **preferred** velocity. It is no longer the final actuator command. A DWA-style, acceleration-limited trajectory generator samples forward, stop, small reverse (down to −0.3 m/s), and angular velocity targets. Each is forward simulated for 1.5 seconds with the same acceleration limits as physics. Only the first 20 Hz control-period velocity is emitted, so velocity changes remain bounded even when the sampled terminal speed is much higher.

Rollouts reject lethal local-footprint-costmap cells and predicted track-footprint intersections, including a 0.06 m dynamic margin plus time/missed-observation uncertainty. Local static inflation already includes the robot footprint and 0.04 m margin, so the footprint is not inflated twice. Dynamic tracks use constant-velocity prediction and explicit circular footprint separation. Candidates are scored on preferred-command agreement, lookahead progress, global inflation, minimum clearance, and future escape clearance. Under a collision threat, escape clearance takes priority over stopping exactly at the goal: the robot can yield, reverse, turn away, or pass through the goal region and return. If no rollout is safe, it brakes and explicitly reports that no safe trajectory was found rather than labeling a colliding rollout safe.

The independent collision monitor checks all scan angles around the circular footprint, including the rear when reversing, and computes relative-velocity time to collision for tracked obstacles approaching from any direction. It distinguishes a safer escape from a dangerous approach so emergency braking does not unnecessarily cancel an evasive reverse command. The raw-scan near-stop distance is 0.13 m in the direction of travel; tracked imminent-contact stopping uses a 0.35-second TTC threshold, with caution below 0.9 seconds. Emergency braking cancels forward inertia immediately, an explicit idealization. Turning remains possible. The planner retries when the path is blocked or progress remains stopped for about one second.

### Frontier exploration

Frontiers are known-free cells adjacent to unknown cells. Spatially sampled representatives are sorted by distance; the first A*-reachable candidate is selected. The robot repeats as it discovers new space. Coverage reports observed cells divided by total rectangular map area, not an oracle of reachable free space. “Exploration complete” means no remaining reachable frontier representatives, not proof every hidden cell has been observed.

## ROS 2 / Nav2 equivalents

| TypeScript module | ROS 2 / Nav2 equivalent |
| --- | --- |
| world.ts | Gazebo-like world, differential-drive physics, ground-truth contacts |
| lidar.ts | sensor_msgs/LaserScan publisher |
| odometry.ts | nav_msgs/Odometry wheel-odometry publisher |
| mapper.ts | slam_toolbox-style mapper publishing nav_msgs/OccupancyGrid; known poses only here |
| costmap.ts | nav2_costmap_2d inflation layer |
| planner.ts | Nav2 planner server (NavFn / Smac conceptual equivalent; this implementation is A*) |
| controller.ts | Nav2 Regulated Pure Pursuit controller |
| tracker.ts | Obstacle-tracker / spatio-temporal voxel-layer-like dynamic observation concept, projected into 2D; not a literal STVL implementation |
| dwa.ts | Nav2 DWB-controller-style sampled, acceleration-limited trajectory scoring |
| recovery.ts | Nav2 behavior-server-style local clearance recovery, without costmap clearing |
| safety.ts | nav2_collision_monitor-style all-around zones and time-to-collision checks |
| explorer.ts | explore_lite-style frontier exploration |
| metrics.ts | Navigation evaluation / diagnostics recorder |
| bus.ts | Typed, in-process ROS-style publish/subscribe graph |
| simulation.ts | Deterministic executor and node orchestration |
| benchmark.worker.ts | Headless evaluation process |

Modules are under src/robotics, independent of React and DOM rendering. This is an educational architectural analogy, not binary-compatible ROS messages or an implementation of Nav2.

## Topics and rates

| Topic | Payload | Nominal simulated rate |
| --- | --- | --- |
| /scan | LaserScan | 10 Hz |
| /odom | Odometry | 60 Hz |
| /map | OccupancyGrid (log odds) | 10 Hz |
| /tracked_obstacles | TrackedObstacles: estimated circle tracks and velocities | 10 Hz |
| /plan | Path | On planning / replanning |
| /cmd_vel | Twist | 20 Hz |
| /goal_pose | Point / goal pose position | On user / explorer goal |

Costmap inflation runs at 5 Hz; DWA and collision monitoring run at 20 Hz. The live graph shows measured publications over a rolling two-second **simulation-time** window. These are not network rates. OccupancyGrid.data stores log odds rather than ROS's serialized −1 / 0–100 values; a ROS bridge would convert them. Scan clustering and bounded, short-horizon sampling keep computation independent of the render loop; approximately 60 fps remains the rendering target, not a universal device guarantee.

## Benchmark

Open Benchmark, choose N trials (default 20, up to 200), select a layout and seed in its configuration panel, and run. Navigation runs without rendering in a dedicated Web Worker, so the interactive interface remains available. Cancel terminates the worker. The benchmark snapshots its configuration independently of the live simulator and always enables moving obstacles. Numerical inputs are clamped to safe simulator ranges when a run starts.

Each trial derives a seed deterministically, samples collision-free start/goal poses separated by at least 3 m, and uses a coarse ground-truth **static** planning check only to ensure the pair is connected before the trial. The robot itself starts with an unknown map and never receives this oracle. Static reachability does not guarantee success around dynamic obstacles. Every trial uses the same 60 Hz physics/navigation code as the live simulator, with a 150 simulated-second timeout.

The results show success rate, mean/median successful time to goal, mean successful driven-distance / straight-line-distance ratio, mean replans across all trials, and total contacts across all trials. Failed runs remain in the trial table and exported CSV; their timeout times are not mixed into successful-time statistics. No successful trials produces a zero-valued time/ratio summary, not a claim of instantaneous navigation. A zero success rate must be interpreted alongside those fields. Goal success uses the robot's estimated pose within 0.2 m of its goal, so odometry drift can make estimated arrival disagree with ground truth.

Contact counters are split into static (walls/furniture) and dynamic (patrollers), plus whether the robot was moving or stopped at contact entry. “Moving” means absolute linear velocity > 0.02 m/s or absolute angular velocity > 0.05 rad/s immediately before collision response. Every obstacle-pair contact episode is counted, including contacts initiated by a patroller while the robot is stopped. A 0.015 m separation hysteresis prevents centimeter-level chatter from creating multiple episodes. Static + dynamic and moving + stopped each sum to total contacts. Live metrics and each benchmark row expose all four counts; the benchmark summary also shows the number of trials with contacts.

Export CSV includes those counts and a quoted JSON contact-event field with simulation timestamp, static/dynamic category, physical obstacle identifier (metrics only), pre-response velocities, and moving/stopped flag. These are measurements of this simplified simulator, not claims of real-world robot performance.

### Measured benchmark: 50 trials per layout, base seed 7

Measured on 2026-11-27 using the same headless worker logic as the app, with moving obstacles enabled. Other settings: 180 beams, 360° field of view, 6 m range, 0.8 m/s maximum forward speed, 0.6 m lookahead, 0.12 m global route margin, 0.04 m local footprint margin, 0.35 m/s patroller speed, exact odometry, 60 Hz physics, and a 150 simulated-second trial timeout. Trial seeds use base seed + zero-based trial index × 7919. No truth-based obstacle prediction, patroller avoidance, obstacle removal, or collision-counter exclusions were used.

| Layout | Success | Mean time (s) | Median time (s) | Mean driven / straight | Mean replans |
| --- | --- | --- | --- | --- | --- |
| Simple room | 100% (50/50) | 17.646 | 16.200 | 1.198 | 2.08 |
| Office with doorways | 100% (50/50) | 27.373 | 20.425 | 1.664 | 6.14 |
| Cluttered warehouse | 90% (45/50) | 28.714 | 19.900 | 1.611 | 6.50 |

| Layout | Total contacts | Static | Dynamic | While moving | While stopped | Trials with contacts |
| --- | --- | --- | --- | --- | --- | --- |
| Simple room | 2 | 0 | 2 | 0 | 2 | 1/50 |
| Office with doorways | 4 | 0 | 4 | 0 | 4 | 3/50 |
| Cluttered warehouse | 3 | 0 | 3 | 0 | 3 | 3/50 |
| Total | 9 | 0 | 9 | 0 | 9 | 7/150 |

Time and ratio statistics include successful trials only. Replans and contacts include all trials, including timeouts.

**This is an improvement, not a collision-free result.** The reported pre-change baseline was 148/150 goals reached, 32 contacts across 27/150 trials, all dynamic. The measured updated stack reached 145/150 goals, with 9 dynamic contacts across 7 trials: fewer contact events and affected trials, but three fewer successful goals. The added predictive safety incurs longer paths and can still yield into a deadlock. All nine residual contacts occurred while stopped and remain counted as avoidance failures. They are not dismissed because the patroller initiated the contact.

Reproduction cases needing more work: simple trial 4; office trials 6, 9, and 50; warehouse trials 14, 22, and 50. Warehouse trials 6, 7, 24, 48, and 49 timed out. These trial numbers are one-based within their layout. The new architecture is implemented, but eliminating all remaining stopped-robot contacts and recovering warehouse success requires further work; no guarantee of zero dynamic contacts is claimed.

## Honest limitations

- 2D only; no height, camera, wheel slip, inertia, complex contact response, or full rigid-body dynamics.
- **Mapping with known poses, not full SLAM.** Optional odometry noise drifts without scan matching or loop closure.
- A* treats unknown space optimistically; a hidden obstacle may invalidate a planned route.
- Chamfer inflation approximates Euclidean distance; paths are geometric, not dynamically optimized.
- Moving circles patrol by reversing near static walls; no human intention model.
- Scan/track-based safety is not certified. Occlusion, a narrow field of view, first-scan ambiguity, uncertain circle fits, and track association errors can still cause missed or false hazards.
- Constant-velocity prediction cannot foresee a blind patroller suddenly reversing at a wall. There is no formal collision-free guarantee, global time-optimal planning, or complete recovery policy for every dynamic deadlock.
- All metrics, topic rates, and benchmark times use simulated time.
- Browser performance varies; 60 fps is a target, not a universal guarantee.

## Next steps

1. Scan matching and loop closure for real SLAM rather than known-pose mapping.
2. Compare alternative DWB critics or a model-predictive local controller against the current DWA-style planner.
3. A ROS 2 bridge via rosbridge, converting these typed messages into standard ROS messages.
4. Multi-hypothesis obstacle motion prediction, recovery behaviors, and richer benchmark scenario suites.

