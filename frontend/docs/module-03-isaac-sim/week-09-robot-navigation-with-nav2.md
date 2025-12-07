# Week 9: Robot Navigation with Nav2

This week, we will explore advanced robot navigation using the Nav2 stack in ROS 2, integrated with Isaac Sim. We will cover:

- Introduction to Nav2.
- Configuring navigation stacks.
- Autonomous navigation in simulated environments.
- Path planning and obstacle avoidance.

- Introduction to Nav2.

	Nav2 (Navigation2) is the ROS 2 navigation framework that provides a complete, modular stack for autonomous mobile robot navigation: localization, mapping, global and local planning, obstacle avoidance, and lifecycle-based bring-up. Nav2 is composed of lifecycle-aware nodes so you can deterministically configure and activate the navigation pipeline during system bring-up.

- Configuring navigation stacks.

	A typical Nav2 setup includes these components:

	- `map_server` — serves an occupancy grid map (often from a pre-built map file).
	- `amcl` — Adaptive Monte Carlo Localization for estimating the robot pose on the map.
	- Global planner — computes a long-range path (options: `navfn`, `smac_planner`).
	- Local controller — trajectories and velocity commands for following the path (options: `dwb_controller`, `reg_nhc`).
	- Costmaps (global & local) — maintain obstacle layers and inflation to support planners and controllers.
	- `bt_navigator` — executes behavior trees to manage navigation tasks (send goals, recoveries).
	- `lifecycle_manager` — orchestrates the configure/activate lifecycle transitions for Nav2 nodes.

	Configuration is primarily driven by parameter YAML files (commonly named `nav2_params.yaml`) and behavior tree XML files. Example launch command (replace paths appropriately):

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=/path/to/map.yaml \
  params_file:=/path/to/nav2_params.yaml \
  use_sim_time:=true
```

	The `params_file` contains scoped sections for each Nav2 node (costmap, controller, planner, AMCL, etc.). Tune parameters like `controller_frequency`, `inflation_radius`, `planner_timeout`, and `collision_check_distance` to match your robot and environment.

- Autonomous navigation in simulated environments.

	When testing Nav2 in simulation (Gazebo, Isaac Sim, or Unity), make sure:

	- Sensor topics (lidar, depth/camera) and `tf` frames are published with correct names and QoS settings.
	- `use_sim_time` is enabled so Nav2 and other ROS nodes use the simulation clock.
	- Robot state (joint states / odometry) and transforms are published (e.g., `robot_state_publisher` + `tf` or simulation bridge).

	Typical simulation bring-up flow:

	1. Start the simulator and spawn the robot (ensure the robot publishes `/odom`, sensors, and `/tf`).
	2. Launch Nav2 bringup (map server + AMCL or localization alternative). Provide the simulation map if available.
	3. Open RViz and use the Nav2 panels to send navigation goals, or send goals programmatically.

	Example: programmatically send a goal using the `nav2_simple_commander` package (conceptual):

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

navigator = BasicNavigator()
goal = PoseStamped()
goal.header.frame_id = 'map'
goal.pose.position.x = 1.0
goal.pose.position.y = 0.5
goal.pose.orientation.w = 1.0
navigator.goToPose(goal)
```

- Path planning and obstacle avoidance.

	Nav2 separates global path planning from local trajectory generation. The global planner finds a path on the static map while the local controller and costmaps handle dynamic obstacles and real-time replanning. Important concepts and knobs:

	- Costmap layers: sensor sources, obstacle layers, voxel layers (for 3D) and inflation layers determine how nearby obstacles are treated.
	- Planner selection: choose a planner type that fits your environment; `smac` (search-based) works well for complex maps while `navfn` is simpler and fast.
	- Controller tuning: local controllers (DWB or others) have parameters for acceleration limits, sampling resolution, and obstacle avoidance aggressiveness.
	- Behavior Tree (BT): Nav2 uses BTs to sequence navigation steps and recovery behaviors. Customize or swap BT XML files to change high-level behaviors (e.g., how the robot recovers from stuck states).

	Monitoring and debugging tools:

	- `ros2 topic echo`, `rviz2` visualization of global plan, local plan, costmaps, and sensor data.
	- `ros2 service call /<node>/get_loggers` or Nav2 introspection tools for runtime diagnostics.

	Common issues and fixes:

	- Robot drifts: check odometry / frame alignment and AMCL settings.
	- Goals unreachable: verify map correctness, inflation radius, and planner timeouts.
	- High CPU usage: reduce sensor rates, simplify collision geometry, or run fewer plugins.

	Best practices:

	- Validate navigation on simulation before deploying to hardware.
	- Keep simulation and production parameter sets separate; version control your `nav2_params.yaml` and BT XML.
	- Use lifecycle manager for clean startup and graceful shutdown.
	- Log and record runs with `ros2 bag` for offline debugging and parameter tuning.