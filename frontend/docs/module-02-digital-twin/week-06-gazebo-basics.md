# Week 6: Gazebo Basics and Physics

This week, we will introduce Gazebo, a powerful 3D robot simulator. We will cover:

- Getting started with Gazebo.
- Simulating robots and environments.
- Understanding Gazebo physics engine.
- Creating and manipulating worlds.

- Getting started with Gazebo.

	Gazebo is a feature-rich 3D simulator for testing robots, sensors, and controllers in realistic environments. With ROS 2, use the `gazebo_ros` packages (part of the ROS 2 ecosystem) to bridge ROS topics, services, and parameters with the simulator. To start a simple Gazebo session with ROS 2 integration, use a launch file provided by `gazebo_ros`, for example:

```bash
# start Gazebo with the default empty world (example)
ros2 launch gazebo_ros gazebo.launch.py
```

	The Gazebo GUI provides tools to pause/unpause physics, inspect models, change camera views, and examine logs. For headless CI runs or faster simulation, start Gazebo without the GUI using appropriate launch arguments.

- Simulating robots and environments.

	Robots are typically described using URDF/SDF. For simulation, SDF is the native format but URDF (often generated from `xacro`) can be converted and published to Gazebo via the `robot_state_publisher` and `spawn_entity` utilities. To spawn a robot from a URDF published on the `robot_description` parameter:

```bash
# spawn using robot_description topic
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity my_robot

# or spawn directly from an SDF file
ros2 run gazebo_ros spawn_entity.py -file path/to/robot.sdf -entity my_robot
```

	Environments (worlds) combine terrain, lights, and static objects. Gazebo ships several example worlds and supports custom worlds written in SDF. Use the GUI or world files to place models and tune lighting for sensor realism.

- Understanding Gazebo physics engine.

	Gazebo uses a physics engine (ODE, Bullet, DART, or Simbody depending on build) to simulate rigid-body dynamics, collisions, joints, and contacts. Key tunable settings include solver type, real-time factor, max step size, and update rate. When setting up controllers and sensors:

	- Provide realistic mass and inertia values in your URDF/SDF `inertial` tags.
	- Keep collision geometry simple to improve performance.
	- Tune friction and restitution for realistic contact behavior.

	To control physics behavior at runtime, use Gazebo services or launch-time parameters; for automated testing, monitor the real-time factor to ensure the simulation runs at acceptable speed.

- Creating and manipulating worlds.

	World files (SDF) describe the static environment and initial model placement. Create or edit world files to add terrain, lights, and objects. You can save a scene from the GUI or write SDF manually. For programmatic manipulation, publish model state updates or use Gazebo services such as `/gazebo/spawn_entity`, `/gazebo/delete_entity`, and `/gazebo/set_model_state`.

	Example: spawn a model programmatically:

```bash
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'box', xml: '<sdf version=\"1.6\">...'</sdf>', robot_namespace: '', initial_pose: { position: { x: 1.0, y: 0.0, z: 0.5 } } }"
```

	Best practices: separate simulation-only plugins (sensors, fake hardware) from production code, version-control your world and model files, and use deterministic seeds for randomized elements when running reproducible experiments.