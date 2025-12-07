# Week 7: Unity Integration for Digital Twins

This week, we will explore integrating Unity for creating advanced digital twins. We will cover:

- Introduction to Unity for robotics simulation.
- Importing robot models.
- Connecting Unity with ROS 2.
- Creating interactive simulation environments.

- Introduction to Unity for robotics simulation.

	Unity is a powerful real-time 3D engine well suited for building high-fidelity digital twins, interactive visualizations, and sensor-rich simulations. For robotics, Unity offers flexible rendering, physics, and scripting (C#) which allow you to simulate sensors (RGB, depth, LiDAR), custom controllers, and operator interfaces. Unity's frame rate and rendering pipeline give you control over fidelity vs performance, making it a strong complement to physics-first simulators like Gazebo when you need realistic visuals or complex user interaction.

- Importing robot models.

	Robots can be imported into Unity as meshes (FBX, OBJ) or generated from URDF via helper tools (Unity Robotics Hub provides URDF importers). When importing:

	- Convert meshes to an efficient runtime format and create simplified collision meshes for physics.
	- Retain hierarchy and joint naming so that controllers and message mappings remain consistent with your ROS setup.
	- Use prefabs to store robot configurations, sensors, and scripts for easy reuse across scenes.

- Connecting Unity with ROS 2.

	For ROS 2 integration, the recommended approach is Unity's Robotics Hub packages (ROS-TCP-Connector) which provide a Unity-side client and a lightweight ROS-side bridge (ros2_tcp_endpoint). The connector serializes Unity objects and messages over TCP and maps them to ROS topics, services, and actions.

	Typical workflow:

	1. Install the Unity Robotics packages (ROS-TCP-Connector) into your Unity project (via Unity Package Manager or by importing the package).
	2. In Unity, add a `ROSConnection` object and configure publishers/subscribers that map Unity data (transforms, sensor frames, floats/vectors) to ROS message types.
	3. Run the ROS-side bridge (`ros2 run <bridge_package> <bridge_node>` or a provided launch script) to accept TCP connections from Unity.
	4. Start the Unity scene — it will connect to the bridge and begin exchanging messages (for example, publish `/camera/image_raw` or subscribe to `/cmd_vel`).

	Minimal example patterns:

	- Unity publishes sensor data to ROS (topic): camera images, point clouds, or transforms.
	- Unity subscribes to ROS commands (topic/service): velocity commands (`/cmd_vel`) or high-level mission commands.

	Example (conceptual C# pattern):

```csharp
// pseudo-code using ROS-TCP-Connector APIs
public class CmdVelSubscriber : MonoBehaviour {
  ROSConnection ros;
  void Start(){
    ros = ROSConnection.GetOrCreateInstance();
    ros.Subscribe<TwistMsg>("/cmd_vel", OnCmdVel);
  }
  void OnCmdVel(TwistMsg msg){
    // apply to Unity physics / character controller
  }
}
```

	Notes: Unity uses its own update loop; when integrating physics and ROS messages, buffer incoming messages and apply them in FixedUpdate() to keep physics stable. Also ensure timestamps and frame names align with your ROS TF conventions.

- Creating interactive simulation environments.

	Unity excels at building interactive worlds: UI overlays, operator controls, dynamic objects, and game-like scenarios for human-in-the-loop testing. Use Unity's physics layers and colliders for efficient collision checks, and attach sensors to prefabs so they move with the robot. For repeatable experiments, seed random elements and automate scene setup with editor scripts or launch-time configuration.

	Best practices and tips:

	- Use the Unity Robotics Hub's provided message serializers to avoid custom serialization bugs.
	- Keep collision geometry simple for performance; use simplified meshes for physics and high-detail meshes for visuals only.
	- Use simulated time in ROS (`/use_sim_time`) so logs and replay tools remain consistent between Unity and ROS 2.
	- Profile performance (frame rate, GC allocations) and tune update rates for sensors to balance realism and throughput.

	If you want, I can add a step-by-step mini-lab: install the Unity Robotics package, spawn a robot prefab, run the bridge, and send `/cmd_vel` from ROS to move the robot in Unity. Would you like that added as a hands-on exercise?