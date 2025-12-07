# Week 3: URDF (Unified Robot Description Format)

This week, we will delve into URDF, a powerful XML format for describing robots. We will cover:

- Structure of a URDF file.
- Defining links and joints.
- Adding visual and collision properties.
- Using URDF with RViz.

- Structure of a URDF file.

	A URDF (Unified Robot Description Format) file is an XML document that describes a robot's kinematic and (optionally) dynamic model. The top-level element is `<robot name="...">` and the file contains repeated `link` and `joint` elements that together form the robot's kinematic tree. A typical URDF also includes optional `material`, `transmission`, and `gazebo` tags for visualization, simulation, and controller integration. For larger robots, URDF files are often generated or templated using `xacro` (XML macros) to avoid duplication.

- Defining links and joints.

	Links represent rigid bodies and are defined with a unique name plus optional `inertial`, `visual`, and `collision` child elements. Joints connect two links and define the relative transform and motion type (`fixed`, `revolute`, `continuous`, `prismatic`, `floating`, `planar`). Each joint specifies a `parent` link, a `child` link, an `origin` (pose), and joint-specific parameters (axis, limits). Correct parent/child relationships are crucial — URDF expresses a tree (not a general graph), where one link is the root.

- Adding visual and collision properties.

	The `visual` element specifies how the link should appear in viewers (geometry: box, cylinder, sphere, mesh; and material/color). The `collision` element defines simplified geometry used for collision checking — it's best practice to keep collision geometry simpler than visual geometry for simulation performance. If dynamics or controllers are used, add a robust `inertial` element with mass and inertia values; incorrect inertials lead to unstable simulations.

- Using URDF with RViz.

	To visualize a URDF in RViz, publish the robot description to the `robot_description` parameter (commonly by running `robot_state_publisher`), and ensure joint states are published (e.g., with `joint_state_publisher` or a hardware driver). You can load your URDF with a launch file or pass it as a parameter at runtime. RViz displays the `robot_description` visual elements and updates as joint states change; use `ros2 run rviz2 rviz2` or an equivalent launch to open the tool.

	Example minimal URDF snippet:

```xml
<robot name="simple_bot">
	<link name="base_link"/>
	<link name="wheel_link"/>
	<joint name="wheel_joint" type="continuous">
		<parent link="base_link"/>
		<child link="wheel_link"/>
		<origin xyz="0 0 -0.1" rpy="0 0 0"/>
		<axis xyz="0 1 0"/>
	</joint>
</robot>
```

	Tips: use `xacro` for parametrized URDFs, keep collision meshes simple, include accurate `inertial` data for simulation, and validate your URDF with `check_urdf` or by loading it in RViz before using it in controllers or Gazebo.