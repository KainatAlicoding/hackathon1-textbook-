# Week 4: TF (Transform) Frames

This week, we will learn about TF frames, which are crucial for tracking coordinate frames over time in ROS 2. We will cover:

- What are TF frames and why are they important?
- Broadcasting and listening to transforms.
- TF trees and debugging tools.
- Practical applications of TF.

- What are TF frames and why are they important?

	TF (transform) frames are a system for keeping track of multiple coordinate frames over time. Each frame has a name and a transform (translation + rotation) describing its pose relative to a parent frame. TF lets nodes ask questions like "where is the camera relative to the robot base at time t?" and automatically handles interpolation of transforms over time. This is essential for sensor fusion, motion planning, mapping, and any task that needs consistent geometry across different sensors and components.

- Broadcasting and listening to transforms.

	Nodes that know a transform between two frames broadcast that transform (using a TF broadcaster). Consumers use TF listeners to query the transform at a specific time or to subscribe to transform updates. In ROS 2, transform data is published on the `/tf` (dynamic) and `/tf_static` (static) topics. Use `tf2_ros` APIs in `rclpy` or `rclcpp` to implement broadcasters and listeners.

	Example: publish a static transform from the command line:

```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link
```

	Example (conceptual `rclpy` broadcaster):

```python
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node

class StaticFramePublisher(Node):
	def __init__(self):
		super().__init__('static_frame_pub')
		self.broadcaster = TransformBroadcaster(self)
		t = TransformStamped()
		t.header.frame_id = 'base_link'
		t.child_frame_id = 'camera_link'
		t.transform.translation.x = 0.1
		t.transform.rotation.w = 1.0
		self.broadcaster.sendTransform(t)

def main():
	rclpy.init()
	node = StaticFramePublisher()
	rclpy.spin_once(node)
	node.destroy_node()
	rclpy.shutdown()
```

- TF trees and debugging tools.

	The TF tree (or frame graph) shows parent/child relationships between frames. Use these tools to inspect and debug TF data:

	- `ros2 topic echo /tf` and `ros2 topic echo /tf_static` to view raw transform messages.
	- `ros2 run tf2_tools view_frames` (requires Graphviz) to generate a visualization of the frame tree.
	- `rqt_tf_tree` or `rqt_graph` for interactive visualizations.
	- `tf2_echo` (if installed) to query a specific transform between two frames at runtime.

	When debugging, check for missing links in the tree, incorrect parent/child relationships, wrong frame naming, or stale timestamps causing transform lookups to fail.

- Practical applications of TF.

	TF is used wherever spatial relationships matter: transform sensor data into a common frame (e.g., camera -> base_link), compute the pose of detected objects in map coordinates, align odometry to a map, and provide transforms for controllers and planners. Best practices: choose stable, descriptive frame names (`base_link`, `map`, `odom`, `camera_link`), publish static transforms with `/tf_static`, keep dynamic transform rates consistent, and prefer timestamps from the sensor when available so listeners can interpolate accurately.