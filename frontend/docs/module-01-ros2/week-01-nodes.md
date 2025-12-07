# Week 1: ROS 2 Nodes

This week, we will introduce ROS 2 nodes, the fundamental building blocks of any ROS 2 system. We will cover:

- What are nodes?

	ROS 2 nodes are the primary units of computation and encapsulation in a ROS system. Each node is a logical container for functionality — it typically owns publishers, subscribers, service and action servers/clients, timers, and parameters. Nodes provide a clear separation of concerns and make it easy to distribute functionality across processes or machines.

	Key concepts:

	- Responsibilities: a node implements a single logical role (for example, sensor interface, controller, planner, or state estimator) and exposes communication endpoints (topics, services, actions) to interact with other nodes.

	- Process model and composition: nodes can run as independent OS processes (helpful for fault isolation) or be composed into a single process using component composition (which reduces IPC overhead and startup complexity).

	- Naming and namespaces: nodes have names and live in namespaces; namespacing and remapping let you reuse node implementations across robots and environments without code changes.

	- Parameters: nodes expose configuration via parameters which can be set at launch time or changed at runtime to adapt behavior without code changes.

	- Communication primitives: nodes communicate using topics (asynchronous streams), services (synchronous RPC-style calls), and actions (preemptible long-running goals). Choosing the right primitive depends on latency, reliability, and interaction pattern.

	Example (conceptual): a `camera_node` might publish image frames on `/camera/image_raw`, provide a service `/camera/set_exposure`, and expose parameters for resolution and framerate.

	Common pitfalls: avoid packing multiple unrelated responsibilities into one node (which makes testing and reuse harder), and watch blocking work inside callbacks — use timers, executors, or worker threads to keep the node responsive.

- Creating and running nodes.

	You create nodes using language-specific client libraries such as `rclpy` (Python) or `rclcpp` (C++). Typical steps are: define a node class, create publishers/subscribers/clients/servers, and let the executor spin to process callbacks (for example `rclpy.spin(node)`). Nodes are usually launched with `ros2 run` or `ros2 launch` and can be packaged into ROS 2 packages for distribution and deployment.

- Node lifecycles.

	ROS 2 supports managed (lifecycle) nodes which expose explicit states and transitions (unconfigured, inactive, active, finalized). Lifecycle nodes make startup, configuration, activation, and shutdown deterministic and help with safe resource allocation and system bring-up. The lifecycle API and `lifecycle_msgs` allow tools and launch scripts to control node state transitions.

- Best practices for node design.

	Aim for single-responsibility nodes that are easy to test and reuse. Prefer composition of small nodes over monolithic processes when it makes sense. Use parameters for configuration, structured logging for observability, and appropriate QoS settings for reliable communication. Avoid long blocking operations in callbacks (use worker threads or timers), handle shutdown gracefully, and write unit and integration tests for node behavior.