# Week 2: ROS 2 Topics

This week, we will explore ROS 2 topics, the primary mechanism for asynchronous data exchange in ROS 2. We will cover:

- What are topics?
- Publishing and subscribing to topics.
- Quality of Service (QoS) settings.
- Introspection tools for topics.

- What are topics?

	Topics are named buses for asynchronous, many-to-many data exchange in ROS 2. A node publishes messages to a topic and any number of other nodes can subscribe to receive those messages. Topics are ideal for streaming sensor data, state updates, and other continuous flows of information where a loose coupling between producers and consumers is desired.

- Publishing and subscribing to topics.

	Publishing and subscribing are done via a node's publishers and subscribers. Publishers send typed messages to a topic, and subscribers register callbacks that are invoked when messages arrive. In code you create a publisher or subscriber using the client library API (for example `rclpy` or `rclcpp`) and use the executor to process incoming messages. From the command line you can test topics with `ros2 topic pub` (publish a message) and `ros2 topic echo` (print messages from a topic).

- Quality of Service (QoS) settings.

	QoS controls the middleware behavior for topics (reliability, durability, history, depth, deadline, lifespan, liveliness). Common options:

	- Reliability: `reliable` vs `best_effort` — use `reliable` when you need delivery guarantees (slower), `best_effort` for low-latency lossy streams.
	- Durability: `volatile` vs `transient_local` — `transient_local` preserves the last published message for late-joining subscribers.
	- History & depth: control how many messages are kept for delivery; use `keep_last` with an appropriate depth for bounded memory.

	Choosing QoS depends on network conditions, data criticality, and latency needs. We'll practice switching QoS profiles and observing behavior differences.

- Introspection tools for topics.

	ROS 2 provides several tools to inspect and debug topics:

	- `ros2 topic list` — list active topics.
	- `ros2 topic echo <topic>` — print messages published on a topic.
	- `ros2 topic hz <topic>` — measure message publish frequency.
	- `ros2 topic pub <topic> <msg_type> ...` — publish test messages from the CLI.
	- `rqt_graph` — visualize nodes and topic connections (useful for higher-level debugging).

	These tools help validate message flow, confirm data rates, and test QoS interactions during development.