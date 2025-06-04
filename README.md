"""
README – auto_bag_interfaces (Client Usage Guide)

This package provides the service definition for communicating with the auto_bag server,
which records a list of ROS 2 topics into bag files.

────────────────────────────────────────────────────────────────────

1. Service Definition: RecordTopics.srv

Request:
    string command         # "start" or "stop"
    string[] topics        # List of topics to record

Response:
    bool success
    string message

────────────────────────────────────────────────────────────────────

2. YAML Format Example

Your input YAML file should look like this:

topics:
  - /topic/one
  - /topic/two
  - /topic/three

────────────────────────────────────────────────────────────────────

3. Python Client Example

```python
import rclpy
from rclpy.node import Node
from auto_bag_interfaces.srv import RecordTopics
import yaml

class RecordClient(Node):
	def __init__(self):
		super().__init__('record_client')
		self.cli = self.create_client(RecordTopics, 'record_topics')
		while not self.cli.wait_for_service(timeout_sec=2.0):
			self.get_logger().info('Waiting for service...')
		self.req = RecordTopics.Request()

	def send_request(self, command: str, yaml_path: str):
		try:
			with open(yaml_path, 'r') as f:
				data = yaml.safe_load(f)
				topics = data.get('topics', [])
				if not topics:
					self.get_logger().error("No topics found in YAML file.")
					return
				self.req.command = command
				self.req.topics = topics
		except Exception as e:
			self.get_logger().error(f"Failed to load YAML: {e}")
			return

		future = self.cli.call_async(self.req)
		rclpy.spin_until_future_complete(self, future)
		if future.result():
			self.get_logger().info(f"Server response: {future.result().message}")
		else:
			self.get_logger().error("Service call failed.")

def main():
	rclpy.init()
	node = RecordClient()

	# Modify these values
	command = "start"  # or "stop"
	yaml_path = "path/to/topics.yaml"

	node.send_request(command, yaml_path)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
```


────────────────────────────────────────────────────────────────────

4. How to Run

source install/setup.bash
python3 your_client_script.py

────────────────────────────────────────────────────────────────────

5. Notes

- Make sure the auto_bag server is running before you launch this script.
- This interface package only provides the service definition.
  The actual server behavior is handled by the auto_bag package.
"""

