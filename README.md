# User Package – auto_bag_interfaces

This package is used on the **client side** to interact with the MoCap PC **server** via the `RecordTopics` service.

The server is responsible for recording ROS 2 topics into a bag file, based on a YAML configuration string sent by the client.

---

# Full working client example

import rclpy
from rclpy.node import Node
from auto_bag_interfaces.srv import RecordTopics
import yaml

class RecordTopicsClient(Node):
    def __init__(self):
        super().__init__('record_topics_client')
        self.cli = self.create_client(RecordTopics, '/record_topics')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.req = RecordTopics.Request()

    def send_request(self, yaml_file_path):

# Load and convert YAML file to string

        with open(yaml_file_path, 'r') as f:
            yaml_dict = yaml.safe_load(f)
            yaml_str = yaml.dump(yaml_dict)

        self.req.yaml_text = yaml_str
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Response: success={future.result().success}, message='{future.result().message}'")
        else:
            self.get_logger().error("Service call failed.")

def main():
    rclpy.init()
    client = RecordTopicsClient()
    client.send_request('path/to/topics.yaml')  # Replace with actual path
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()