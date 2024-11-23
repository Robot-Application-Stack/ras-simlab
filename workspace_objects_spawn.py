import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import yaml

class ContainerSpawner(Node):
    def __init__(self, yaml_file_path):
        super().__init__('container_spawner')
        self.publisher = self.create_publisher(Pose, '/spawn_model', 10)
        self.timer = self.create_timer(1.0, self.spawn_containers)  # Timer to trigger publishing
        self.containers = self.load_yaml(yaml_file_path)
        self.current_index = 0

    def load_yaml(self, file_path):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['containers']

    def spawn_containers(self):
        if self.current_index < len(self.containers):
            container = self.containers[self.current_index]

            # Create Pose message
            pose_msg = Pose()
            pose_msg.position.x = container['position'][0]
            pose_msg.position.y = container['position'][1]
            pose_msg.position.z = container['position'][2]
            pose_msg.orientation.x = container['orientation'][0]
            pose_msg.orientation.y = container['orientation'][1]
            pose_msg.orientation.z = container['orientation'][2]
            pose_msg.orientation.w = container['orientation'][3]

            self.get_logger().info(f"Spawning container {container['id']} at {pose_msg.position} with orientation {pose_msg.orientation}")
            
            # Publish Pose message
            self.publisher.publish(pose_msg)
            self.current_index += 1
        else:
            self.get_logger().info("All containers spawned.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    yaml_file_path = 'containers.yaml'  # Path to your YAML file
    container_spawner = ContainerSpawner(yaml_file_path)
    rclpy.spin(container_spawner)
    container_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
