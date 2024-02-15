import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',  # Replace 'map_topic' with your actual map topic
            self.map_callback,
            10)
        self.save_map_service = self.create_service(
            Empty,
            'save_map',
            self.save_map_callback)
        self.map_data = None

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info('Received map message')

    def save_map_callback(self, request, response):
        if self.map_data is not None:
            map_name = 'leo'  # Replace 'map_name' with your desired map name
            map_path = '/home/nathan/leo/src/leo_v2/maps/' + map_name  # Specify your desired save path
            if not os.path.exists(map_path):
                os.makedirs(map_path)
            map_file = os.path.join(map_path, map_name + '.pgm')
            with open(map_file, 'w') as f:
                f.write("P2\n")
                f.write("# Map saved from ROS\n")
                f.write(str(self.map_data.info.width) + " " + str(self.map_data.info.height) + "\n")
                f.write("255\n")
                for cell in self.map_data.data:
                    f.write(str(cell) + "\n")
            self.get_logger().info('Map saved successfully at: %s' % map_file)
        else:
            self.get_logger().warn('No map data available to save.')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
