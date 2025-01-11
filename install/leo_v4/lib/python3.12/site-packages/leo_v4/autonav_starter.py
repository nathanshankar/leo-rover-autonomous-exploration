import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import subprocess

class ButtonLauncherNode(Node):
    def __init__(self):
        super().__init__('button_launcher')
        self.publisher = self.create_publisher(String, 'gui_button_topic', 10)

    def send_button_click_message(self):
        msg = String()
        msg.data = 'launch_button_clicked'
        self.publisher.publish(msg)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Launch Button")
        self.setGeometry(100, 100, 300, 200)

        self.button = QPushButton('Launch auto nav', self)
        self.button.setGeometry(50, 50, 200, 50)
        self.button.clicked.connect(self.launch_ros2_launch_file)

    def launch_ros2_launch_file(self):
        button_launcher_node = ButtonLauncherNode()
        button_launcher_node.send_button_click_message()

        # Launching leo_explorer/explore.launch.py
        subprocess.Popen(['ros2', 'launch', 'leo_explorer', 'explore.launch.py'])

def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
