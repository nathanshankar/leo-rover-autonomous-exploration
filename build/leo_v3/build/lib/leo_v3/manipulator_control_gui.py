import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from math import pi
import numpy as np
from scipy.interpolate import CubicSpline
import time

class TrajectoryGeneratorThread(QThread):
    trajectory_finished = pyqtSignal()

    def __init__(self, node, target_positions, duration):
        super().__init__()
        self.node = node
        self.target_positions = target_positions
        self.duration = duration
        self.canceled = False

    def run(self):
        start_positions = self.node.get_current_joint_positions()
        splines = [CubicSpline([0, 1], [start, end], axis=0) for start, end in zip(start_positions, self.target_positions)]

        start_time = time.time()
        while True:
            elapsed_time = time.time() - start_time
            progress = min(1.0, elapsed_time / self.duration)

            interpolated_positions = [s(progress) for s in splines]

            if self.canceled:
                break

            self.node.set_joint_positions(interpolated_positions)

            if progress >= 1.0:
                break

            time.sleep(0.01)

        if not self.canceled:
            self.node.set_joint_positions(self.target_positions)
        self.trajectory_finished.emit()

    def cancel(self):
        self.canceled = True

class ManipulatorControlNode(Node):
    def __init__(self):
        super().__init__('manipulator_control_node')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.trajectory_thread = None

        self.init_ui()

    def init_ui(self):
        app = QApplication(sys.argv)
        widget = QWidget()
        widget.setWindowTitle('Manipulator Control GUI')

        layout = QVBoxLayout()

        # Define sliders and labels for each joint
        joints = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate', 'gripper', 'finger']
        self.sliders = []

        # Joint limits from xacro properties
        joint_limits = {
            'waist': (-pi + 0.00001, pi - 0.00001),
            'shoulder': (np.radians(-113), np.radians(111)),
            'elbow': (np.radians(-120), np.radians(95)),
            'wrist_angle': (np.radians(-100), np.radians(123)),
            'wrist_rotate': (-pi + 0.00001, pi - 0.00001),
            'gripper': (-pi, pi),  # Gripper joint is continuous
            'finger': (0.015, 0.037),
        }

        sleep_positions = [0.0, -1.7999999523162842, 1.5499999523162842, 0.800000011920929, 0.0, 0.0634808731572743, 0.026]

        for joint, sleep_position in zip(joints, sleep_positions):
            label = QLabel(f'{joint.capitalize()}:')
            slider = QSlider()
            slider.setOrientation(1)
            slider.setMinimum(0)
            slider.setMaximum(100)

            # Set joint limits
            min_limit, max_limit = joint_limits[joint]
            # Set finger slider to have a horizontal orientation
            if joint == 'finger':
                slider.setMinimum(int((min_limit / pi + 1) * 50*100))
                slider.setMaximum(int((max_limit / pi + 1) * 50*100))
                slider.setValue(int((sleep_position / pi + 1) * 50*100))
            else:
                slider.setMinimum(int((min_limit / pi + 1) * 50))
                slider.setMaximum(int((max_limit / pi + 1) * 50))
                slider.setValue(int((sleep_position / pi + 1) * 50))

            slider.valueChanged.connect(self.slider_value_changed)

            layout.addWidget(label)
            layout.addWidget(slider)

            self.sliders.append(slider)

        # Add buttons for sleep position and home position
        sleep_button = QPushButton('Go to Sleep Position')
        home_button = QPushButton('Go to Home Position')

        sleep_button.clicked.connect(self.go_to_sleep_position)
        home_button.clicked.connect(self.go_to_home_position)

        layout.addWidget(sleep_button)
        layout.addWidget(home_button)

        widget.setLayout(layout)
        widget.show()

        # Setup a QTimer to periodically call the send_joint_values method
        timer = QTimer(widget)
        timer.timeout.connect(self.send_joint_values)
        timer.start(100)  # Adjust the interval (in milliseconds) as needed

        sys.exit(app.exec_())

    def slider_value_changed(self):
        # Handle slider value changes if needed
        pass

    def go_to_sleep_position(self):
        target_positions = [0.0, -1.7999999523162842, 1.5499999523162842, 0.800000011920929, 0.0, 0.0634808731572743, 0.026]
        self.move_to_position(target_positions)

    def go_to_home_position(self):
        target_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.07808157145138353, 0.0206161841274051, -0.0206161841274051]
        self.move_to_position(target_positions)

    def move_to_position(self, target_positions, duration=2.0):
        if self.trajectory_thread:
            self.trajectory_thread.cancel()
            self.trajectory_thread.wait()

        self.trajectory_thread = TrajectoryGeneratorThread(self, target_positions, duration)
        self.trajectory_thread.trajectory_finished.connect(self.on_trajectory_finished)
        self.trajectory_thread.start()

    def on_trajectory_finished(self):
        self.get_logger().info('Trajectory finished')

    def interpolate_joint_positions(self, start_positions, target_positions, progress):
        cs = CubicSpline([0, 1], np.vstack([start_positions, target_positions]), axis=0)
        return cs(progress)

    def get_current_joint_positions(self):
        return [
            (slider.value() / 50.0 - 1) * pi if joint != 'finger' else (slider.value() / (50 * 100) - 1) * pi
            for slider, joint in zip(self.sliders, ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate', 'gripper', 'finger'])
        ]

    def set_joint_positions(self, positions):
        for slider, position, joint in zip(self.sliders, positions, ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate', 'gripper', 'finger']):
            if joint == 'finger':
                slider.setValue(int((position / pi + 1) * 50 * 100))
            else:
                slider.setValue(int((position / pi + 1) * 50))

    def send_joint_values(self):
        joint_values = self.get_current_joint_positions()

        # Adjust the finger joint value logic
        finger_slider_value = joint_values[-1]
        joint_values[-1] = finger_slider_value  # left_finger
        joint_values.append(-finger_slider_value)  # right_finger

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate', 'gripper', 'left_finger', 'right_finger']
        joint_state_msg.position = joint_values

        self.publisher.publish(joint_state_msg)
        self.get_logger().info(f'Sending joint states: {joint_values}')

def main(args=None):
    rclpy.init(args=args)

    manipulator_control_node = ManipulatorControlNode()

    rclpy.spin(manipulator_control_node)

    manipulator_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
