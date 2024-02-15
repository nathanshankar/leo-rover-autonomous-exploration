# Leo Rover Autonomous Exploration

Leo Rover represents the pinnacle of innovation in autonomous exploration technology, revolutionizing the way we navigate and explore remote environments. Designed to tackle the most challenging terrains with precision and efficiency, Leo Rover utilizes frontier exploration techniques to autonomously chart uncharted territories, making it an indispensable tool for scientists, researchers, and explorers alike.

At the heart of Leo Rover's capabilities lies its advanced autonomy system, which leverages cutting-edge algorithms to analyze the surrounding environment in real-time. Equipped with a suite of sensors including LiDAR, cameras, and inertial measurement units (IMUs), Leo Rover meticulously maps its surroundings, identifying obstacles, terrain features, and potential hazards with remarkable accuracy.

One of Leo Rover's standout features is its utilization of frontier exploration strategies. By employing frontier-based algorithms, Leo Rover intelligently selects the most promising paths for exploration, prioritizing unexplored regions while avoiding previously visited areas. This allows Leo Rover to maximize its efficiency and coverage, ensuring thorough exploration of even the most expansive and rugged landscapes.

Leo Rover's robust mechanical design further enhances its capabilities in harsh environments. With rugged, all-terrain wheels, robust suspension systems, and a durable chassis, Leo Rover is built to withstand the rigors of off-road exploration, traversing rocky terrain, sandy deserts, and icy tundras with ease.

Remote operation and monitoring of Leo Rover are facilitated through an intuitive user interface, enabling operators to oversee missions, adjust parameters, and analyze data in real-time. Additionally, Leo Rover's modular design allows for easy customization and integration of additional sensors or payloads, further expanding its versatility and utility across a wide range of applications.

Whether it's conducting geological surveys, mapping uncharted territories, or scouting potential sites for exploration, Leo Rover stands ready to lead the way, pushing the boundaries of autonomous exploration and unlocking new frontiers of knowledge and discovery. With Leo Rover at the helm, the future of exploration has never looked brighter.

# How to use:
```console
git clone https://github.com/nathanshankar/leo-rover-autonomous-exploration.git
```

# Simulation:
https://github.com/nathanshankar/leo-rover-autonomous-exploration/assets/66565433/462c39cf-6fc1-4d7c-ad56-a1654acd67a0

# Methods of Control

## 1. Manual Control through teleop_twist_keyboard (User Priority)

The `teleop_twist_keyboard` package provides a simple keyboard teleoperation interface for controlling a robot that subscribes to the `geometry_msgs/Twist` message, which typically controls the linear and angular velocity of the robot.

### Controls:

- **Linear Motion**:
  - `i` key: Move the robot forward (increase linear velocity).
  - `,` (comma) key: Move the robot backward (decrease linear velocity).
- **Angular Motion**:
  - `l` key: Rotate the robot counterclockwise (increase angular velocity).
  - `.` (period) key: Rotate the robot clockwise (decrease angular velocity).
- **Stopping the Robot**:
  - `k` key: Stop the robot's motion (set linear and angular velocity to 0).
- **Exiting the Teleop Mode**:
  - `q` key: Quit the teleop mode and exit the program.

When using `teleop_twist_keyboard`, you'll typically run the node, and then you can use the specified keys on your keyboard to control the robot's motion. This is useful for testing and debugging robot motion without needing physical hardware or a complex control interface.

## 2. Autonomous Control: Frontier Exploration

Frontier exploration is a fundamental concept in robotics and autonomous systems, particularly in the context of robotic exploration of unknown or unstructured environments. It involves the systematic exploration of an environment to discover and map regions that have not been previously visited or adequately surveyed.

### Definition and Objective:
Frontier exploration aims to autonomously guide robots to explore and map unknown environments efficiently and effectively. The primary objective is to identify and visit areas of the environment that have not yet been explored while maximizing coverage and minimizing redundant exploration.

### Key Components and Techniques:
1. **Frontier Detection**: Robots equipped with sensors, such as LiDAR, cameras, or depth sensors, scan their surroundings to detect areas where the environment transitions from known to unknown.
2. **Frontier Selection**: Once frontiers are detected, the robot employs algorithms to prioritize them based on various criteria.
3. **Path Planning**: After selecting a frontier, the robot plans an optimal path to reach it while avoiding obstacles.
4. **Navigation and Control**: The robot executes the planned path, relying on its navigation and control systems to move safely and accurately.
5. **Mapping and Localization**: As the robot explores, it continuously updates its map of the environment and refines its localization estimates.
