# rtabmap_sim

## Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble** 
- **Gazebo Classic**

## Installation

1. Install RTAB-Map ROS package:
   ```bash
   sudo apt install ros-humble-rtabmap-ros

# Follow the commands
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

# Clone the repository
   git clone https://github.com/tirpudevk111/rtabmap_sim.git


# Build the package
   cd ~/ros2_ws
   colcon build --symlink-install

# launch the simulation.
   ros2 launch rtabmap_sim rtabmap_sim.launch.py use_sim_time:=true
   ```
   