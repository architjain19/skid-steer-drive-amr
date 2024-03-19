# Installation:

- ROS Humble Installation Documentation: (install ros-humble desktop)
> https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

- Install Gazebo for simulation environment:
> https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install
> Note: Source gazebo setup file if any plugin or files are not loading `source /usr/share/gazebo/setup.sh`

- Install Gazebo ROS Packages and Plugins for ros-humble:
> `sudo apt install ros-humble-gazebo-ros-pkgs`
> `sudo apt-get install ros-humble-gazebo-plugins`

- Install other necessary packages:
> `sudo apt-get install ros-humble-xacro`
> `sudo apt-get install ros-humble-joint-state-publisher`
> `sudo apt-get install ros-humble-joint-state-publisher-gui`

# Execution Instructions:

- Firstly, launch the gazebo world:
> `ros2 launch amr_description start_world_launch.py`

- Then, launch this file for mapping:
> `ros2 launch amr_navigation amr_navigation.launch.py`

- Once map is saved, instead of navigation, you can launch this file for navigation:
    > `ros2 launch amr_navigation amr_mapping.launch.py`