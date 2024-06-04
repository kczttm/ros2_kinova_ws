# KINOVA Gen3 Control Packages Based On [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/index.html) 

This guild was performed on a x86_64 Linux System with Ubuntu 22.04. The computer carried a NVIDIA GeForce RTX 4090.

## Installation and Preparation of Docker Dev Env
First follow the [Isaac ROS Developer Environment Setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html) to install Docker, nvidia-container-toolkit, and Git LFS. 

For Step 1, select `On x86_64 platforms`.
For step 4, select `x86_64 and Jetson without SSD`.

Once finished, navigate to the `src` folder, and clone this repository and `isaac_ros_common`:
```
cd ${ISAAC_ROS_WS}/src && \
   git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git && \
   git clone https://github.com/kczttm/ros2_kinova_ws.git
```

Copy `.isaac_ros_common-config` file to the home directory:
```
cd ros2_kinova_ws
cp ./.isaac_ros_common-config ~/
```

To run the docker
```
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh
```

## Package Overviews

### [gen3_7dof](/src/gen3_7dof/gen3_7dof/README.md)
This package directly command the KINOVA Gen3 7Dof arm using the [KORTEX API](https://github.com/Kinovarobotics/kortex/tree/master/api_python/examples/102-Movement_high_level)

The method apriltag_follower work with the [Isaac ROS AprilTag](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/index.html) package to follow a 30 mm april tag with 10 mm +z spacing.

To launch the apriltag_chaser node, setup `Isaac ROS AprilTag package, then:
```
source /workspaces/isaac_ros-dev/install/setup.bash && \
   ros2 launch gen3_7dof apriltag_chaser.launch.py
```

### [vision_pkg](/src/vision_pkg/vision_pkg/README.md)
This package demonstrate the way to calibrate usb_cam using ROS2 