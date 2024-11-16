# KINOVA Gen3 Control Packages Based On [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/index.html) 

This guild was performed on a x86_64 Linux System with Ubuntu 22.04. The computer carried a NVIDIA GeForce RTX 4090.

## Installation and Preparation of Docker Dev Env
### For Ubuntu 22.04 please starts here
1. Install Docker Engine from [this guild](https://docs.docker.com/engine/install/ubuntu/).

2. Configure Docker for rootless access [here](https://docs.docker.com/engine/install/linux-postinstall/).

3. Follow the [Developer Environment Setup](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html) to install nvidia-container-toolkit, Git LFS, and setup `~/workspaces/isaac_ros-dev/src` as `ISAAC_ROS_WS`.

For Step 1, select `On x86_64 platforms`.
For step 4, select `x86_64 and Jetson without SSD`.

### Install [ISAAC ROS Commons](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
```
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

Once finished, navigate to the `src` folder, and clone this repository and `isaac_ros_common`:
```
cd ${ISAAC_ROS_WS}/src && \
   git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

### Obtain the repository
> **_NOTE:_**  make sure to clone it into the the right repo
```
cd ${ISAAC_ROS_WS}/src && \
   git clone https://github.com/kczttm/ros2_kinova_ws.git
```

Navigate to the project directory and copy `.isaac_ros_common-config` file to the home directory:
```
cd ros2_kinova_ws && \
   cp ./.isaac_ros_common-config ~/
```
### Entering Docker
We will add the following shortcut to build a docker env according to `~/.isaac_ros_common-config`
```
echo "alias ldb='cd ${ISAAC_ROS_WS}src/isaac_ros_common && ./scripts/run_dev.sh'" >> ~/.bashrc
echo "alias ld='cd ${ISAAC_ROS_WS}src/isaac_ros_common && ./scripts/run_dev.sh --skip_image_build'" >> ~/.bashrc
```
Note that `ldb` will build the docker first then launch it. `ld` will just launch what has already been built.

Now we start by with a fresh build:
```
source ~/.bashrc
ldb
```

### Inside Docker
For the first time building (or you have made changes non-python files under `${ISAAC_ROS_WS}src/`) build the package:
```
colcon build --symlink-install
source /workspaces/isaac_ros-dev/install/setup.bash
```

## Package Overviews

### [gen3_7dof](/src/gen3_7dof/gen3_7dof/README.md)
This package directly command the KINOVA Gen3 7Dof arm using the [KORTEX API](https://github.com/Kinovarobotics/kortex/tree/master/api_python/examples/102-Movement_high_level)

The method apriltag_follower work with the [Isaac ROS AprilTag](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/index.html) package to follow a 30 mm april tag with 10 mm +z spacing.

To launch the apriltag_chaser node, setup `Isaac ROS AprilTag package, then:
```
ros2 launch gen3_7dof apriltag_chaser.launch.py
```

### [vision_pkg](/src/vision_pkg/vision_pkg/README.md)
This package demonstrate the way to calibrate usb_cam using ROS2 

## Common bug and Solutions
### GPU memory occupied by prior application on host computer
```
ERROR gxf/std/unbounded_allocator.cpp@58: Failure in cudaMalloc. cuda_error: cudaErrorUnknown, error_str: unknown error
```

Potential Solution: Restart the nvidia_uvm module from [Andre_NG on reddit](https://www.reddit.com/r/VFIO/comments/15b0z78/comment/kofs4t3/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button)
1. Identify the processes using the "nvidia_uvm" module. in terminal run `fuser -v /dev/nvidia*` This will display the processes using the NVIDIA GPU. (In my case, it was pt_main_thread which was from running main.ipynb)

2. Once you identify the processes, you can either stop those processes (run "Restart" button on top) or terminate them. `sudo kill -9 123` (<-- EXAMPLE for process with ID 123)

3. After terminating the processes, run:
```
sudo modprobe -r nvidia_uvm && sudo modprobe nvidia_uvm
```
