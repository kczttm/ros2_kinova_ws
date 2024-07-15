# To check camera's supported format, open a terminal
```
v4l2-ctl --device=/dev/video2 --list-formats
```

# Calibrating the camera using ROS2
## In first terminal
```
ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/my_camera --params-file src/ros2_kinova_ws/src/vision_pkg/config/params_endoscope.yaml
```

## In second terminal 
```
ros2 run camera_calibration cameracalibrator --size 5x6 --square 0.0286 --ros-args -r image:=/my_camera/image_raw -r camera:=/my_camera
```
* Note that count the edge not the checker blocks!!!!

## Lastly Save the Result
```
tar -xvf /tmp/calibrationdata.tar.gz -C ./ ost.yaml && mv ost.yaml camera_info_endoscope.yaml
```

## Endoscope Result
```
**** Calibrating ****
mono pinhole calibration...
D = [0.04842971187430401, -0.093931916025017, -0.003760140924062491, -0.004636118825134327, 0.0]
K = [1170.2880719748991, 0.0, 817.2710182516167, 0.0, 1173.04130269531, 599.5357454161584, 0.0, 0.0, 1.0]
R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P = [1172.7640380859375, 0.0, 809.502450307984, 0.0, 0.0, 1181.7200927734375, 595.1825632246328, 0.0, 0.0, 0.0, 1.0, 0.0]
None
# oST version 5.0 parameters


[image]

width
1600

height
1200

[narrow_stereo]

camera matrix
1170.288072 0.000000 817.271018
0.000000 1173.041303 599.535745
0.000000 0.000000 1.000000

distortion
0.048430 -0.093932 -0.003760 -0.004636 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
1172.764038 0.000000 809.502450 0.000000
0.000000 1181.720093 595.182563 0.000000
0.000000 0.000000 1.000000 0.000000
```

# Run the Apriltag 6D Pose Pub
## In the first terminal
```
ros2 launch vision_pkg isaac_ros_apriltag_endoscope.launch.py
```

## In the second terminal
```
rviz2 -d /workspaces/isaac_ros-dev/src/isaac_ros_apriltag/isaac_ros_apriltag/rviz/usb_cam.rviz
```

## To change apriltag size after printing the tag
Navigate to [`isaac_ros_apriltag_endoscope.launch.py`](/src/vision_pkg/launch/isaac_ros_apriltag_endoscope.launch.py).
Edit the field `size` under the `apriltag_node`