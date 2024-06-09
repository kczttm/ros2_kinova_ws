# Move the Robot to one pose from terminal
## First see where the robot currently is 
In one terminal:
```
ros2 run gen3_7dof chase_controller
```

open a new terminal:
```
ros2 topic echo /gen3_7dof/EE_pose_topic --once
```

suppose we get:
```
---
position:
  x: -0.22083130478858948
  y: 0.42648670077323914
  z: 0.4363674223423004
orientation:
  x: 0.4281393423632258
  y: -0.5767096056554706
  z: -0.5419545592930683
  w: 0.43633472234851145
```

## Then move the robot just a bit away from that spot
Open a third terminal, move the robot in +x direction for 0.1 meters:
```
ros2 topic pub --once /gen3_7dof/desired_pose_topic geometry_msgs/msg/Pose "{position: {x: -0.1208, y: 0.4264, z: 0.4363}, orientation: {x: 0.4281, y: -0.5767, z: -0.5419, w: 0.4363}}"
```
