# moveit

```
source devel/setup.bash
```

```
bash can_activate.sh can0 1000000
```

```
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2
```

```
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false
```


```
rosservice call /joint_moveit_ctrl_endpose "joint_endpose: [0.099091, 0.008422, 0.246447, -0.09079689034052749, 0.7663049838381912, -0.02157924359457128, 0.6356625934370577]
max_velocity: 0.5
max_acceleration: 0.5" 
```

```
sunx@sunx-Dell:~/code_proj/piper_ros$ rostopic echo /end_pose
header: 
  seq: 1
  stamp: 
    secs: 1746691801
    nsecs: 101052522
  frame_id: ''
pose: 
  position: 
    x: 0.29918
    y: 0.158215
    z: 0.546709
  orientation: 
    x: -0.5348467462521367
    y: 0.5559670270530007
    z: -0.4009418111941521
    w: 0.4940498829973581
---
header: 
  seq: 2
  stamp: 
    secs: 1746691801
    nsecs: 105129718
  frame_id: ''
pose: 
  position: 
    x: 0.29918
    y: 0.158215
    z: 0.546709
  orientation: 
    x: -0.5348467462521367
    y: 0.5559670270530007
    z: -0.4009418111941521
    w: 0.4940498829973581
---
header: 
  seq: 3
  stamp: 
    secs: 1746691801
    nsecs: 110637664
  frame_id: ''
pose: 
  position: 
    x: 0.29918
    y: 0.158215
    z: 0.546709
  orientation: 
    x: -0.5348467462521367
    y: 0.5559670270530007
    z: -0.4009418111941521
    w: 0.4940498829973581
---
header: 
```


```
sunx@sunx-Dell:~/code_proj/piper_ros$ rostopic echo /joint_states
header: 
  seq: 29308
  stamp: 
    secs: 1746691818
    nsecs: 768511772
  frame_id: ''
name: 
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  - joint7
  - joint8
position: [0.6228161076581155, 1.675233533541314, -2.2109422333004463, -0.6607026337863795, 0.9603146184289711, 0.38110546472366735, 0.0, 0.0]
velocity: []
effort: []
---
header: 
  seq: 29309
  stamp: 
    secs: 1746691818
    nsecs: 778451204
  frame_id: ''
name: 
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  - joint7
  - joint8
position: [0.6228161076581155, 1.675233533541314, -2.2109422333004463, -0.6607026337863795, 0.9603146184289711, 0.38110546472366735, 0.0, 0.0]
velocity: []
effort: []
---
header: 
  seq: 29310
  stamp: 
    secs: 1746691818
    nsecs: 788527965
  frame_id: ''
name: 
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  - joint7
  - joint8
position: [0.6228161076581155, 1.675233533541314, -2.2109422333004463, -0.6607026337863795, 0.9603146184289711, 0.38110546472366735, 0.0, 0.0]
velocity: []
effort: []
---
header: 
  seq: 29311
  stamp: 
    secs: 1746691818
    nsecs: 798505306
  frame_id: ''
name: 
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
  - joint7
  - joint8
position: [0.6228161076581155, 1.675233533541314, -2.2109422333004463, -0.6607026337863795, 0.9603146184289711, 0.38110546472366735, 0.0, 0.0]
velocity: []
effort: []
---

```
