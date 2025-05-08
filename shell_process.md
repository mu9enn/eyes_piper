# moveit

```
bash can_activate.sh can0 1000000
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
source devel/setup.bash
```
