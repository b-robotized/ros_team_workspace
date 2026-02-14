

## Moving the robot using MoveIt2
> **NOTE:** If you are not familiar with MoveIt2, check the docs for some [useful examples](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

To move the robot using Moveit2, we first bring up the robot with the mock hardware enabled:
```
ros2 launch <robot_bringup_package> $CELL_NAME$.launch.xml use_mock:=true
```

After that, in another terminal we launch MoveIt2:
```
ros2 launch $CELL_NAME$_moveit moveit.launch.xml
```
Now we can use the `MotionPlanning` widget in `rviz2` to assign goals, plan and execute motions.
