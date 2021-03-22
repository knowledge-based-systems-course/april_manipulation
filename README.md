# april_manipulation

Functionality:

- moveit configuration for mia_hand_on_ur5 robot
- moveit configuration for ur5 robot

# Run instructions

Configure environment:

        export ROBOT=mia_hand_on_ur5

Run bringup:

        roslaunch april_robot_bringup robot.launch sim:=true

unpause physics:

        rosservice call /gazebo/unpause_physics "{}"

Launch move group:

        roslaunch mia_hand_on_ur5_moveit_config move_group.launch

run moveit commander:

        rosrun moveit_commander moveit_commander_cmdline.py

type interactively in the moveit commander terminal:

        use arm
        go home
        go up

Done, arm should move now
