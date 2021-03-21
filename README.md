# april_manipulation

Functionality:

- moveit configuration for ur5 arm

# Run instructions

Run bringup:

        roslaunch april_robot_bringup_sim robot_mia_hand_on_ur5.launch

Run trajectory controller:

        roslaunch april_robot_bringup_sim mia_hand_on_ur5_traj_ros_control.launch

unpause physics:

        rosservice call /gazebo/unpause_physics "{}"

Launch move group:

        roslaunch april_robot_bringup_sim mia_hand_on_ur5_traj_ros_control.launch

run moveit commander:

        rosrun moveit_commander moveit_commander_cmdline.py

type interactively in the moveit commander terminal:

        use arm
        go home

Done, arm should move now
