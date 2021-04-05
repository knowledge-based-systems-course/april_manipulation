# april_manipulation

Functionality:

- moveit configuration for mia_hand_on_ur5 robot
- moveit configuration for ur5 robot
- move arm to predefined configurations defined in joint space
- pick experimental script, so far going to a cartesian pose with the hand works

# Run instructions

Configure environment:

        export ROBOT=mia_hand_on_ur5

Run bringup:

        roslaunch april_robot_bringup robot.launch sim:=true

Launch move group:

        roslaunch mia_hand_on_ur5_moveit_config move_group.launch

run moveit commander:

        rosrun moveit_commander moveit_commander_cmdline.py

type interactively in the moveit commander terminal:

        use arm
        go home
        go up

Done, arm should move now

# Pick experimental node

So far it has been tested with a cylinder in the simple_grasping world

Launch world:

        export ROBOT_ENV=simple_grasping
        roslaunch april_robot_bringup robot.launch sim:=true

Rviz visualisation:

        rosrun rviz rviz --display-config `rospack find april_pick_place_object`/config/config.rviz

Launch required components:

        roslaunch april_pick_place_object pick_demo.launch

Launch pick demo node with appropiate parameters:

        roslaunch april_pick_place_object pick.launch

Trigger pick node:

        rostopic pub /pick_object_node/event_in std_msgs/String "data: 'cylinder'" --once

Done, now robot should grasp the object.
