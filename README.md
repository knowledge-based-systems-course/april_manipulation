# april_manipulation

Functionality:

- moveit configuration for mia_hand_on_ur5 robot
- moveit configuration for ur5 robot
- move arm to predefined configurations defined in joint space
- pick node using simple pregrasp planner

# Run instructions to launch moveit move group

Configure environment:

        export ROBOT=mia_hand_on_ur5

Run bringup:

        roslaunch april_robot_bringup robot.launch sim:=true

Launch move group:

        roslaunch mia_hand_on_ur5_moveit_config move_group.launch
        # See move_group notes below

run moveit commander:

        rosrun moveit_commander moveit_commander_cmdline.py

type interactively in the moveit commander terminal:

        use arm
        go home
        go up

Done, arm should move now

# Moveit move group notes

By default moveit is launched with slow arm speed for safety reasons.
When using the simulator you can speed up the arm using:

        roslaunch mia_hand_on_ur5_moveit_config move_group.launch default_velocity_scaling_factor:=1.0 default_acceleration_scaling_factor:=1.0

From code, the recommended way to include moveit (in the context of april) is:

        <!-- moveit move group -->
        <arg name="use_pointcloud" default="true"/>
        <!-- arm default speed -->
        <arg name="default_velocity_scaling_factor" default="1.0"/> <!-- fast arm, use it like this only for simulation -->
        <arg name="default_acceleration_scaling_factor" default="1.0"/> <!-- fast arm, use it like this only for simulation -->
        <include file="$(find mia_hand_on_ur5_moveit_config)/launch/move_group.launch" >
            <arg name="use_pointcloud" value="$(arg use_pointcloud)"/>
            <!-- arm speed -->
            <arg name="default_velocity_scaling_factor" value="$(arg default_velocity_scaling_factor)"/>
            <arg name="default_acceleration_scaling_factor" value="$(arg default_acceleration_scaling_factor)"/>
        </include>

# Pick node

Please see the readme inside the april_pick_place_object folder
