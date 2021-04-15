# april_pick_place_object

So far it has been tested with a cylinder in the simple_grasping world

Launch world:

        export ROBOT_ENV=simple_grasping
        roslaunch april_robot_bringup robot.launch sim:=true

Rviz visualisation:

        rosrun rviz rviz --display-config `rospack find april_pick_place_object`/config/rviz/config.rviz

Launch required components:

        roslaunch april_pick_place_object pick_required_components.launch model_name:=my_object_name

replace "my_object_name" with any object from:

        ls `rospack find april_environments`/models

e.g cylinder, insole, tall_insole, etc.

If you need to adjust z spawn height, you can use z_spawn_position:=my_float_value, where my_float_value can be e.g 1.15.

Consider that the height of the table is 1.015 so the passed value should be above that.

Launch pick demo node with appropiate parameters:

        roslaunch april_pick_place_object pick_<my_grasp_planner>.launch

Where <my_grasp_planner> can be "handcoded_grasp_planner" or "simple_pregrasp_planner"

Trigger pick node:

        rostopic pub /pick_object_node/event_in std_msgs/String "data: 'cylinder'" --once

Done, now robot should grasp the object.

# rqt_grasp

Alternatively an experimental gui for manipulation is available called rqt_grasp.

to run do:

        rosrun rqt_grasp rqt_grasp

Currently very limited functionality is available, as this rqt was actually borrowed from another project.

The only buttons that currently work are:

- reset object pose

teleport cylinder back to the center of the table

- rand obj pose

teleport object to a random position on the table but with the cylinder always facing up

- grasp object

trigger pick pipeline to pick the object with the robot

# manually recording poses with gazebo *with space mouse*

For the handcoded grasp planner we provide with functionality to easily record grasp poses (transform between object and gripper end effector).

For that you can use the following commands:

        roslaunch april_pick_place_object teach_grasp_poses.launch object_name:=my_object_name

replace "my_object_name" with any object from:

        ls `rospack find april_environments`/models

Use the space mouse move the gripper to a graspable pose, then press enter in main launch file window (from step 1) to record a tf.

After you are done press q to exit and generate yaml file with all transforms, after recording multiple poses you can use the file to feed the handcoded grasp planner with the recorded data.

# manually recording poses with gazebo *without space mouse*

The node works best with a 3Dconnexion space mouse, however if you don't have the hardware alternatively you can use the teleop keyboard

Run the main launch file from previous point but selecting the option not to use the space mouse:

        roslaunch april_pick_place_object teach_grasp_poses.launch object_name:=my_object_name spacenav_node_required:=false

run teleop node to publish a 6D twist msg taking input from keyboard:

        rosrun april_pick_place_object teleop_twist_keyboard.py

all other steps are same as in "manually recording poses with gazebo with space mouse"
