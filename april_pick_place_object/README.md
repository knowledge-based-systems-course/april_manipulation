# april_pick_place_object

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
