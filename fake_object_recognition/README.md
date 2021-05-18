# fake_object_recognition

This node can be used as a mockup for object 6D pose estimation and classification.

Conceptual steps:

- Define a virtual box with position and dimensions that represents the field of view of a camera
- Query Gazebo for all objects in the scene
- Iterate over all objects, see if they are inside the box, if so then publish their pose as cob_perception_msgs/DetectionArray
- Additioanlly publish the object detections as tf

Virtual Box representing the field of view of the camera example:

<img src="https://github.com/aprilprojecteu/april_perception/blob/noetic/fake_object_recognition/doc/fov_perception_mockup.png" alt="drawing" width="600"/>

Object recognition example:

<img src="https://github.com/aprilprojecteu/april_perception/blob/noetic/fake_object_recognition/doc/object_detector_mockup.png" alt="drawing" width="600"/>

# Steps to test this component

First launch a roscore

        roscore

Run a gazebo simulation of your preference, for example:

        export ROBOT=mia_hand_on_ur5
        export ROBOT_ENV=inescop
        roslaunch april_robot_bringup robot.launch sim:=true
        roslaunch april_inescop simulated_use_case_6.launch

Run the fake object recognition node and rviz for visualisation purposes and rqt dynamic reconfigure to modify the fov box dimensions and spawn pose

        rosrun fake_object_recognition object_recognition_mockup.py
        rosrun rviz rviz # configure to listen to tf topic and marker, set fixed frame as "world"
        rosrun rqt_reconfigure rqt_reconfigure

Make sure a gazebo object is within the fov box, we recommend to spawn an insole and advance conveyor belt until such condition is met:

        # start conveyor belt a
        rosservice call /machinery/conveyor_belt_a/control "power: 75"
        # spawn insoles
        rostopic pub -r 0.3 /insole_spawner/spawn_object std_msgs/String "data: "
        # stop conveyor belt a once insoles are inside fov (green) box
        rosservice call /machinery/conveyor_belt_a/control "power: 0"

Listen to the detections topic:

        rostopic echo /object_recognition

Trigger the component (any string is fine):

        rostopic pub /perceive_obj_trigger std_msgs/String "data: ''" --once
