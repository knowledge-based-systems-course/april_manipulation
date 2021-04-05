#!/usr/bin/env python3

import sys
import copy
import rospy
import tf
import moveit_commander

from std_msgs.msg import String
from std_srvs.srv import Empty
from cob_perception_msgs.msg import DetectionArray
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from planning_scene_interface import PlanningSceneInterface # Using file from source! wait until moveit binaries are updated, then delete this line and uncomment next one
# from moveit_commander import PlanningSceneInterface

class PickTools():
    def __init__(self):

        # parameters
        arm_group_name = rospy.get_param('~arm_group_name', 'arm')
        arm_goal_tolerance = rospy.get_param('~arm_goal_tolerance', 0.01)
        gripper_close = rospy.get_param('~gripper_close', None)
        gripper_open = rospy.get_param('~gripper_open', None)
        self.gripper_joint_names = rospy.get_param('~gripper_joint_names', None)
        self.gripper_joint_efforts = rospy.get_param('~gripper_joint_efforts', None)
        self.pick_attempts = rospy.get_param('~pick_attempts', 5)
        object_name = None
        self.object_padding = rospy.get_param('~object_padding', 0.04)
        self.grasp_quality = rospy.get_param('~grasp_quality', 1.0)
        # pregrasp parameters
        self.pre_grasp_approach_min_dist = rospy.get_param('~pre_grasp_approach/min_dist', 0.08)
        self.pre_grasp_approach_desired = rospy.get_param('~pre_grasp_approach/desired', 0.25)
        self.pre_grasp_approach_axis = rospy.get_param('~pre_grasp_approach/axis', [0.0, 1.0, 0.0])
        # post grasp retreat parameters
        self.post_grasp_retreat_frame_id = rospy.get_param('~post_grasp_retreat/frame_id', 'world')
        self.post_grasp_retreat_min_dist = rospy.get_param('~post_grasp_retreat/min_dist', 0.1)
        self.post_grasp_retreat_desired = rospy.get_param('~post_grasp_retreat/desired', 0.15)
        self.post_grasp_retreat_axis = rospy.get_param('~post_grasp_retreat/axis', [0.0, 0.0, 1.0])

        if not gripper_close or not gripper_open:
            raise ValueError('mandatory parameters gripper_close or/and gripper_open not set')

        if not self.gripper_joint_names or not self.gripper_joint_efforts:
            raise ValueError('mandatory parameter gripper_joint_names or/and gripper_joint_efforts not set')

        # publications and subscriptions
        rospy.Subscriber('~event_in', String, self.eventInCB)
        rospy.Subscriber('/object_recognition/detections', DetectionArray, self.objectRecognitionCB)
        self.event_out_pub = rospy.Publisher('~event_out', String, queue_size=1)
        self.trigger_perception_pub = rospy.Publisher('/object_recognition/event_in', String, queue_size=1)

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander(arm_group_name, wait_for_servers=10.0)
        self.arm.set_goal_tolerance(arm_goal_tolerance)
        self.scene = PlanningSceneInterface()

        # setup publishers for visualisation purposes
        self.grasp_pose_pub = rospy.Publisher('~visualisation/grasp_pose', PoseStamped, queue_size=1)
        rospy.sleep(0.5) # give some time for publisher to register

        self.gripper_open_trajectory = self.make_gripper_trajectory(gripper_open)
        self.gripper_close_trajectory = self.make_gripper_trajectory(gripper_close)

        # flag to indicate perception callback was triggered potentially having detected objects
        self.obj_recognition_received = False
        self.detections_msg = None

    def eventInCB(self, msg):
        self.event_out_pub.publish(self.pick_object(msg.data))

    def objectRecognitionCB(self, msg):
        rospy.loginfo('received object recognition data')
        self.detections_msg = msg
        self.obj_recognition_received = True

    def make_object_pose(self, object_name, obj_recog_msg):
        '''
        iterate over detected objects, check if object_name is among them, if not exit
        extract object pose and bounding box of a particular object of interest (object_name)
        '''
        object_pose = PoseStamped()
        reference_frame = obj_recog_msg.header.frame_id
        if reference_frame:
            object_pose.header.frame_id = reference_frame
        else:
            rospy.logwarn('object detections reference frame not set')
        for detection in obj_recog_msg.detections:
            if detection.label == object_name:
                bounding_box_x = detection.bounding_box_lwh.x + self.object_padding
                bounding_box_y = detection.bounding_box_lwh.y + self.object_padding
                bounding_box_z = detection.bounding_box_lwh.z + self.object_padding
                if detection.pose.header.frame_id != object_pose.header.frame_id:
                    rospy.logwarn('recognized object reference frame is not same as all objects reference frame')
                    object_pose.header.frame_id = detection.pose.header.frame_id
                if not object_pose.header.frame_id:
                    rospy.logerr('object recognition frame_id not set')
                    return
                object_pose.pose.position.x =     detection.pose.pose.position.x
                object_pose.pose.position.y =     detection.pose.pose.position.y
                object_pose.pose.position.z =     detection.pose.pose.position.z
                object_pose.pose.orientation.x =  detection.pose.pose.orientation.x
                object_pose.pose.orientation.y =  detection.pose.pose.orientation.y
                object_pose.pose.orientation.z =  detection.pose.pose.orientation.z
                object_pose.pose.orientation.w =  detection.pose.pose.orientation.w
                return object_pose, [bounding_box_x, bounding_box_y, bounding_box_z]
        rospy.logerr('object to be picked was not perceived')
        return

    def clean_scene(self):
        '''
        iterate over all object in the scene, delete them from the scene
        '''
        for item in self.scene.get_known_object_names():
            self.scene.remove_world_object(item)

    def clear_octomap(self, octomap_srv_name='clear_octomap'):
        '''
        call service to clear octomap
        '''
        rospy.loginfo('clearing octomap')
        rospy.ServiceProxy(octomap_srv_name, Empty)()

    def move_arm_to_posture(self, arm_posture_name):
        '''
        use moveit commander to send the arm to a predefined arm configuration
        defined in srdf
        '''
        rospy.loginfo(f'moving arm to {arm_posture_name}')
        self.arm.set_named_target(arm_posture_name)
        self.arm.go()

    def make_gripper_trajectory(self, joint_angles):
        '''
        Set and return the gripper posture as a trajectory_msgs/JointTrajectory
        '''
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint_names
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = joint_angles
        trajectory_point.effort = self.gripper_joint_efforts
        trajectory_point.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectory_point)
        return trajectory

    def make_gripper_translation_msg(self, frame_id, min_dist=0.08, desired=0.25, axis=[1.0, 0.0, 0.0]):
        '''
        make moveit_msgs/GripperTranslation msg
        '''
        
        # make empty msg of type moveit_msgs/GripperTranslation
        # see: http://docs.ros.org/en/api/moveit_msgs/html/msg/GripperTranslation.html
        gripper_translation_msg = GripperTranslation()

        # the direction of the translation
        gripper_translation_msg.direction.header.frame_id = frame_id
        gripper_translation_msg.direction.vector.x = axis[0]
        gripper_translation_msg.direction.vector.y = axis[1]
        gripper_translation_msg.direction.vector.z = axis[2]

        # the desired translation distance
        gripper_translation_msg.desired_distance = desired

        # the min distance that must be considered feasible before the grasp is even attempted
        gripper_translation_msg.min_distance = min_dist

        return gripper_translation_msg

    def make_grasps_msgs(self, object_name, grasp_pose):
        '''
        generate grasp configurations for moveit
        grasp_pose : The position of the end-effector for the grasp
        '''

        # make empty msg of type moveit_msgs/Grasp
        # see http://docs.ros.org/en/api/moveit_msgs/html/msg/Grasp.html
        g = Grasp()

        # The estimated probability of success for this grasp, or some other
        # measure of how 'good' it is.
        g.grasp_quality = self.grasp_quality

        # The internal posture of the hand for the pre-grasp
        # only positions are used
        g.pre_grasp_posture = self.gripper_open_trajectory

        # The approach direction to take before picking an object
        g.pre_grasp_approach = self.make_gripper_translation_msg(self.arm.get_end_effector_link(),
            min_dist=self.pre_grasp_approach_min_dist, desired=self.pre_grasp_approach_desired, axis=self.pre_grasp_approach_axis)

        # The position of the end-effector for the grasp.  This is the pose of
        # the 'parent_link' of the end-effector, not actually the pose of any
        # link *in* the end-effector.  Typically this would be the pose of the
        # most distal wrist link before the hand (end-effector) links began.
        g.grasp_pose = grasp_pose

        # The internal posture of the hand for the grasp
        # positions and efforts are used
        g.grasp_posture = self.gripper_close_trajectory

        # The retreat direction to take after a grasp has been completed (object is attached)
        g.post_grasp_retreat = self.make_gripper_translation_msg(self.post_grasp_retreat_frame_id,
            min_dist=self.post_grasp_retreat_min_dist, desired=self.post_grasp_retreat_desired, axis=self.post_grasp_retreat_axis)

        # the maximum contact force to use while grasping (<=0 to disable)
        g.max_contact_force = 1.0

        # an optional list of obstacles that we have semantic information about
        # and that can be touched/pushed/moved in the course of grasping
        g.allowed_touch_objects = [object_name]

        # A name for this grasp
        g.id = 'top_grasp'

        # NOTE : one could change the orientation and generate more grasps, currently the list has only 1 grasp

        return [g]

    def test_going_to_cartesian_pose(self):
        '''
        this is a test function, does not belong to pick, remove
        goto cartesian pose with moveit
        '''

        # home pose expressed in cartesian coordinates, obtained via:
        # rosrun tf tf_echo world hand_ee_link
        target_pose = PoseStamped()

        # object in cartesian coordinates (is also recorded as "object" posture in srdf)
        translation = [-0.000950, 0.682979, 1.158076]
        rotation = [-0.116526, 0.609924, -0.207705, -0.755825]

        # home posture in cartesian coordinates
        #translation = [-0.288703, 0.154675, 1.592202]
        #rotation = [0.612385, -0.684744, -0.190438, -0.346182]

        # current pose, when doing bringup arm is already there
        #translation = [-0.596009, -0.291193, 1.481584]
        #rotation = [-0.703003, 0.082303, 0.701726, 0.081197]

        rospy.loginfo(f'planning frame : {self.arm.get_planning_frame()}')
        target_pose.header.frame_id =     self.arm.get_planning_frame() # 'world'
        target_pose.pose.position.x =     translation[0]
        target_pose.pose.position.y =     translation[1]
        target_pose.pose.position.z =     translation[2]
        target_pose.pose.orientation.x =  rotation[0]
        target_pose.pose.orientation.y =  rotation[1]
        target_pose.pose.orientation.z =  rotation[2]
        target_pose.pose.orientation.w =  rotation[3]
        rospy.loginfo('going to cartesian pose')
        self.grasp_pose_pub.publish(target_pose)
        self.arm.set_pose_target(target_pose, end_effector_link='hand_ee_link')
        self.arm.go()
        rospy.loginfo('finished going to cartesian pose...')
        rospy.spin()

    def pick_object(self, object_name):
        '''
        1) move arm to a position where the attached camera can see the scene (octomap will be populated)
        2) clear octomap
        3) add table and object to be grasped to planning scene
        4) generate a list of grasp configurations
        5) call pick moveit functionality
        '''
        rospy.loginfo(f'picking object : {object_name}')

        # flag to keep track of the state of the grasp
        success = False

        # trigger perception to recognize objects
        rospy.loginfo('triggering perception to recognize objects')
        self.trigger_perception_pub.publish(String(object_name))
        rospy.sleep(1.0)

        # make sure objects are recognized
        if not self.obj_recognition_received:
            rospy.logerr('object recognition has not detected any objects')
            return

        # move arm to pregrasp in joint space, not really needed, can be removed
        self.move_arm_to_posture('home')

        # ::::::::: setup planning scene
        rospy.loginfo('setup planning scene')

        # remove all objects from the planning scene
        self.clean_scene()

        # add a table to the planning scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'world'
        box_x = 1.39
        box_y = 0.85
        box_z = 1.0
        table_pose.pose.position.x = 0.06
        table_pose.pose.position.y = 0.69
        table_pose.pose.position.z = box_z / 2.0
        table_pose.pose.orientation.x = 0.0
        table_pose.pose.orientation.y = 0.0
        table_pose.pose.orientation.z = 0.0
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box('table', table_pose, (box_x, box_y, box_z))

        # add an object to be grasped, data is comes from perception
        object_pose, bounding_box = self.make_object_pose(object_name, self.detections_msg)
        self.scene.add_box(object_name, object_pose, (bounding_box[0], bounding_box[1], bounding_box[2]))

        # print objects that were added to the planning scene
        rospy.loginfo(f'planning scene objects: {self.scene.get_known_object_names()}')

        # ::::::::: pick
        rospy.loginfo(f'picking object now')

        # generate a list of moveit grasp messages
        grasp_pose = PoseStamped()
        grasp_pose.header.frame_id = 'world' # TODO: do it w.r.t object_name

        # object in cartesian coordinates
        translation = [-0.000950, 0.682979, 1.158076]
        rotation = [-0.116526, 0.609924, -0.207705, -0.755825]

        grasp_pose.pose.position.x = translation[0]
        grasp_pose.pose.position.y = translation[1]
        grasp_pose.pose.position.z = translation[2]
        grasp_pose.pose.orientation.x = rotation[0]
        grasp_pose.pose.orientation.y = rotation[1]
        grasp_pose.pose.orientation.z = rotation[2]
        grasp_pose.pose.orientation.w = rotation[3]
        grasps = self.make_grasps_msgs(object_name, grasp_pose)

        # publish grasp pose for visualisation purposes
        self.grasp_pose_pub.publish(grasp_pose)

        ## remove octomap, table and object are added manually to planning scene
        #self.clear_octomap()

        # try to pick object, repeat x times if failure
        for attempt in range(self.pick_attempts):
            if self.robot.arm.pick(object_name, grasps):
                rospy.loginfo('moveit grasp object was called and returned')
                break
            else:
                rospy.logerr(f'grasp failed, attempt #{attempt}')
                rospy.sleep(0.2)

        # give feedback for publication
        if success:
            return String('e_success')
        return String('e_failure')

    def start_pick_node(self):
        # wait for trigger
        rospy.loginfo('ready to receive pick requests (publish object name on event_in topic)')
        rospy.spin()
        # shutdown moveit cpp interface before exit
        moveit_commander.roscpp_shutdown()

if __name__=='__main__':
    rospy.init_node('pick_object_node', anonymous=True)
    pick = PickTools()
    pick.start_pick_node()
    #pick.test_going_to_cartesian_pose()
