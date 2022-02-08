#!/usr/bin/env python3

import sys
import importlib
import rospy
import moveit_commander
import tf
from tf import TransformListener

from std_msgs.msg import String
from std_srvs.srv import Empty
from cob_perception_msgs.msg import DetectionArray
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_commander import PlanningSceneInterface

class PickTools():
    def __init__(self):

        # parameters
        arm_group_name = rospy.get_param('~arm_group_name', 'arm')
        hand_group_name = rospy.get_param('~hand_group_name', 'hand')
        arm_goal_tolerance = rospy.get_param('~arm_goal_tolerance', 0.01)
        planning_time = rospy.get_param('~planning_time', 20.0)
        self.pregrasp_posture_required = rospy.get_param('~pregrasp_posture_required', False)
        self.pregrasp_posture = rospy.get_param('~pregrasp_posture', 'home')
        self.planning_scene_boxes = rospy.get_param('~planning_scene_boxes', [])
        #self.planning_scene_boxes = [] # remove
        self.clear_planning_scene = rospy.get_param('~clear_planning_scene', False)
        # configure the desired grasp planner to use
        import_file = rospy.get_param('~import_file', 'grasp_planner.simple_pregrasp_planner')
        import_class = rospy.get_param('~import_class', 'SimpleGraspPlanner')
        # TODO: include octomap

        # to be able to transform PoseStamped later in the code
        #self.transformer = tf.listener.TransformerROS()
        self.tf_listener = TransformListener()

        # import grasp planner and make object out of it
        self.grasp_planner = getattr(importlib.import_module(import_file), import_class)()

        # subscriptions and publications
        rospy.Subscriber('~event_in', String, self.eventInCB)
        rospy.Subscriber('~grasp_type', String, self.graspTypeCB) # TODO remove, get from actionlib
        self.grasp_type = 'side_grasp' # TODO remove, get from actionlib
        rospy.Subscriber('/object_recognition/detections', DetectionArray, self.objectRecognitionCB)
        self.event_out_pub = rospy.Publisher('~event_out', String, queue_size=1)
        self.trigger_perception_pub = rospy.Publisher('/object_recognition/event_in', String, queue_size=1)

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander(arm_group_name, wait_for_servers=10.0)
        self.robot.arm.set_planning_time(planning_time)
        self.hand = moveit_commander.MoveGroupCommander(hand_group_name, wait_for_servers=10.0)
        self.arm.set_goal_tolerance(arm_goal_tolerance)
        self.scene = PlanningSceneInterface()

        # flag to indicate perception callback was triggered potentially having detected objects
        self.obj_recognition_received = False
        self.detections_msg = None

        # keep memory about the last object we have grasped
        self.grasped_object = ''

    def graspTypeCB(self, msg):
        self.grasp_type = msg.data

    def eventInCB(self, msg):
        # TODO: task is missing, only object name can be specified now
        self.event_out_pub.publish(self.pick_object(msg.data, self.grasp_type))

    def objectRecognitionCB(self, msg):
        rospy.loginfo('received object recognition data')
        self.detections_msg = msg
        self.obj_recognition_received = True

    def transform_pose(self, pose, target_reference_frame):
        '''
        transform a pose from any rerence frame into the target reference frame
        '''
        self.tf_listener.getLatestCommonTime(target_reference_frame, pose.header.frame_id)
        return self.tf_listener.transformPose(target_reference_frame, pose)

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
        recognized_objects = [] # make a list of all recognized objects for debugging purposes
        for detection in obj_recog_msg.detections:
            recognized_objects.append(detection.label)
            if detection.label == object_name:
                padding = self.grasp_planner.get_object_padding()
                bounding_box_x = detection.bounding_box_lwh.x + padding
                bounding_box_y = detection.bounding_box_lwh.y + padding
                bounding_box_z = detection.bounding_box_lwh.z + padding
                if detection.pose.header.frame_id != object_pose.header.frame_id:
                    rospy.logwarn('recognized object reference frame is not same as all objects reference frame')
                    object_pose.header.frame_id = detection.pose.header.frame_id
                if not object_pose.header.frame_id:
                    rospy.logerr('object recognition frame_id not set')
                    return None, None
                object_pose.pose.position.x =     detection.pose.pose.position.x
                object_pose.pose.position.y =     detection.pose.pose.position.y
                # set the collision model 5mm higher to avoid repeated collisions with the surface
                object_pose.pose.position.z =     detection.pose.pose.position.z + 0.005 + padding / 2.0
                object_pose.pose.orientation.x =  detection.pose.pose.orientation.x
                object_pose.pose.orientation.y =  detection.pose.pose.orientation.y
                object_pose.pose.orientation.z =  detection.pose.pose.orientation.z
                object_pose.pose.orientation.w =  detection.pose.pose.orientation.w
                return self.transform_pose(object_pose, self.robot.get_planning_frame()),\
                            [bounding_box_x, bounding_box_y, bounding_box_z]
        rospy.logerr(f'object to be picked ({object_name}) was not perceived, perceived objects were : {recognized_objects}')
        return None, None

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

    def move_hand_to_posture(self, hand_posture_name):
        '''
        use moveit commander to send the hand to a predefined arm configuration
        defined in srdf
        '''
        self.hand.set_named_target(hand_posture_name)
        self.hand.go()

    def print_moveit_error_helper(self, error_code, moveit_error_code, moveit_error_string):
        '''
        function that helps print_moveit_error method to have less code
        '''
        if error_code == moveit_error_code:
            rospy.logwarn(f'moveit says : {moveit_error_string}')

    def print_moveit_error(self, error_code):
        '''
        receive moveit result, compare with error codes, print what happened
        '''
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.PLANNING_FAILED, 'PLANNING_FAILED')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_MOTION_PLAN, 'INVALID_MOTION_PLAN')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE,\
                                                                                'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.CONTROL_FAILED, 'CONTROL_FAILED')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA, 'UNABLE_TO_AQUIRE_SENSOR_DATA')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.TIMED_OUT, 'TIMED_OUT')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.PREEMPTED, 'PREEMPTED')

        self.print_moveit_error_helper(error_code, MoveItErrorCodes.START_STATE_IN_COLLISION, 'START_STATE_IN_COLLISION')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS, 'START_STATE_VIOLATES_PATH_CONSTRAINTS')

        self.print_moveit_error_helper(error_code, MoveItErrorCodes.GOAL_IN_COLLISION, 'GOAL_IN_COLLISION')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS, 'GOAL_VIOLATES_PATH_CONSTRAINTS')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED, 'GOAL_CONSTRAINTS_VIOLATED')

        self.print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_GROUP_NAME, 'INVALID_GROUP_NAME')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS, 'INVALID_GOAL_CONSTRAINTS')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_ROBOT_STATE, 'INVALID_ROBOT_STATE')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_LINK_NAME, 'INVALID_LINK_NAME')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.INVALID_OBJECT_NAME, 'INVALID_OBJECT_NAME')

        self.print_moveit_error_helper(error_code, MoveItErrorCodes.FRAME_TRANSFORM_FAILURE, 'FRAME_TRANSFORM_FAILURE')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE, 'COLLISION_CHECKING_UNAVAILABLE')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.ROBOT_STATE_STALE, 'ROBOT_STATE_STALE')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.SENSOR_INFO_STALE, 'SENSOR_INFO_STALE')
        self.print_moveit_error_helper(error_code, MoveItErrorCodes.COMMUNICATION_FAILURE, 'COMMUNICATION_FAILURE')

        self.print_moveit_error_helper(error_code, MoveItErrorCodes.NO_IK_SOLUTION, 'NO_IK_SOLUTION')

    def detach_all_objects(self):
        '''
        usually self.hand.detach_object() should do the trick, however it doesn't work
        therefore we keep in memory the objects we grasped in the past and deattached them by their name explicitely
        '''
        if self.grasped_object != '':
            self.hand.detach_object(name=self.grasped_object)
            self.grasped_object = ''

    def pick_object(self, object_name, grasp_type):
        '''
        1) move arm to a position where the attached camera can see the scene (octomap will be populated)
        2) clear octomap
        3) add table and object to be grasped to planning scene
        4) generate a list of grasp configurations
        5) call pick moveit functionality
        '''
        rospy.loginfo(f'picking object : {object_name}')

        # open gripper
        rospy.loginfo('gripper will open now')
        self.move_hand_to_posture('open')

        # detach (all) object if any from the gripper
        self.detach_all_objects()

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
        if self.pregrasp_posture_required:
            self.move_arm_to_posture(self.pregrasp_posture)

        # ::::::::: setup planning scene
        rospy.loginfo('setup planning scene')

        # remove all objects from the planning scene if needed
        if self.clear_planning_scene:
            rospy.logwarn('Clearing planning scene')
            self.clean_scene()

        # add a list of custom boxes defined by the user to the planning scene
        for planning_scene_box in self.planning_scene_boxes:
            # add a box to the planning scene
            table_pose = PoseStamped()
            table_pose.header.frame_id = planning_scene_box['frame_id']
            box_x = planning_scene_box['box_x_dimension']
            box_y = planning_scene_box['box_y_dimension']
            box_z = planning_scene_box['box_z_dimension']
            table_pose.pose.position.x = planning_scene_box['box_position_x']
            table_pose.pose.position.y = planning_scene_box['box_position_y']
            table_pose.pose.position.z = planning_scene_box['box_position_z']
            table_pose.pose.orientation.x = planning_scene_box['box_orientation_x']
            table_pose.pose.orientation.y = planning_scene_box['box_orientation_y']
            table_pose.pose.orientation.z = planning_scene_box['box_orientation_z']
            table_pose.pose.orientation.w = planning_scene_box['box_orientation_w']
            self.scene.add_box(planning_scene_box['scene_name'], table_pose, (box_x, box_y, box_z))

        # add an object to be grasped, data comes from perception
        object_pose, bounding_box = self.make_object_pose(object_name, self.detections_msg)
        if not object_pose:
            return
        self.scene.add_box(object_name, object_pose, (bounding_box[0], bounding_box[1], bounding_box[2]))

        # print objects that were added to the planning scene
        rospy.loginfo(f'planning scene objects: {self.scene.get_known_object_names()}')

        # ::::::::: pick
        rospy.loginfo(f'picking object now')

        # generate a list of moveit grasp messages, poses are also published for visualisation purposes
        grasps = self.grasp_planner.make_grasps_msgs(object_name, object_pose, self.arm.get_end_effector_link(), grasp_type)

        # remove octomap, table and object are added manually to planning scene
        self.clear_octomap()

        # try to pick object with moveit
        result = self.robot.arm.pick(object_name, grasps)
        rospy.loginfo(f'moveit result code: {result}')
        # lower perception flag
        self.obj_recognition_received = False
        # handle moveit pick result
        if result == MoveItErrorCodes.SUCCESS:
            rospy.loginfo(f'Successfully picked object : {object_name}')
            self.grasped_object = object_name
            return String('e_success')
        else:
            rospy.logerr(f'grasp failed')
            self.print_moveit_error(result)
        return String('e_failure')

    def start_pick_node(self):
        # wait for trigger
        rospy.loginfo('ready to receive pick requests (publish object name on event_in topic)')
        rospy.spin()
        # shutdown moveit cpp interface before exit
        moveit_commander.roscpp_shutdown()

if __name__=='__main__':
    rospy.init_node('pick_object_node', anonymous=False)
    pick = PickTools()
    pick.start_pick_node()
