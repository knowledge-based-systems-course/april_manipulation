#!/usr/bin/env python3

import rospy

from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GraspPlanningCore:
    '''
    Abstract class that defines classes to interact with a grasp planning module
    It cannot be used by itself, you need to inherit from it and implement virtual methods
    '''
    def __init__(self):

        # parameters
        self.gripper_joint_names = rospy.get_param('~gripper_joint_names', None)
        gripper_close = rospy.get_param('~gripper_close', None)
        gripper_open = rospy.get_param('~gripper_open', None)
        self.gripper_joint_efforts = rospy.get_param('~gripper_joint_efforts', None)
        self.grasp_quality = rospy.get_param('~grasp_quality', None)
        self.object_padding = rospy.get_param('~object_padding', None)
        self.max_contact_force = rospy.get_param('~max_contact_force', None)
        # pregrasp parameters
        self.pre_grasp_approach_min_dist = rospy.get_param('~pre_grasp_approach/min_dist', None)
        self.pre_grasp_approach_desired = rospy.get_param('~pre_grasp_approach/desired', None)
        self.pre_grasp_approach_axis = rospy.get_param('~pre_grasp_approach/axis', None)
        # post grasp retreat parameters
        self.post_grasp_retreat_frame_id = rospy.get_param('~post_grasp_retreat/frame_id', None)
        self.post_grasp_retreat_min_dist = rospy.get_param('~post_grasp_retreat/min_dist', None)
        self.post_grasp_retreat_desired = rospy.get_param('~post_grasp_retreat/desired', None)
        self.post_grasp_retreat_axis = rospy.get_param('~post_grasp_retreat/axis', None)
        # grasp type
        self.grasp_orientations = {}
        self.grasp_orientations['side_grasp'] = rospy.get_param('~side_grasp_orientation', None)
        self.grasp_orientations['top_grasp'] = rospy.get_param('~top_grasp_orientation', None)

        # parameter sanity check
        if not gripper_close or not gripper_open:
            raise ValueError('mandatory parameters gripper_close or/and gripper_open not set')

        if not self.gripper_joint_names or not self.gripper_joint_efforts:
            raise ValueError('mandatory parameter gripper_joint_names or/and gripper_joint_efforts not set')

        self.gripper_open_trajectory = self.make_gripper_trajectory(gripper_open)
        self.gripper_close_trajectory = self.make_gripper_trajectory(gripper_close)

    def make_gripper_trajectory(self, joint_angles):
        '''
        Set and return the gripper posture as a trajectory_msgs/JointTrajectory
        only one point is set which is the final gripper target
        '''
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joint_names
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = joint_angles
        trajectory_point.effort = self.gripper_joint_efforts
        trajectory_point.time_from_start = rospy.Duration(1.0)
        trajectory.points.append(trajectory_point)
        return trajectory

    def get_object_padding(self):
        return self.object_padding

    def make_grasps_msgs(self, object_name, grasp_pose, end_effector_frame):
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
        g.pre_grasp_approach = self.make_gripper_translation_msg(end_effector_frame,
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
        g.max_contact_force = self.max_contact_force

        # an optional list of obstacles that we have semantic information about
        # and that can be touched/pushed/moved in the course of grasping
        g.allowed_touch_objects = [object_name]

        # A name for this grasp
        g.id = 'top_grasp'

        # NOTE : one could change the orientation and generate more grasps, currently the list has only 1 grasp

        return [g]

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

    def compute_grasp_orientations(self, grasp_type):
        return self.grasp_orientations[grasp_type]
