#!/usr/bin/env python3

import rospy
from grasp_planning_core import GraspPlanningCore
from pose_generator import PoseGenerator
from geometry_msgs.msg import PoseStamped

class SimpleGraspPlanner(GraspPlanningCore):
    '''
    Implement concrete methods out of GraspPlanningCore class
    A simple grasp planner:
    1) receive object pose
    2) replace it with a recorded "example" orientation
    3) sample around it in roll, pitch, yaw angles
    '''
    def __init__(self):
        super().__init__()
        self.pose_generator = PoseGenerator()

    def generate_poses(self, object_pose):
        '''
        receive object pose, generate multiple poses around it uniformly sampling in roll, pitch, yaw
        '''
        offset_vector = [0.0, 0.0, 0.0]
        pose_array_msg = self.pose_generator.spherical_sampling(object_pose, offset_vector)
        return pose_array_msg
