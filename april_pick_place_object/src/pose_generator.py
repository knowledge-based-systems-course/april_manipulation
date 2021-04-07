#!/usr/bin/env python3

import copy
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseArray, Pose, PoseStamped

class PoseGenerator():
    '''
    Receive a pose and generate multiple ones with different orientations
    '''
    def __init__(self):

        # parameters
        self.step = rospy.get_param('~step', 0.2)
        self.roll_start = rospy.get_param('~roll_start', -0.3) # ~15 degree
        self.roll_end = rospy.get_param('~roll_end', 0.3) # ~15 degree
        self.pitch_start = rospy.get_param('~pitch_start', -0.3) # ~15 degree
        self.pitch_end = rospy.get_param('~pitch_end', 0.3) # ~15 degree
        self.yaw_start = rospy.get_param('~yaw_start', -0.3) # ~15 degree
        self.yaw_end = rospy.get_param('~yaw_end', 0.3) # ~15 degree

        self.pose_array_pub = rospy.Publisher('~poses', PoseArray, queue_size=50)
        # give some time for pose generator to register in the ros network
        rospy.sleep(0.5)

    def generate_angles(self, start=0.0, end=2.0, step=0.1):
        my_list = []
        if abs(start - end) <= 0.000001: # if start == end
            return [start]
        assert(start < end)
        temp_number = start
        my_list.append(start)
        while temp_number < end:
            temp_number += step
            my_list.append(temp_number)
        # remove last element (is slightly greater than the end value)
        my_list.pop()
        my_list.append(end)
        return my_list

    def generate_poses(self, original_pose, offset_vector):
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = original_pose.header.frame_id
        pose_array_msg.header.stamp = rospy.Time.now()

        pose_msg = Pose()

        quaternion = (original_pose.pose.orientation.x, original_pose.pose.orientation.y,\
                      original_pose.pose.orientation.z, original_pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        original_roll = euler[0]
        original_pitch = euler[1]
        original_yaw = euler[2]

        tf_world_to_obj = tf.transformations.euler_matrix(euler[0], euler[1], euler[2])
        tf_obj_to_world = np.linalg.inv(tf_world_to_obj)

        # tranform offset from object reference frame to world reference frame
        tf_offset_vector = np.dot(tf_obj_to_world, np.array([offset_vector[0], offset_vector[1], offset_vector[2], 1.0]))

        pose_msg.position.x = original_pose.pose.position.x + tf_offset_vector[0]
        pose_msg.position.y = original_pose.pose.position.y + tf_offset_vector[1]
        pose_msg.position.z = original_pose.pose.position.z + tf_offset_vector[2]

        # get rotation matrix
        rot_matrix = tf.transformations.euler_matrix(original_roll, original_pitch, original_yaw)

        for roll in self.generate_angles(start=self.roll_start, end=self.roll_end, step=self.step):
            for pitch in self.generate_angles(start=self.pitch_start, end=self.pitch_end, step=self.step):
                for yaw in self.generate_angles(start=self.yaw_start, end=self.yaw_end, step=self.step):
                    m = tf.transformations.euler_matrix(roll, pitch, yaw)
                    new_m = np.dot(tf_obj_to_world, m)
                    # transform new_m to quaternion
                    n_roll, n_pitch, n_yaw = tf.transformations.euler_from_matrix(new_m)
                    quaternion = tf.transformations.quaternion_from_euler(n_roll, n_pitch, n_yaw)
                    pose_msg.orientation.x = quaternion[0]
                    pose_msg.orientation.y = quaternion[1]
                    pose_msg.orientation.z = quaternion[2]
                    pose_msg.orientation.w = quaternion[3]
                    pose_array_msg.poses.append(copy.deepcopy(pose_msg))
        return pose_array_msg

    def publish_pose_array_msg(self, pose_array_msg):
        self.pose_array_pub.publish(pose_array_msg)

if __name__=='__main__':
    rospy.init_node('pose_generator_node', anonymous=False)
    pose_generator = PoseGenerator()
    # make a random pose
    original_pose = PoseStamped()
    # ----
    original_pose.header.frame_id = 'world'
    some_position = [1.0, 1.0, 0]
    some_orientation = tf.transformations.quaternion_from_euler(0.785398, 0.0, 0.0)
    offset_vector = [0.0, 0.0, 1.0]
    # ----
    original_pose.pose.position.x = some_position[0]
    original_pose.pose.position.y = some_position[1]
    original_pose.pose.position.z = some_position[2]
    original_pose.pose.orientation.x = some_orientation[0]
    original_pose.pose.orientation.y = some_orientation[1]
    original_pose.pose.orientation.z = some_orientation[2]
    original_pose.pose.orientation.w = some_orientation[3]
    pose_array_msg = pose_generator.generate_poses(original_pose, offset_vector)
    pose_generator.publish_pose_array_msg(pose_array_msg)
