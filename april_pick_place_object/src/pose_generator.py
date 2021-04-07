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

    def spherical_sampling(self, original_pose, offset_vector):
        # offset to object tf
        tf_offset_to_object = tf.transformations.euler_matrix(0, 0, 0)
        tf_offset_to_object[0][3] = offset_vector[0] # x
        tf_offset_to_object[1][3] = offset_vector[1] # y
        tf_offset_to_object[2][3] = offset_vector[2] # z

        # object to world tf
        rot = []
        rot.append(original_pose.pose.orientation.x)
        rot.append(original_pose.pose.orientation.y)
        rot.append(original_pose.pose.orientation.z)
        rot.append(original_pose.pose.orientation.w)
        euler_rot = tf.transformations.euler_from_quaternion(rot)
        tf_object_to_world = tf.transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
        tf_object_to_world[0][3] = original_pose.pose.position.x # x
        tf_object_to_world[1][3] = original_pose.pose.position.y # y
        tf_object_to_world[2][3] = original_pose.pose.position.z # z

        # apply rotations to identity matrix
        # prepare pose array msg
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = original_pose.header.frame_id
        pose_array_msg.header.stamp = rospy.Time.now()
        tf_pose = Pose()
        # sample multiple rotations
        for roll in self.generate_angles(start=self.roll_start, end=self.roll_end, step=self.step):
            for pitch in self.generate_angles(start=self.pitch_start, end=self.pitch_end, step=self.step):
                for yaw in self.generate_angles(start=self.yaw_start, end=self.yaw_end, step=self.step):
                    nm = tf.transformations.euler_matrix(roll, pitch, yaw) # rpy

                    # transform back to world reference frame
                    processed_pose_in_world_rf_m = np.dot(tf_object_to_world, np.dot(tf_offset_to_object, nm))
                    proll, ppitch, pyaw = tf.transformations.euler_from_matrix(processed_pose_in_world_rf_m)
                    position = (processed_pose_in_world_rf_m[0][3], processed_pose_in_world_rf_m[1][3], processed_pose_in_world_rf_m[2][3])
                    q_orientation = tf.transformations.quaternion_from_euler(proll, ppitch, pyaw)

                    # pack elements into pose
                    tf_pose.position.x = position[0]
                    tf_pose.position.y = position[1]
                    tf_pose.position.z = position[2]
                    tf_pose.orientation.x = q_orientation[0]
                    tf_pose.orientation.y = q_orientation[1]
                    tf_pose.orientation.z = q_orientation[2]
                    tf_pose.orientation.w = q_orientation[3]

                    # append to pose array msg
                    pose_array_msg.poses.append(copy.deepcopy(tf_pose))
        return pose_array_msg

    def publish_pose_array_msg(self, pose_array_msg):
        self.pose_array_pub.publish(pose_array_msg)

if __name__=='__main__':
    # example of how to use this node, in practice it is used as a library
    rospy.init_node('pose_generator_node', anonymous=False)
    pose_generator = PoseGenerator()
    # make a random pose
    original_pose = PoseStamped()
    # ----
    original_pose.header.frame_id = 'world'
    some_position = [1.0, 0, 0]
    some_orientation = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    offset_vector = [0, 1.0, 0]
    # ----
    original_pose.pose.position.x = some_position[0]
    original_pose.pose.position.y = some_position[1]
    original_pose.pose.position.z = some_position[2]
    original_pose.pose.orientation.x = some_orientation[0]
    original_pose.pose.orientation.y = some_orientation[1]
    original_pose.pose.orientation.z = some_orientation[2]
    original_pose.pose.orientation.w = some_orientation[3]
    pose_array_msg = pose_generator.spherical_sampling(original_pose, offset_vector)
    pose_generator.publish_pose_array_msg(pose_array_msg)
