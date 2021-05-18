#!/usr/bin/env python3

import copy
import numpy as np
import rospy
import tf

import geometry_msgs.msg
import std_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray

class GripperVisualiser:
    '''
    Subscribe to geometry_msgs/PoseArray topic
    for each pose, draws a gripper in rviz using visualization_msgs/MarkerArray
    each pose is assumed to represent the gripper end effector frame
    '''
    def __init__(self):
        # parameters
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')
        self.marker_ns = rospy.get_param('~marker_namespace', 'grasp_poses')
        self.rgb_mesh_color = rospy.get_param('~rgb_mesh_color', [153.0, 102.0, 51.0]) # use 0.0 - 255.0 range
        self.alpha = rospy.get_param('~transparency', 0.5) # the transparency of the mesh, if 1.0 no transparency is set

        # publish marker array to rviz for visualising the gripper
        self.marker_array_pub = rospy.Publisher('gripper', MarkerArray, queue_size=1)
        # subscribe to pose array to place the gripper in multiple locations
        rospy.Subscriber('pose_array', PoseArray, self.poseArrayCB)
        rospy.sleep(0.5)
        self.listener = tf.TransformListener()
        self.tf_gripper_to_world = None
        self.mesh_count = 0
        rospy.loginfo('gripper visualisation node started')

    def get_tf_pose_array_wrt_global(self, frame_id):
        try:
            (trans, rot) = self.listener.lookupTransform(self.global_reference_frame, frame_id, rospy.Time(0))
            euler_rot = tf.transformations.euler_from_quaternion(rot)
            tf_gripper_to_world = tf.transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
            tf_gripper_to_world[0][3] = trans[0] # x
            tf_gripper_to_world[1][3] = trans[1] # y
            tf_gripper_to_world[2][3] = trans[2] # z
            self.tf_gripper_to_world = tf_gripper_to_world
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn(f'failed to find transform from {self.global_reference_frame} to {frame_id} , will retry')
        return False

    def detele_previous_markers(self):
        marker_array_msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.ns = self.marker_ns
        marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(marker)
        self.marker_array_pub.publish(marker_array_msg)
        rospy.sleep(0.2)

    def poseArrayCB(self, msg):
        self.detele_previous_markers()
        self.mesh_count = 0
        for i in range(5): # try 5 times to query tf
            if self.get_tf_pose_array_wrt_global(msg.header.frame_id):
                marker_array_msg = MarkerArray()
                for pose in msg.poses:
                    rot = []
                    rot.append(pose.orientation.x)
                    rot.append(pose.orientation.y)
                    rot.append(pose.orientation.z)
                    rot.append(pose.orientation.w)
                    euler_rot = tf.transformations.euler_from_quaternion(rot)
                    tf_pose_to_posearrayorigin = tf.transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
                    tf_pose_to_posearrayorigin[0][3] = pose.position.x # x
                    tf_pose_to_posearrayorigin[1][3] = pose.position.y # y
                    tf_pose_to_posearrayorigin[2][3] = pose.position.z # z
                    self.tf_pose_to_posearrayorigin = tf_pose_to_posearrayorigin
                    marker_array_msg = self.extend_marker_array_msg(marker_array_msg)
                self.marker_array_pub.publish(marker_array_msg)
                break
            else:
                rospy.sleep(0.3)

    def make_marker_msg(self, mesh_path, mesh_scale, position, orientation):
        marker = Marker()
        # marker.lifetime = rospy.Duration(3.0)
        marker.ns = self.marker_ns
        marker.header.frame_id = self.global_reference_frame
        marker.type = Marker.MESH_RESOURCE
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale = geometry_msgs.msg.Vector3(mesh_scale[0], mesh_scale[1], mesh_scale[2])
        marker.color = std_msgs.msg.ColorRGBA(\
            self.rgb_mesh_color[0]/255., self.rgb_mesh_color[1]/255., self.rgb_mesh_color[2]/255., self.alpha)
        marker.mesh_resource = mesh_path
        return marker

    def transform_part_to_gripper_ref_frame(self, t1, r1, mesh_translation, mesh_rotation):
        r1_euler = tf.transformations.euler_from_quaternion(r1)
        tf_mesh_to_gripper = tf.transformations.euler_matrix(r1_euler[0], r1_euler[1], r1_euler[2])
        tf_mesh_to_gripper[0][3] = t1[0] # x
        tf_mesh_to_gripper[1][3] = t1[1] # y
        tf_mesh_to_gripper[2][3] = t1[2] # z

        tf_part_to_mesh = tf.transformations.euler_matrix(mesh_rotation[0], mesh_rotation[1], mesh_rotation[2])
        tf_part_to_mesh[0][3] = mesh_translation[0] # x
        tf_part_to_mesh[1][3] = mesh_translation[1] # y
        tf_part_to_mesh[2][3] = mesh_translation[2] # z

        # transform once more to global reference frame
        new_m = np.dot(self.tf_gripper_to_world, (np.dot(self.tf_pose_to_posearrayorigin, np.dot(tf_mesh_to_gripper, tf_part_to_mesh))))
        n_roll, n_pitch, n_yaw = tf.transformations.euler_from_matrix(new_m)
        q_orientation = tf.transformations.quaternion_from_euler(n_roll, n_pitch, n_yaw)
        position = [new_m[0][3], new_m[1][3], new_m[2][3] ]
        return position, q_orientation

    def extend_marker_array_msg(self, marker_array_msg):
        link_dic = rospy.get_param('~gripper_transformations')
        for key in link_dic:
            for visual in link_dic[key]:
                tf_translation = link_dic[key][visual]['tf_translation']
                tf_rotation = link_dic[key][visual]['tf_rotation']
                mesh_translation = link_dic[key][visual]['mesh_translation']
                mesh_rotation = link_dic[key][visual]['mesh_rotation']
                mesh_path = link_dic[key][visual]['mesh_path']
                mesh_scale = link_dic[key][visual]['mesh_scale']
                position, q_orientation = self.transform_part_to_gripper_ref_frame(tf_translation, tf_rotation, mesh_translation, mesh_rotation)
                marker_msg = self.make_marker_msg(mesh_path, mesh_scale, position, q_orientation)
                marker_msg.id = self.mesh_count
                self.mesh_count += 1
                marker_array_msg.markers.append(copy.deepcopy(marker_msg))
        return marker_array_msg

if __name__=='__main__':
    rospy.init_node('gripper_visualiser_node', anonymous=False)
    gv = GripperVisualiser()
    rospy.spin()
