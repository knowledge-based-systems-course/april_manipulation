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
    Allows to place the gripper in multiple poses via rviz, using marker array msgs and robot mesh
    TODO: read from urdf model, currently only mia hand gripper is available
    '''
    def __init__(self):
        # parameters
        # the transparency of the mesh, if 1.0 no transparency is set
        self.alpha = 0.5
        self.global_reference_frame = 'world'
        self.marker_ns = 'grasp_poses'

        # publish marker array to rviz for visualising the gripper
        self.marker_array_pub = rospy.Publisher('gripper', MarkerArray, queue_size=1)
        # subscribe to pose array to place the gripper in multiple locations
        rospy.Subscriber('/pose_generator_node/poses', PoseArray, self.poseArrayCB)
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

    def make_marker_msg(self, mesh, position, orientation):
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
        marker.scale = geometry_msgs.msg.Vector3(1.0, 1.0, 1.0)
        marker.color = std_msgs.msg.ColorRGBA(0.0, 1.0, 0.0, self.alpha) # green
        marker.mesh_resource = 'package://mia_hand_description/meshes/stl/' + mesh
        return marker

    def transform_part_to_gripper_ref_frame(self, t1, r1, part_trans, part_rot):
        r1_euler = tf.transformations.euler_from_quaternion(r1)
        tf_mesh_to_gripper = tf.transformations.euler_matrix(r1_euler[0], r1_euler[1], r1_euler[2])
        tf_mesh_to_gripper[0][3] = t1[0] # x
        tf_mesh_to_gripper[1][3] = t1[1] # y
        tf_mesh_to_gripper[2][3] = t1[2] # z
        # tf_gripper_to_mesh = np.linalg.inv(tf_mesh_to_gripper) # no need for the inverse this time

        tf_part_to_mesh = tf.transformations.euler_matrix(part_rot[0], part_rot[1], part_rot[2])
        tf_part_to_mesh[0][3] = part_trans[0] # x
        tf_part_to_mesh[1][3] = part_trans[1] # y
        tf_part_to_mesh[2][3] = part_trans[2] # z
        # tf_mesh_to_part = np.linalg.inv(tf_part_to_mesh) # no need for the inverse this time

        # new_m = np.dot(tf_mesh_to_gripper, tf_part_to_mesh)
        # transform once more to global reference frame
        new_m = np.dot(self.tf_gripper_to_world, (np.dot(self.tf_pose_to_posearrayorigin, np.dot(tf_mesh_to_gripper, tf_part_to_mesh))))
        n_roll, n_pitch, n_yaw = tf.transformations.euler_from_matrix(new_m)
        q_orientation = tf.transformations.quaternion_from_euler(n_roll, n_pitch, n_yaw)
        position = [new_m[0][3], new_m[1][3], new_m[2][3] ]
        return position, q_orientation

    def extend_marker_array_msg(self, marker_array_msg):

        # THUMB
        # rosrun tf tf_echo hand_ee_link MCP1
        # MCP1 thumb.stl (thumb)
        t1 = [0.027, -0.031, -0.051]
        r1 = [0.426, -0.138, -0.385, 0.807]
        part_trans = [0.01557, 0.03431, -0.00021]
        part_rot = [1.18797, 0.58341, 0.64822]
        position, q_orientation = self.transform_part_to_gripper_ref_frame(t1, r1, part_trans, part_rot)
        marker_msg = self.make_marker_msg('thumb.stl', position, q_orientation)
        marker_msg.id = self.mesh_count
        self.mesh_count += 1
        marker_array_msg.markers.append(copy.deepcopy(marker_msg))

        # RING
        # rosrun tf tf_echo hand_ee_link MCP4
        # MCP4 ring.stl (ring)
        t1 = [-0.012, 0.018, -0.054]
        r1 = [0.076, 0.679, 0.197, 0.703]
        part_trans = [-0.00188, 0.04482, 0.0001]
        part_rot = [0, -1.74533, 1.5708]
        position, q_orientation = self.transform_part_to_gripper_ref_frame(t1, r1, part_trans, part_rot)
        marker_msg = self.make_marker_msg('ring.stl', position, q_orientation)
        marker_msg.id = self.mesh_count
        self.mesh_count += 1
        marker_array_msg.markers.append(copy.deepcopy(marker_msg))

        # LITTLE
        # rosrun tf tf_echo hand_ee_link MCP5
        # MCP5 little.stl (little)
        t1 = [-0.028, 0.013, -0.054]
        r1 = [0.049, 0.648, 0.287, 0.704]
        part_trans = [-0.00182, 0.03674, -0.00019]
        part_rot = [-0.08866, -1.91864, 1.56133]
        position, q_orientation = self.transform_part_to_gripper_ref_frame(t1, r1, part_trans, part_rot)
        marker_msg = self.make_marker_msg('little.stl', position, q_orientation)
        marker_msg.id = self.mesh_count
        self.mesh_count += 1
        marker_array_msg.markers.append(copy.deepcopy(marker_msg))

        # INDEX
        # rosrun tf tf_echo hand_ee_link index_finger_sensor
        # index_finger_sensor index.stl (index)
        t1 = [0.031, 0.039, -0.043]
        r1 = [0.205, 0.672, 0.072, 0.708]
        # taken from urdf
        part_trans = [-0.0032, 0.02824, 0.00022]
        part_rot = [0.45906, -1.37586, 1.10409]
        position, q_orientation = self.transform_part_to_gripper_ref_frame(t1, r1, part_trans, part_rot)
        marker_msg = self.make_marker_msg('index.stl', position, q_orientation)
        marker_msg.id = self.mesh_count
        self.mesh_count += 1
        marker_array_msg.markers.append(copy.deepcopy(marker_msg))

        # MIDDLE
        # rosrun tf tf_echo hand_ee_link middle_finger_sensor
        # middle_finger_sensor middle.stl (middle)
        t1 = [0.005, 0.043, -0.044]
        r1 = [0.137, 0.694, 0.137, 0.694]
        part_trans = [-0.00319, 0.02824, 0.00022]
        part_rot = [1.5708, -1.5708, 0]
        position, q_orientation = self.transform_part_to_gripper_ref_frame(t1, r1, part_trans, part_rot)
        marker_msg = self.make_marker_msg('middle.stl', position, q_orientation)
        marker_msg.id = self.mesh_count
        self.mesh_count += 1
        marker_array_msg.markers.append(copy.deepcopy(marker_msg))

        # PALM
        # rosrun tf tf_echo hand_ee_link wrist
        # wrist palm_simple.stl (palm)
        t1 = [0.000, -0.085, -0.103]
        r1 = [0.319, -0.000, -0.000, 0.948]
        # taken from urdf
        part_trans = [0.0038, 0.06597, -0.01205]
        part_rot = [-0.2618, 0, 0]
        # apply transform
        position, q_orientation = self.transform_part_to_gripper_ref_frame(t1, r1, part_trans, part_rot)
        marker_msg = self.make_marker_msg('palm_simple.stl', position, q_orientation)
        marker_msg.id = self.mesh_count
        self.mesh_count += 1
        # scale down, taken from urdf
        marker_msg.scale = geometry_msgs.msg.Vector3(0.12203, 0.12203, 0.12203)
        marker_array_msg.markers.append(copy.deepcopy(marker_msg))

        return marker_array_msg

if __name__=='__main__':
    rospy.init_node('gripper_visualiser_node', anonymous=False)
    gv = GripperVisualiser()
    rospy.spin()
