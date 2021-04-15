#!/usr/bin/python3

import rospy
import tf

from gazebo_msgs.msg import LinkStates

'''
subscribe to /gazebo/link_states
from parameter take a desired link to track
publish link pose as tf
'''

class LinkTFgtPublisher:
    def __init__(self):
        # parameters
        self.prefix = rospy.get_param('~prefix', 'gazebo_ros_vel/mia_hand::') # robot_name::
        self.link_name = rospy.get_param('~link_name', 'wrist') # the name of the link you want to publish tf
        # subscribe to gazebo model states (gives poses of all existing objects in the simulation
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.LinkStatesCB)
        self.rate = rospy.Rate(30)
        self.link_state_msg_received = False
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.loginfo('link tf ground truth node started')

    def LinkStatesCB(self, msg):
        self.link_state_msg_received = True
        self.link_state_msg = msg

    def publishTF(self):
        # get the index of the array where the desired link can be found
        index = None
        for i, link_name in enumerate(self.link_state_msg.name):
            if link_name == self.prefix + self.link_name:
                index = i
        if not index:
            rospy.logerr('link not found in /gazebo/link_states')
            return
        # get link pose
        link_pose = self.link_state_msg.pose[index]
        # broadcast object tf
        self.tf_broadcaster.sendTransform((link_pose.position.x, link_pose.position.y, link_pose.position.z),\
        (link_pose.orientation.x, link_pose.orientation.y, link_pose.orientation.z, link_pose.orientation.w),\
        rospy.Time.now(), self.link_name, 'world')

    def start_link_tf_ft_pub(self):
        while not rospy.is_shutdown():
            if self.link_state_msg_received:
                # lower flag
                self.link_state_msg_received = False
                self.publishTF()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('link_tf_gt_pub', anonymous=False)
    link_tf_gt_pub = LinkTFgtPublisher()
    link_tf_gt_pub.start_link_tf_ft_pub()
