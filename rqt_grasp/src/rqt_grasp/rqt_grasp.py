#!/usr/bin/env python3

import os
import rospy
import rospkg
import rosnode
import actionlib
import tf
import random
import math
import std_srvs.srv
import std_msgs.msg
import dynamic_reconfigure.client

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QAbstractButton, QListWidgetItem
from python_qt_binding.QtCore import Signal, QTimer, QThread

# from robot_action_msgs.msg import PickObjectAction, PickObjectGoal

from std_msgs.msg import String
# from high_level_robot_api import  robot_class
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import PoseStamped, Twist

class publishObjPoseThread(QThread):

    def __init__(self):
        QThread.__init__(self)
        self.obj_pose = None
        self.pub_obj_selector_obj_pose_mockup = rospy.Publisher('/mcr_perception/object_selector/output/object_pose', PoseStamped, queue_size=1)

                                                       
    def __del__(self):
        self.wait()

    def set_pose(self, pose):
        self.obj_pose = pose
        

    def set_ms_sleep_time(self, ms_to_sleep):
        self.sleep_ = ms_to_sleep

    def run(self):
        while 1:
            if self.obj_pose:
                rospy.logdebug('publishing mockup obj pose')
                # update pose stamp
                self.obj_pose.header.stamp = rospy.Time.now()
                # publish obj pose
                self.pub_obj_selector_obj_pose_mockup.publish(self.obj_pose)
            else:
                rospy.logerr('mockup thread: obj pose not set !')
            # sleep to control frequency
            self.msleep(self.sleep_)


class RqtGrasp(Plugin):

    def __init__(self, context):
        super(RqtGrasp, self).__init__(context)
        rospy.loginfo('Initializing grasp rqt, have a happy grasping !')


        self.vel_base = Twist()
        self.listener2 = tf.TransformListener() #creating tf.TransformListener object
        self.goalpose_msg = PoseStamped() #to store goal pose
        self.goalpose_msg_odom = PoseStamped()
        self.base_to_odom = None
        self.wait_for_transform = 0.1

        self.pub_base_vel_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Give QObjects reasonable names
        self.setObjectName('RqtGrasp')
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file (xml description of the gui window created with qtcreator)
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_grasp'), 'common/resources', 'rqtgrasp.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('rqtgrasp.ui')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        rospy.loginfo('start widget connections...')

        ## make a connection between the qt objects and this class methods

        ## head pan
        #self._widget.head_rotation_slider.valueChanged[int].connect(self.handle_head_cam_tilt_slider)
        #self._widget.boxHeadPan_neck_angle.valueChanged[int].connect(self.handle_head_rotation_box)

        ## head cam tilt
        #self._widget.cmdHeadCamTilt_look_to_obj.clicked.connect(self.handle_head_cam_look_to_obj)
        #self._widget.head_camera_tilt_slider.valueChanged[int].connect(self.handle_camera_tilt_slider)
        #self._widget.boxHeadCamTilt_cam_angle.valueChanged[int].connect(self.head_cam_tilt_box)

        ## gripper
        #self._widget.cmdGripper_open.clicked.connect(self.handle_open_gripper)
        #self._widget.cmdGripper_close.clicked.connect(self.handle_close_gripper)

        ## (switch) arm controller and gazebo (only needed for simulation)

        ## find out from env if simulated robot is being used
        #self.sim_robot = None
        ##if os.getenv('ROBOT' == '05-with-arm-sim'):
            ## self.sim_robot is a member variable flag very useful later in the code to disable sim related actions
        #self.sim_robot = True
        #rospy.loginfo('Simulated robot detected (ROBOT env equal to 05-with-arm-sim)')

        #self._widget.cmdGazebo_reset_world.clicked.connect(self.handle_gazebo_reset_world)
        self._widget.cmdGazebo_randomize_obj.clicked.connect(self.handle_gazebo_randomize_obj)
        self._widget.cmdGazebo_reset_obj.clicked.connect(self.handle_gazebo_reset_obj_pose)

        ## (switch) arm controller connections
        #self._widget.cmdArmController_switch_position.clicked.connect(self.handle_switch_arm_to_position)
        #self._widget.cmdArmController_switch_trajectory.clicked.connect(self.handle_switch_arm_to_trajectory)
        ## else:
        ##     rospy.loginfo('Real robot detected (env ROBOT is different than 05-with-arm-sim), switch arm ctrl + Gazebo functionality will be disabled!')
        ##     self.sim_robot = False
        ##     self._widget.fraArmController.setEnabled(False)
        ##     self._widget.fraGazebo.setEnabled(False)


        ## Scripts
        self._widget.cmdScripts_grasp_obj.clicked.connect(self.handle_scripts_grasp_obj)
        #self._widget.cmdScripts_grasp_5_obj.clicked.connect(self.handle_scripts_grasp_5_obj)

        ## Pick obj client
        #self._widget.cmdPickObjClient_run.clicked.connect(self.handle_run_pick_action_client)
        #self._widget.cmdPickObjClient_cancel.clicked.connect(self.handle_cancel_pick_action_client)

        ## Perception
        #self._widget.cmdPerception_recognize_obj.clicked.connect(self.handle_perception_recognize_obj)
        #self._widget.chkPerceptionMockupObjPose.stateChanged.connect(self.handle_perception_mockup_obj)

        ## Pregrasp planner
        #self._widget.cmdPregraspPlanner_start.clicked.connect(self.handle_start_pregrasp_planner)

        ## Moveit
        #self._widget.cmdMoveit_out_of_view.clicked.connect(self.handle_moveit_goto_out_of_view)
        #self._widget.cmdMoveit_pregrasp.clicked.connect(self.handle_moveit_goto_pregrasp)
        #self._widget.cmdMoveit_eg_obj.clicked.connect(self.handle_moveit_goto_eg_obj)
        #self._widget.cmdMoveit_kill_arm.clicked.connect(self.handle_moveit_kill_arm)
        #self._widget.cmdMoveit_calib.clicked.connect(self.handle_moveit_goto_calib)
        #self._widget.cmdMoveit_candle.clicked.connect(self.handle_moveit_goto_candle)

        #rospy.loginfo('Done creating widget connections...')

        #####################################################################################################################


        ## INIT ROBOT COMPONENTS ----

        ## create  robot class object ( high level functionality python wrapper)
        ##try:
            ### hri to rotate the  head, percetion to recognize objects, manipulation to send arm to poses
            ##self. = robot_class.Robot(enabled_components=['hri', 'perception', 'misc', 'manipulation'])
        ##except Exception as e:
            ##rospy.logfatal("Couldn't create the robot_class object. Exception: {}".format(e))

        ## ARM CTRL ---------------

        ## to keep track of the arm ctrl status and avoid switching controllers all the time
        #self.arm_ctrl_memory = 'position'  #ou trajectory

        ## prepare arm ctrl switch service
        #self.switch_arm_srv = rospy.ServiceProxy ('/controller_manager/switch_controller', SwitchController)

        ## You call a service by creating a rospy.ServiceProxy with the name of the service you wish to call.
        ## O SwitchController e a service_class (e o tipo de service) e foi importado do controller_manager_msgs.srv 
        ## You often will want to call rospy.wait_for_service() to block until a service is available. 
        ## Neste caso however, nao usaram o wait_for_service

        ## prepare srv request msg to switch from position to trajectory and viceversa
        #self.switch_ctrl_request_position = SwitchControllerRequest()   
        #self.switch_ctrl_request_trajectory = SwitchControllerRequest() 

        ##estes Requests tmb foram importados do .srv e sao outra service_class (ainda ha as Responses)


        ####################################################################################################################

        ## create position controller (srv) list
        #position_ctrl_list = []
        #for i in range(0,7):
            #position_ctrl_list.append('left_arm_joint' + str(i) + '_position_controller')
        ## to stop position ctrl, start trajectory one
        #self.switch_ctrl_request_position.start_controllers = position_ctrl_list
        #self.switch_ctrl_request_position.stop_controllers = ['left_arm_traj_controller']
        #self.switch_ctrl_request_position.strictness = 1     # strict 2, best effort 1
        ## to start trajectory ctrl, stop position one
        #self.switch_ctrl_request_trajectory.start_controllers = ['left_arm_traj_controller']
        #self.switch_ctrl_request_trajectory.stop_controllers = position_ctrl_list
        #self.switch_ctrl_request_trajectory.strictness = 1   # strict 2, best effort 1

        ## PERCEPTION --------------

        ## NOTE: a bit of background on the object selector:
        ## its a component with memory that listens to the output of the perception pipeline (object list)
        ## and remembers all the last batch of recognized objects and its associated 3D poses
        ## then later on you can query a particular object pose by doing some topic publishing and listening

        ## to keep track of output of perception internally in a class variable for sanity checks purposes
        #self.object_list = None

        ## to keep a count of the number of times the user has triggered perception
        #self.perception_count = 0

        ## get min confidence (threshold) from param server and write value to label
        #self._widget.txtPerception_confidence.setPlainText(str(rospy.get_param('~perception_confidence', 0.0)))

        ## object selector communication, to (later) publish obj pose mockup
        ## to trigger object selector
        #self.pub_obj_selector_trigger = rospy.Publisher('/mcr_perception/object_selector/event_in', String, queue_size=1)
        ## to publish desired object name to object selector
        #self.pub_obj_selector_obj_name = rospy.Publisher('/mcr_perception/object_selector/input/object_name', String, queue_size=1)

        ## subscribe, to get object pose from object selector
        #rospy.Subscriber('/mcr_perception/object_selector/output/object_pose', PoseStamped, self.objSelectorPoseOutCB, queue_size=1)
        ## subscribe, to get object selector response (e_successm e_failure)
        #rospy.Subscriber('/mcr_perception/object_selector/event_out', String, self.objSelectorCallback, queue_size=1)
        ##subscribe to the mockup pose GUI 
        #rospy.Subscriber('/mir_manipulation/pregrasp_planner_pipeline/target_pose', PoseStamped, self.objMockUpGUI, queue_size=1)
        

        ## prepare obj mockup pose thread (publisher)
        #self.mockup_pub_thread = publishObjPoseThread()
        #self.mockup_pub_thread.set_ms_sleep_time(200) # 200 ms = 5 hz
        ## flag to indicate if mockup needs to be published
        #self.mockup_obj = False

        ## to store the pose received in the object selector callback, but init values from param server
        #self.obj_pose = PoseStamped()

        #self.pose_gui = PoseStamped()

        #self.obj_pose.pose.position.x = rospy.get_param('~mockup_pose_position_x', 0.591151914331)
        #self.obj_pose.pose.position.y = rospy.get_param('~mockup_pose_position_y', 0.134625228351)
        #self.obj_pose.pose.position.z = rospy.get_param('~mockup_pose_position_z', 0.534574985504)
        #self.obj_pose.pose.orientation.x = rospy.get_param('~mockup_orientation_x', 0.701755623502)
        #self.obj_pose.pose.orientation.y = rospy.get_param('~mockup_orientation_y', 0.086920802947)
        #self.obj_pose.pose.orientation.z = rospy.get_param('~mockup_orientation_z', 0.701731505718)
        #self.obj_pose.pose.orientation.w = rospy.get_param('~mockup_orientation_w', -0.0869293551171)
        #self.obj_pose.header.frame_id = 'base_link'

        ## MOVEIT PICK

        # rostopic pub /pick_object_node/event_in std_msgs/String "data: cylinder" --once'
        self.pregrasp_planner_pub = rospy.Publisher('/pick_object_node/event_in', String, queue_size=1)

        ## PREGRASP PLANNER --------

        ## create publisher and subscriber to trigger pregrasp planner
        #self.pregrasp_planner_pub = rospy.Publisher('/mir_manipulation/pregrasp_planner_pipeline/event_in', String, queue_size=1)
        #rospy.Subscriber("/mir_manipulation/pregrasp_planner_pipeline/event_out", String, self.ppCallback, queue_size=1)

        #self.motion_planner_pub = rospy.Publisher('/mcr_arm_motions/move_arm_planned_motion/event_in', String, queue_size=1)
        #self.planning_scene_pub = rospy.Publisher('/_moveit_scene_node/event_in', String, queue_size=1)
        ## DYNAMIC reconfigure to alter pp params

        ## reload config file of dynamic reconfigure to test stable params without relaunching it
        #self.pub_dynamic_reconf = rospy.Publisher('/mcr_common/dynamic_reconfigure_client/reload_config', std_msgs.msg.Empty, queue_size=1)

        ## the name of the pregrasp planner node such that we can call dynamic reconfigure on it
        #self.pregrasp_planner_node_name = rospy.get_param('~pregrasp_planner_node_name', '/mir_manipulation/pregrasp_planner_pipeline/pregrasp_planner')

        ## disable pregrasp planner frame at startup (need mockup pose to work)
        #self._widget.fraPregraspPlanner.setEnabled(False)


        ##### GAZEBO -----------------

        ## find out from env if simulated robot is being used
        ##if os.getenv('ROBOT' == '05-with-arm-sim'):
            ## prepare gazebo service to reset world

        #self.gazebo_reset_srv = rospy.ServiceProxy ('/gazebo/reset_world', std_srvs.srv.Empty)

        # prepare gazebo service to set obj pose
        self.gazebo_set_obj_pose_srv = rospy.ServiceProxy ('/gazebo/set_model_state', SetModelState)

        # configure obj pose request
        self.obj_pose_req = SetModelStateRequest()
        # object name and pose in x, y, z, roll, pitch, yaw
        self.graspable_obj = rospy.get_param('~object', ['cylinder', 0, 0.7, 1.15, 0, 0, 0])
        # get obj quaternion from roll, pitch, yaw
        quaternion = tf.transformations.quaternion_from_euler(self.graspable_obj[4], self.graspable_obj[5], self.graspable_obj[6])
        # get obj name and pose from param server, if not found then set example pose by default
        self.obj_pose_req.model_state.model_name = rospy.get_param('~obj_name', self.graspable_obj[0])
        self.obj_pose_req.model_state.pose.position.x = rospy.get_param('~obj_pose_position_x', self.graspable_obj[1])
        self.obj_pose_req.model_state.pose.position.y = rospy.get_param('~obj_pose_position_y', self.graspable_obj[2])
        self.obj_pose_req.model_state.pose.position.z = rospy.get_param('~obj_pose_position_z', self.graspable_obj[3])
        self.obj_pose_req.model_state.pose.orientation.x = rospy.get_param('~obj_pose_orientation_x', quaternion[0])
        self.obj_pose_req.model_state.pose.orientation.y = rospy.get_param('~obj_pose_orientation_y', quaternion[1])
        self.obj_pose_req.model_state.pose.orientation.z = rospy.get_param('~obj_pose_orientation_z', quaternion[2])
        self.obj_pose_req.model_state.pose.orientation.w = rospy.get_param('~obj_pose_orientation_w', quaternion[3])
        self.obj_pose_req.model_state.reference_frame = "" # leave it empty, it works...

        # # for random obj pose : get object bounds from param server, obj will be randomized between those limits
        # self.obj_bounds_x_min = rospy.get_param('~object_bounds/x_min', 1.0)
        # self.obj_bounds_x_max = rospy.get_param('~object_bounds/x_max', 1.15)
        # self.obj_bounds_y_min = rospy.get_param('~object_bounds/y_min', 0.45)
        # self.obj_bounds_y_max = rospy.get_param('~object_bounds/y_max', 0.7)
        # self.obj_bounds_z_min = rospy.get_param('~object_bounds/z_min', 0.53)
        # self.obj_bounds_z_max = rospy.get_param('~object_bounds/z_max', 0.53)

        # # set readed params to texboxs
        # self._widget.txtGazebo_x_min.setText(self.obj_bounds_x_min)
        # self._widget.txtGazebo_x_min.setText(self.obj_bounds_x_max)
        # self._widget.txtGazebo_y_min.setText(self.obj_bounds_y_min)
        # self._widget.txtGazebo_y_min.setText(self.obj_bounds_y_max)
        # self._widget.txtGazebo_z_min.setText(self.obj_bounds_z_min)
        # self._widget.txtGazebo_z_min.setText(self.obj_bounds_z_max)

        context.add_widget(self._widget)
        rospy.loginfo('init finished')

    # ::::::::::::::  class methods

    def handle_open_gripper(self):
        '''
        open gripper with the help of  robot class
        '''
        rospy.loginfo('Open gripper not implemented')
        # self.robot.manipulation.open_gripper()

    def handle_close_gripper(self):
        '''
        open gripper with the help of  robot class
        '''
        rospy.loginfo('Closing gripper not implemented')
        #self.robot.manipulation.close_gripper()

    def handle_run_pick_action_client(self):
        '''
        button that runs an action lib client to call action lib pick_object_server server
        '''
        # disable button to prevent user to press twice during the pick
        self._widget.cmdPickObjClient_run.setEnabled(False)

        # set status label to busy
        self._widget.lblPickObjClient_status.setText("Busy trying to grasp... !")

        # reload pregrasp config from yaml (e.g. _actions/ros/config/_action_servers_params.yaml)
        self.pub_dynamic_reconf.publish(std_msgs.msg.Empty())

        rospy.loginfo('Calling pick object server with a pick object action lib client')
        client = actionlib.SimpleActionClient('pick_object_server', PickObjectAction)
        rospy.loginfo('waiting for pick_object_server to come up (1 sec)')
        if not client.wait_for_server(timeout=rospy.Duration.from_sec(1)):
            rospy.logerr('pick_object_server in not up, did you launched _demos pick_object_demo.launch?')
            self._widget.lblPickObjClient_status.setText("Error : pick_object_server not up !")
            self._widget.cmdPickObjClient_run.setEnabled(True)
            return False
        rospy.loginfo('pick_object_server is up')
        goal = PickObjectGoal()
        try:
            # get object to be picked from textbox
            obj_to_pick = self._widget.textBox_pick_obj_client_obj_name.toPlainText()
            # sanity checks: ensure object was perceived before
            if not self.object_list:
                rospy.logerr('before picking you have to perceive the object !')
                self._widget.cmdPickObjClient_run.setEnabled(True)
                return False

            if obj_to_pick not in self.object_list:
                rospy.logerr('You have triggered perception. all good but the object you requested to pick is not found on the object list!')
                return False

            # fill action lib client msg goal
            goal.object = obj_to_pick
            rospy.loginfo('Sending action lib goal to pick_object_server : ' + goal.object)
            client.send_goal(goal)
            # get timeout from box
            pick_timeout = float(self._widget.txtPickObjClient_timeout.toPlainText())
            rospy.loginfo('waiting for server response for %f secs'% pick_timeout)
            client.wait_for_result(rospy.Duration.from_sec(pick_timeout))
            if client.get_result().success:
                rospy.loginfo('Succesfull grasp ! : ) congrats !')
                # set text on qt label
                self._widget.lblPickObjClient_status.setText("Object grasped succesfully ! : )")
                # enable button again
                self._widget.cmdPickObjClient_run.setEnabled(True)
                return True
            else:
                rospy.logerr('failed to grasp object : (')
                # set text on qt label
                self._widget.lblPickObjClient_status.setText("Failed to grasp obj !")
                # enable button again
                self._widget.cmdPickObjClient_run.setEnabled(True)
                return False
        except:
            rospy.logerr('An exception occurred while calling pick_object_server')
            # set text on qt label
            self._widget.lblPickObjClient_status.setText("Exception ocurred... !")
            # enable button again
            self._widget.cmdPickObjClient_run.setEnabled(True)
            return False


    def handle_cancel_pick_action_client(self):
        '''
        cancel the goal send by self.handle_run_pick_action_client()
        '''
        rospy.logwarn('Not implemented yet!')
        self._widget.lblPickObjClient_status.setText("Error: cancel button is not implemented yet")


    def handle_perception_recognize_obj(self):
        '''
        trigger perception to recognize workspace objects
        '''
        rospy.loginfo('Perceiving objects with  head cam')

        # get confidence from gui textbox
        try:
            perception_confidence = float(self._widget.txtPerception_confidence.toPlainText())
        except:
            rospy.logerr('failed while converting perception confidence to float')
            self._widget.lblPerception_status_feedback.setText("error !")
            return

        # get object to perceive from text box
        object_to_perceive = self._widget.txtPerception_target_obj_name.toPlainText()

        # alter obj to pick according to the one to perceive, if you want to perceive that one makes sense later on you want to pick it
        self._widget.textBox_pick_obj_client_obj_name.setPlainText(object_to_perceive)

        # use  robot class to call perception
        # self.object_list = self.robot.perception.perceive_object(confidence=perception_confidence)

        rospy.loginfo('obj list : ' + str(self.object_list))

        if self.object_list:
            # check if target obj is among the ones perceived
            if object_to_perceive in self.object_list:
                self._widget.lblPerception_status_feedback.setText('ok ! (' + str(self.perception_count) + ')')
                self.perception_count += 1
                # inform pick frame that object is ready to be picked
                self._widget.lblPickObjClient_status.setText("Ready to pick !")
            else:
                rospy.loginfo('some objects were found but not the one you want to pick!')
                self._widget.lblPerception_status_feedback.setText('failed ! (' + str(self.perception_count) + ')')
                self.perception_count += 1
        else:
            rospy.loginfo('No object found by perception pipeline')
            self._widget.lblPerception_status_feedback.setText("empty !")

   

    def objMockUpGUI (self,msg):

        self._widget.fraPregraspPlanner.setEnabled(True)

        self.pose_in = msg

        self.mockup_pub_thread.set_pose(self.pose_gui)
        self.mockup_pub_thread.start()

    def checkTF (self):

        #This function get executed every time you receive a goal pose message
        
        # ensure that the code gets executed from beginning to end one time completely
        # while not rospy.is_shutdown():
        #     # try:
        #     # refresh the timestamp of the received pose
        #self.pose_in.header.stamp = rospy.Time.now() - rospy.Duration(0.025) # hack
        # wait for transform to become availble
        #self.listener2.waitForTransform("head_camera_depth_optical_frame","base_link",self.pose_in.header.stamp,\
        #                              rospy.Duration(self.wait_for_transform))
        base_to_odom = self.listener2.lookupTransform("head_camera_depth_optical_frame","base_link",rospy.Time.now() - - rospy.Duration(0.05))

        print(base_to_odom)

        # if base_to_odom:
        #     self.base_to_odom = base_to_odom
        #     self.goalpose_msg_odom = self.listener2.transformPose('odom',self.pose_in)
        #     self.goalpose_msg = self.listener2.transformPose('base_link',self.pose_in) #goal pose in reference to base_link
            
        #     print self.goalpose_msg

        #     self.ready_to_start = True

            

        #self.goalpose_received = True
        # print ("goal pose received:", self.goalpose_msg)
        #return

    def handle_moveit_goto_out_of_view(self):

        print("no iiiiffffffffffffffffffffffffffffffffffffffffffffff")
        
        

        # self.goalpose_msg=self.pose_in

        # rospy.sleep(10)

        # a=0
        # b=20

        # while a<b:
        #     a=a+1
        #     b=b-1
        #     print a
        #     print b
        #     self.vel_base.linear.x = 0
        #     self.vel_base.linear.y = 0
        #     self.vel_base.linear.z = 0.0
        #     self.vel_base.angular.x = 0.0 
        #     self.vel_base.angular.y = 0.0
        #     self.vel_base.angular.z = -0.3  #so sera necessario rotacao em torno do Z (com uma velocidade arbitaria??? -0.1 parece bem)  
        #     self.pub_base_vel_twist.publish(self.vel_base)
        #     rospy.sleep(1)

        # self.vel_base.linear.x = 0
        # self.vel_base.linear.y = 0
        # self.vel_base.linear.z = 0.0
        # self.vel_base.angular.x = 0.0 
        # self.vel_base.angular.y = 0.0
        # self.vel_base.angular.z = 0.0    #so sera necessario rotacao em torno do Z (com uma velocidade arbitaria??? -0.1 parece bem)  
        # self.pub_base_vel_twist.publish(self.vel_base)


        # self.checkTF()

        # while abs(self.goalpose_msg.pose.position.x) > abs(self.goalpose_msg.pose.position.y):  # este abs e para tirar
        #     print "no whileeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"
            

        self.vel_base.linear.x = 0
        self.vel_base.linear.y = 0
        self.vel_base.linear.z = 0.0
        self.vel_base.angular.x = 0.0 
        self.vel_base.angular.y = 0.0
        self.vel_base.angular.z = -0.2  #so sera necessario rotacao em torno do Z (com uma velocidade arbitaria??? -0.1 parece bem)  
        self.pub_base_vel_twist.publish(self.vel_base)

        #     self.checkTF()

        #     rospy.sleep(0.2)
        

        # self.vel_base.linear.x = 0
        # self.vel_base.linear.y = 0
        # self.vel_base.linear.z = 0
        # self.vel_base.angular.x = 0.0 
        # self.vel_base.angular.y = 0.0
        # self.vel_base.angular.z = 0.0  
        # self.pub_base_vel_twist.publish(self.vel_base)




        '''
        use moveit to move the arm to a prerecorded arm pose
        out of view: not to see your own arm while perceiving objects from the workspace
        '''
        # self.moveit_goto_pose('out_of_view')

    def objSelectorPoseOutCB(self, msg):
        '''
        Callback for the output geometry_msgs/PoseStamped msg from the object selector
        that contains the pose of the object
        '''
        self.obj_pose_received = True
        self.obj_pose = msg


    def objSelectorCallback(self, msg):
        '''
        object selector event_out callback
        '''
        if msg.data == 'e_selected':
            # give some time for obj pose to arrive
            rospy.sleep(0.5)
            # check if obj pose arrived
            if self.obj_pose_received:
                # update obj pose
                self.mockup_pub_thread.set_pose(self.obj_pose)
                rospy.loginfo('updating mockup pose from the one received by perception')
                # reset flag
                self.obj_pose_received = False


    def handle_perception_mockup_obj(self, checked):
        '''
        Query object selector for the obj pose, save it in member variable and
        start publishing it constantly
        '''
        if checked:
            if self.perception_count < 1:
                rospy.logerr('Error: to mockup pose you need to perceive at least one time!')
                self._widget.lblPerception_status_feedback.setText('failed to mockup')
                return
            rospy.loginfo('publishing obj pose mockup!')
            self._widget.lblPerception_status_feedback.setText('mockup pose enabled')
            self.mockup_obj = True
            # get object name from textbox
            object_to_perceive = self._widget.txtPerception_target_obj_name.toPlainText()
            # send to object selector the name of the object from which we want the pose back
            self.pub_obj_selector_obj_name.publish(std_msgs.msg.String(object_to_perceive))
            # trigger object selector
            self.pub_obj_selector_trigger.publish(std_msgs.msg.String('e_trigger'))
            # give some time for obj pose to arrive
            rospy.sleep(0.5)
            if self.obj_pose:
                # start publishing pose mockup
                self.mockup_pub_thread.start()
                # enable pregrasp planner frame
                # enable
                self._widget.fraPregraspPlanner.setEnabled(True)
            else:
                rospy.logerr('obj pose not received, cannot mockup obj pose')
        else:
            # disable pregrasp planner frame needs mockup pose
            self._widget.fraPregraspPlanner.setEnabled(False)
            if self.mockup_obj:
                rospy.loginfo('stop publishing obj pose mockup!')
                self.mockup_obj = False
                # stop mokcup pose thread
                self.mockup_pub_thread.terminate()
                # give feedback to user
                self._widget.lblPerception_status_feedback.setText('mockup pose disabled')


    def handle_switch_arm_to_position(self):
        '''
         arm has trajectory ctrler enabled by default
        this function switches to position ctrl
        e.g.
        rosservice call /controller_manager/switch_controller "
        start_controllers:
        - 'left_arm_joint0_position_controller'
        - 'left_arm_joint1_position_controller'
        - 'left_arm_joint2_position_controller'
        - 'left_arm_joint3_position_controller'
        - 'left_arm_joint4_position_controller'
        - 'left_arm_joint5_position_controller'
        - 'left_arm_joint6_position_controller'
        stop_controllers:
        - 'left_arm_joint0_velocity_controller'
        - 'left_arm_joint1_velocity_controller'
        - 'left_arm_joint2_velocity_controller'
        - 'left_arm_joint3_velocity_controller'
        - 'left_arm_joint4_velocity_controller'
        - 'left_arm_joint5_velocity_controller'
        - 'left_arm_joint6_velocity_controller'
        - 'left_arm_traj_controller'
        strictness: 1" # strict 2, best effort 1
        '''        
        rospy.loginfo('Switching arm to position control')
        rospy.loginfo('Waiting for /controller_manager/switch_controller service')
        try:
            rospy.wait_for_service ('/controller_manager/switch_controller', 1.0)
            rospy.loginfo('Gazebo service /controller_manager/switch_controller found !, will switch arm ctrl to position now...')
            self.switch_arm_srv(self.switch_ctrl_request_position)
            # keep a history of the state of the crtler
            self.arm_ctrl_memory = 'position'
        except Exception as e:
            rospy.logerr('gazebo service : /controller_manager/switch_controller not available, could not set arm ctrl to position !')
            rospy.logerr(e)


    def handle_switch_arm_to_trajectory(self):
        '''
        switch  arm back into trajectory ctrl for moveit to be able to work
        '''
        rospy.loginfo('Switching arm to trajectory control')
        rospy.loginfo('Waiting for /controller_manager/switch_controller service')
        try:
            rospy.wait_for_service ('/controller_manager/switch_controller', 1.0)
            rospy.loginfo('Gazebo service /controller_manager/switch_controller found !, will switch arm ctrl to trajectory now...')
            self.switch_arm_srv(self.switch_ctrl_request_trajectory)
            # keep a history of the state of the crtler
            self.arm_ctrl_memory = 'trajectory'
        except Exception as e:
            rospy.logerr('gazebo service : /controller_manager/switch_controller not available, could not set arm ctrl to trajectory !')
            rospy.logerr(e)


    def moveit_goto_pose(self, arm_configuration):
        '''
        switch arm ctrl to trajectory and then go to pose
        '''
        rospy.loginfo('Moving arm to arm configuration : ' + arm_configuration)
        if self.sim_robot:
            # do your best to remember the last state of the arm and change ctrl mode if needed
            if self.arm_ctrl_memory == 'position':
                # switch arm to trajectory ctrl
                self.handle_switch_arm_to_trajectory()
        # go to arm pose
        self.robot.manipulation.go_to_pose(arm_configuration, wait=True)


    





    def handle_moveit_goto_pregrasp(self):
        '''
        use moveit to move the arm to a prerecorded arm pose
        pregrasp: go to pregrasp arm configuration prior to go for the object
        '''
        ###################self.moveit_goto_pose('pregrasp')

        self.planning_scene_pub.publish(String('e_start'))

        #self.checkTF()


    def handle_moveit_goto_eg_obj(self):
        '''
        use moveit to move the arm to a prerecorded arm pose
        useful to go to the object in a harcoded way
        '''
        self.moveit_goto_pose('tea_box')


    def handle_moveit_kill_arm(self):
        
        self.pregrasp_planner_pub.publish(String('e_stop'))

        '''
        use moveit to move the arm to a prerecorded arm pose
        useful to send the arm to a position where it can safely turn off torque (be killed)
        '''
        # use moveit to move the arm to prerecorded arm pose : 'kill'
        # self.moveit_goto_pose('kill')
        # # disable torque on the motors
        # self.pub_arm_command.publish(std_msgs.msg.String('disable_torque'))


    def handle_moveit_goto_calib(self):   # usar este callback deste botao para executar o planned motion
        


        self.motion_planner_pub.publish(String('e_start'))


        # '''
        # use moveit to move the arm to a prerecorded arm pose
        # move arm to calib:  looks at its own arm to asses how good is the calibration
        # '''
        # rospy.loginfo('Moving arm to arm configuration : calib')
        # self.moveit_goto_pose('calib')

    def handle_moveit_goto_candle(self):
        '''
        use moveit to move the arm to a prerecorded arm pose
        useful to send the arm to a intermediate pose avoiding possible collisions with the workspace
        '''
        rospy.loginfo('Moving arm to arm configuration : candle, not implemented')
        # self.moveit_goto_pose('candle')

    def handle_gazebo_reset_world(self):
        '''
        reset gazebo model states
        calls:
            rosservice call /gazebo/reset_world "{}"
        '''
        rospy.loginfo('Reseting gazebo world')
        try:
            rospy.wait_for_service ('/gazebo/reset_world', 1.0)
            rospy.loginfo('Gazebo service /gazebo/reset_world found !, will reset now...')
            self.gazebo_reset_srv()
        except Exception as e:
            rospy.logerr('gazebo service : /gazebo/reset_world not available, could not reset world!')
            rospy.logerr(e)


    def update_obj_bounds(self):
        '''
        read obj bounds from textbox and update member variable values
        '''
        try:
            self.obj_bounds_x_min = float(self._widget.txtGazebo_x_min.toPlainText())
            self.obj_bounds_x_max = float(self._widget.txtGazebo_x_max.toPlainText())
            self.obj_bounds_y_min = float(self._widget.txtGazebo_y_min.toPlainText())
            self.obj_bounds_y_max = float(self._widget.txtGazebo_y_max.toPlainText())
            self.obj_bounds_z_min = float(self._widget.txtGazebo_z_min.toPlainText())
            self.obj_bounds_z_max = float(self._widget.txtGazebo_z_max.toPlainText())
        except:
            rospy.logerr('Exception while parsing obj bounds into floats, check input values')


    def handle_gazebo_randomize_obj(self):
        '''
        randomize the object pose to test robustness
        '''
        rospy.loginfo('Randomizing object pose!')

        # read bounds from textbox to update
        self.update_obj_bounds()

        # configure obj pose request
        random_obj_pose_req = SetModelStateRequest()

        # randomize obj pose between user bounds (from param server)
        random_obj_x = random.uniform(self.obj_bounds_x_min, self.obj_bounds_x_max)
        random_obj_y = random.uniform(self.obj_bounds_y_min, self.obj_bounds_y_max)
        random_obj_z = random.uniform(self.obj_bounds_z_min, self.obj_bounds_z_max)

        # randomize yaw between 0 - 360 degree
        random_yaw = random.uniform(0.0, math.pi*2)

        # get obj quaternion from roll, pitch, yaw (yaw is random, preserve original obj roll and pitch)
        quaternion = tf.transformations.quaternion_from_euler(self.graspable_obj[4], self.graspable_obj[5], random_yaw)

        # fill obj pose request
        random_obj_pose_req.model_state.model_name = self.graspable_obj[0] # preserve original object name
        random_obj_pose_req.model_state.pose.position.x = random_obj_x
        random_obj_pose_req.model_state.pose.position.y = random_obj_y
        random_obj_pose_req.model_state.pose.position.z = random_obj_z
        random_obj_pose_req.model_state.pose.orientation.x = quaternion[0]
        random_obj_pose_req.model_state.pose.orientation.y = quaternion[1]
        random_obj_pose_req.model_state.pose.orientation.z = quaternion[2]
        random_obj_pose_req.model_state.pose.orientation.w = quaternion[3]
        random_obj_pose_req.model_state.reference_frame = "" # leave it empty, it works...
        self.gazebo_set_obj_pose_srv(random_obj_pose_req)


    def handle_gazebo_reset_obj_pose(self):
        '''
        send object back to the example pose
        example pose: the one which quaternion was recorded to substitute the obj orientation
        in the simple pregrasp planner pipeline
        '''
        rospy.loginfo('Reset obj pose')
        # call gazebo service to set obj pose to example pose
        self.gazebo_set_obj_pose_srv(self.obj_pose_req)

    def handle_scripts_grasp_obj(self):
        '''
        grasp via moveit!
        '''
        rospy.loginfo('trying to grasp object now')
        obj_to_pick = self._widget.textBox_pick_obj_client_obj_name.toPlainText()
        # send string trigger to topic
        self.pregrasp_planner_pub.publish(String(obj_to_pick))

    def handle_scripts_grasp_5_obj(self):
        '''
        run handle_scripts_grasp_obj 5 times
        '''
        rospy.loginfo('Trying to grasp the object mulitple times!')
        try:
            times_to_grasp = int(self._widget.txtScripts_multiple.toPlainText())
        except:
            rospy.logerr('Failed to parse box into a integer, check input value')
            return

        for i in range(1, times_to_grasp + 1):
            self.handle_scripts_grasp_obj()
            if self.sim_robot:
                # randomize obj pose after first grasp (simulation only)
                self.handle_gazebo_randomize_obj()


    def handle_head_cam_look_to_obj(self):
        '''
        move camera down to have the object in fov
        '''
        rospy.loginfo('Tilt  head cam to have the object in the field of view, not implemented')
        # self.robot.perception.tilt_head_camera(self.obj_cam_angle_fov)
        # update label
        self._widget.boxHeadCamTilt_cam_angle.setValue(self.obj_cam_angle_fov)


    def handle_reload_dynamic_reconfigure(self):
        '''
        publish std_msgs/Empty to mcr_dynamic_reconfigure_client such that
        it reloads the config file. This is done for the pick object server
        that performs calls to dynamic reconfigure in one of his steps.
        '''
        rospy.loginfo('Reloading dynamic reconfigure client config file')
        # publish to mcr_dynamic_reconfigure_client
        self.pub_dynamic_reconf.publish(std_msgs.msg.Empty())


    def ppCallback(self, msg):
        '''
        Callback to receive the response of the simple pregrasp planner
        we expect a std_msgs/String either e_success or e_failure
        '''
        if msg.data == 'e_success':
            rospy.loginfo('pregrasp planner succedeed')
            self._widget.lblPP_status_feedback.setText('e_success')
        elif msg.data == 'e_failure':
            rospy.loginfo('pregrasp planner reported failure')
            self._widget.lblPP_status_feedback.setText('e_failure')
        else:
            rospy.loginfo('received unsupported event "%s" from pregrasp planner, admissible values are: e_success, e_failure'%msg.data)
            self._widget.lblPP_status_feedback.setText('error: received unsupported event')

    def reconfigure_pp_params(self):
        '''
        call dynamic reconfigure from code to alter pp params without having to open dynamic reconfigure
        '''
        try:
            # read textbox values
            min_azimuth = float(self._widget.txtPP_min_azimuth.toPlainText())
            max_azimuth = float(self._widget.txtPP_max_azimuth.toPlainText())
            min_zenith = float(self._widget.txtPP_min_zenith.toPlainText())
            max_zenith = float(self._widget.txtPP_max_zenith.toPlainText())
            min_roll = float(self._widget.txtPP_min_roll.toPlainText())
            max_roll = float(self._widget.txtPP_max_roll.toPlainText())

          

            # create params dictionary
            params = {"min_azimuth": min_azimuth}   #cria um dictionary com 1 elemento
 
            params.update({'max_azimuth': max_azimuth}) #usa a funcao do python update para adicionar mais elementos
            params.update({'min_zenith': min_zenith})
            params.update({'max_zenith': max_zenith})
            params.update({'min_roll': min_roll})
            params.update({'max_roll': max_roll})
        except:
            rospy.logerr('Failed while parsing pregrasp planner parameters, check input values')
            return

        # call dynamic reconfigure api
        try:
            node_name = self.pregrasp_planner_node_name
            client = dynamic_reconfigure.client.Client(node_name, timeout=1.5)
        except:
            rospy.logerr("Service {0} does not exist".format(node_name + '/set_parameters'))

        try:
            config = client.update_configuration(params)
        except Exception as e:
            rospy.logerr("Error: %s", str(e))


    def handle_start_pregrasp_planner(self):
        '''
        send e trigger to pregrasp planner pipeline
        event out of the SPP (simple pregrasp planner) is handled in ppCallback directly
        '''
        self._widget.lblPP_status_feedback.setText('busy...')
        self.reconfigure_pp_params()
        rospy.loginfo('Trigger pregrasp planner pipeline')
        self.pregrasp_planner_pub.publish(String('e_start'))


    def handle_head_cam_tilt_slider(self, value):
        '''
        Move  head pan according to the value of the slide
        '''
        try:
            self.robot.hri.rotate_head_value(value, self.pan_vel)
            self._widget.boxHeadPan_neck_angle.setValue(value)
            self.head_rotation_previous = value
        except Exception as e:
            self._widget.boxHeadPan_neck_angle.setValue(self.head_rotation_previous)
            self._widget.head_rotation_slider.setValue(self.head_rotation_previous)
            rospy.logerr(e)


    def handle_head_rotation_box(self, value):
        '''
        Move  head pan according to the number input in the box
        '''
        self.handle_head_cam_tilt_slider(value)
        # move slide if box changes
        self._widget.head_rotation_slider.setValue(value)


    def handle_camera_tilt_slider(self, value):
        '''
        Move  head cam up and down (tilt) to put the object in fov with slide
        '''
        try:
            self.robot.perception.tilt_head_camera(value)
            self._widget.boxHeadCamTilt_cam_angle.setValue(value)
            self.head_camera_tilt_previous = value
        except Exception as e:
            self._widget.boxHeadCamTilt_cam_angle.setValue(self.head_camera_tilt_previous)
            self._widget.head_camera_tilt_slider.setValue(self.head_camera_tilt_previous)
            rospy.logerr(e)


    def head_cam_tilt_box(self, value):
        '''
        Move  head cam up and down (tilt) to put the object in fov by input number in a box
        '''
        self.handle_camera_tilt_slider(value)
        # move slide if box changes
        self._widget.head_camera_tilt_slider.setValue(self.head_camera_tilt_previous)
