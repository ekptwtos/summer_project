#!/usr/bin/env python

import rospy, sys
import actionlib
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor , AttachedCollisionObject
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from gazebo_msgs.msg import ModelStates
from tf import transformations
import numpy as np
import  pr2_controllers_msgs.msg as pr2c 
import time, threading, thread
from shape_msgs.msg import SolidPrimitive



GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper' 

GRIPPER_FRAME = 'r_wrist_roll_link'  # 'r_gripper_tool_frame' does not work in planning!!!!! it has something to do with the description of the robot i think // 'r_wrist_roll_link' 

GRIPPER_OPEN = [0.088]
GRIPPER_CLOSED = [0.03]
GRIPPER_NEUTRAL = [0.05]

GRIPPER_JOINT_NAMES = ['r_gripper_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'base_footprint'
obj_att = None

class MoveItDemo:
    def __init__(self):

        global obj_att

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_demo')
        
        #Initialize robot
        robot = moveit_commander.RobotCommander()

        # Use the planning scene object to add or remove objects
        self.scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
        
        # Create a publisher for displaying object frames
        self.object_frames_pub = rospy.Publisher('object_frames', PoseStamped, queue_size=10)

        ### Create a publisher for visualizing direction ###
        self.p_pub = rospy.Publisher('target', PoseStamped, latch=True, queue_size = 10)


        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the MoveIt! commander for the arm
        self.right_arm = MoveGroupCommander(GROUP_NAME_ARM)

        # Initialize the MoveIt! commander for the gripper
        self.right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Allow 5 seconds per planning attempt
        self.right_arm.set_planning_time(5)

        # Prepare Action Controller for gripper
        self.ac = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',pr2c.Pr2GripperCommandAction)
        self.ac.wait_for_server()

        # Give the scene a chance to catch up
        rospy.sleep(2)
        
        # Prepare Gazebo Subscriber
        self.pwh = None
        self.pwh_copy = None
        self.idx_targ = None
        self.gazebo_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)


        ### OPEN THE GRIPPER ###
        self.open_gripper()


        # PREPARE THE SCENE
        while self.pwh is None:
            rospy.sleep(0.05)

        ############## CLEAR THE SCENE ################


#        planning_scene.world.collision_objects.remove('target')

        # Remove leftover objects from a previous run
        self.scene.remove_world_object('target')
        self.scene.remove_world_object('table')
#        self.scene.remove_world_object(obstacle1_id)

        # Remove any attached objects from a previous session
        self.scene.remove_attached_object(GRIPPER_FRAME, 'target')

        # Run and keep in the BG the scene generator also add the ability to kill the code with ctrl^c
        timerThread = threading.Thread(target=self.scene_generator)
        timerThread.daemon = True
        timerThread.start()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'gazebo_world'
        initial_pose.pose = target_pose.pose
        

        print "==================== Generating Transformations ==========================="

        #################### PRE GRASPING POSE #########################
        M1 = transformations.quaternion_matrix([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
        M1[0,3] = target_pose.pose.position.x
        M1[1,3] = target_pose.pose.position.y 
        M1[2,3] = target_pose.pose.position.z

        M2 = transformations.euler_matrix(0, 1.57, 0)
        M2[0,3] = 0.0  # offset about x
        M2[1,3] = 0.0   # about y
        M2[2,3] = 0.25  # about z

        T = np.dot(M1,  M2)
        pre_grasping = deepcopy(target_pose)
        pre_grasping.pose.position.x = T[0,3] 
        pre_grasping.pose.position.y = T[1,3]
        pre_grasping.pose.position.z = T[2,3]

        quat = transformations.quaternion_from_matrix(T)
        pre_grasping.pose.orientation.x = quat[0]
        pre_grasping.pose.orientation.y = quat[1]
        pre_grasping.pose.orientation.z = quat[2]
        pre_grasping.pose.orientation.w = quat[3]
        pre_grasping.header.frame_id = 'gazebo_world'
        self.plan_exec(pre_grasping)


        #################### GRASPING POSE #########################

        M3 = transformations.quaternion_matrix([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
        M3[0,3] = target_pose.pose.position.x
        M3[1,3] = target_pose.pose.position.y 
        M3[2,3] = target_pose.pose.position.z

        M4 = transformations.euler_matrix(0, 1.57, 0)
        M4[0,3] = 0.0  # offset about x
        M4[1,3] = 0.0   # about y
        M4[2,3] = 0.18  # about z

        T2 = np.dot(M3,  M4)
        grasping = deepcopy(target_pose)
        grasping.pose.position.x = T2[0,3] 
        grasping.pose.position.y = T2[1,3]
        grasping.pose.position.z = T2[2,3]

        quat2 = transformations.quaternion_from_matrix(T2)
        grasping.pose.orientation.x = quat2[0]
        grasping.pose.orientation.y = quat2[1]
        grasping.pose.orientation.z = quat2[2]
        grasping.pose.orientation.w = quat2[3]
        grasping.header.frame_id = 'gazebo_world'

        self.plan_exec(grasping)


        #Close the gripper
        print "========== Waiting for gazebo to catch up =========="
        self.close_gripper()



        #################### ATTACH OBJECT ######################

        touch_links = [GRIPPER_FRAME, 'r_gripper_l_finger_tip_link','r_gripper_r_finger_tip_link', 'r_gripper_r_finger_link', 'r_gripper_l_finger_link']
        #print touch_links
        self.scene.attach_box(GRIPPER_FRAME, target_id, target_pose, target_size, touch_links)

        # counter to let the planning scene know when to remove the object
        obj_att = 1

        #self.scene.remove_world_object(target_id)

        #################### POST-GRASP RETREAT #########################

        M5 = transformations.quaternion_matrix([initial_pose.pose.orientation.x, initial_pose.pose.orientation.y, initial_pose.pose.orientation.z, initial_pose.pose.orientation.w])
        M5[0,3] = initial_pose.pose.position.x
        M5[1,3] = initial_pose.pose.position.y 
        M5[2,3] = initial_pose.pose.position.z

        M6 = transformations.euler_matrix(0, 1.57, 0)
        M6[0,3] = 0.0  # offset about x
        M6[1,3] = 0.0  # about y
        M6[2,3] = 0.3  # about z

        T3 = np.dot(M5,  M6)
        post_grasping = deepcopy(initial_pose)
        post_grasping.pose.position.x = T3[0,3] 
        post_grasping.pose.position.y = T3[1,3]
        post_grasping.pose.position.z = T3[2,3]

        quat3 = transformations.quaternion_from_matrix(T3)
        post_grasping.pose.orientation.x = quat3[0]
        post_grasping.pose.orientation.y = quat3[1]
        post_grasping.pose.orientation.z = quat3[2]
        post_grasping.pose.orientation.w = quat3[3]
        post_grasping.header.frame_id = 'gazebo_world'

        self.plan_exec(post_grasping)




        # Specify a pose to place the target after being picked up
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = 0.52
        place_pose.pose.position.y = -0.48
        place_pose.pose.position.z = 0.48
        place_pose.pose.orientation.w = 1.0


        n_attempts = 0
        max_place_attempts = 2
        # Generate valid place poses
        places = self.make_places(place_pose)

        success = False
        # Repeat until we succeed or run out of attempts
        while success == False and n_attempts < max_place_attempts:
            for place in places:
                success = self.right_arm.place(target_id, place)
                if success:
                    break
            n_attempts += 1
            rospy.loginfo("Place attempt: " +  str(n_attempts))
            rospy.sleep(0.2)


        self.open_gripper()
        obj_att = None
        rospy.sleep(3)



##        # Initialize the grasp object
##        g = Grasp()
##        grasps = []
##        # Set the first grasp pose to the input pose
##        g.grasp_pose = pre_grasping
##        g.allowed_touch_objects = [target_id]
##        grasps.append(deepcopy(g))

##        right_arm.pick(target_id, grasps)


#        #Change the frame_id for the planning to take place!
#        #target_pose.header.frame_id = 'gazebo_world'


#        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

#        # Exit the script
        moveit_commander.os._exit(0)

##################################################################################################################

    #Get pose from Gazebo
    def model_state_callback(self,msg):

        self.pwh = ModelStates()
        self.pwh = msg

    # Generate a list of possible place poses
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()

        # Start with the input place pose
        place = init_pose

        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]

        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]       

        # A list of pitch angles to try
        #pitch_vals = [0, 0.005, -0.005, 0.01, -0.01, 0.02, -0.02]

        pitch_vals = [0]

        # A list of yaw angles to try
        yaw_vals = [0]

        # A list to hold the places
        places = []

        # Generate a place pose for each angle and translation
        for y in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.pose.position.x = init_pose.pose.position.x + x
                        place.pose.position.y = init_pose.pose.position.y + y

                        # Create a quaternion from the Euler angles
                        q = quaternion_from_euler(0, p, y)

                        # Set the place pose orientation accordingly
                        place.pose.orientation.x = q[0]
                        place.pose.orientation.y = q[1]
                        place.pose.orientation.z = q[2]
                        place.pose.orientation.w = q[3]

                        # Append this place pose to the list
                        places.append(deepcopy(place))

        # Return the list
        return places


    def plan_exec(self, pose):

        self.right_arm.clear_pose_targets()
        self.right_arm.set_pose_target(pose, GRIPPER_FRAME)
        self.right_arm.plan()
        rospy.sleep(5)
        self.right_arm.go(wait=True)


    def close_gripper(self):

        g_close = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(0.035, 100))
        self.ac.send_goal(g_close)
        self.ac.wait_for_result()
        rospy.sleep(15) # Gazebo requires up to 15 seconds to attach object


    def open_gripper(self):

        g_open = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(0.088, 100))
        self.ac.send_goal(g_open)
        self.ac.wait_for_result()
        rospy.sleep(5) # And up to 20 to detach it


    def scene_generator(self):
#        print obj_att
        global target_pose
        global target_id
        global target_size
        target_id = 'target'
        self.taid = self.pwh.name.index('wood_cube_5cm')
        table_id = 'table'
        self.tid = self.pwh.name.index('table') 
        #obstacle1_id = 'obstacle1'
        #self.o1id = self.pwh.name.index('wood_block_10_2_1cm')


        # Set the target size [l, w, h]
        target_size = [0.05, 0.05, 0.05]
        table_size = [1.5, 0.8, 0.03]
        #obstacle1_size = [0.1, 0.025, 0.01]

        ## Set the target pose on the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose = self.pwh.pose[self.taid]
        target_pose.pose.position.z += 0.025
        # Add the target object to the scene
        if obj_att is None:
            self.scene.add_box(target_id, target_pose, target_size)

            table_pose = PoseStamped()
            table_pose.header.frame_id = REFERENCE_FRAME
            table_pose.pose = self.pwh.pose[self.tid]
            table_pose.pose.position.z += 1
            self.scene.add_box(table_id, table_pose, table_size)
            
            #obstacle1_pose = PoseStamped()
            #obstacle1_pose.header.frame_id = REFERENCE_FRAME
            #obstacle1_pose.pose = self.pwh.pose[self.o1id]
            ## Add the target object to the scene
            #scene.add_box(obstacle1_id, obstacle1_pose, obstacle1_size)

            # Specify a pose to place the target after being picked up
            place_pose = PoseStamped()
            place_pose.header.frame_id = REFERENCE_FRAME
            place_pose.pose.position.x = 0.50
            place_pose.pose.position.y = -0.30
            place_pose.pose.orientation.w = 1.0

            # Add the target object to the scene
            self.scene.add_box(target_id, target_pose, target_size)
                    
            ### Make the target purple ###
            self.setColor(target_id, 0.6, 0, 1, 1.0)

            # Send the colors to the planning scene
            self.sendColors()
        else: 
            self.scene.remove_world_object('target')
        # Publish targe's frame
        #self.object_frames_pub.publish(target_pose)


        threading.Timer(0.5, self.scene_generator).start()

    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()

        # Set the id to the name given as an argument
        color.id = name

        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True

        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)

        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItDemo()

