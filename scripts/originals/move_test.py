#!/usr/bin/env python

import rospy, sys
import actionlib
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from gazebo_msgs.msg import ModelStates
from tf import transformations
import numpy as np
import  pr2_controllers_msgs.msg as pr2c 

GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper' 

GRIPPER_FRAME = 'r_wrist_roll_link'  # 'r_gripper_tool_frame' does not work in planning!!!!! it has something to do with the description of the robot i think // 'r_wrist_roll_link' 

GRIPPER_OPEN = [0.088]
GRIPPER_CLOSED = [0.03]
GRIPPER_NEUTRAL = [0.05]

GRIPPER_JOINT_NAMES = ['r_gripper_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'base_footprint'


class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('moveit_demo')
        
        #Initialize robot
        robot = moveit_commander.RobotCommander()

        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
        
        # Create a publisher for displaying object frames
        self.object_frames_pub = rospy.Publisher('object_frames', PoseStamped, queue_size=10)

        ## Create a publisher for visualizing direction ###
        self.p_pub = rospy.Publisher('target', PoseStamped, latch=True, queue_size = 10)

        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the MoveIt! commander for the arm
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)

        # Initialize the MoveIt! commander for the gripper
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Allow 5 seconds per planning attempt
        right_arm.set_planning_time(5)

        # Prepare Gripper and open it
        self.ac = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',pr2c.Pr2GripperCommandAction)
        self.ac.wait_for_server()

        g_open = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(0.088, 100))
        g_close = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(0.0, 1))
        self.ac.send_goal(g_open)

        # Give the scene a chance to catch up
        rospy.sleep(2)
        
        # Prepare Gazebo Subscriber
        self.pwh = None
        self.pwh_copy = None
        self.idx_targ = None
        self.gazebo_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)


        rospy.sleep(2)
        
        # PREPARE THE SCENE
       	
       	while self.pwh is None:
            rospy.sleep(0.05)

        target_id = 'target'
        self.taid = self.pwh.name.index('wood_cube_5cm')
        table_id = 'table'
        self.tid = self.pwh.name.index('table') 
        #obstacle1_id = 'obstacle1'
        #self.o1id = self.pwh.name.index('wood_block_10_2_1cm')

        # Remove leftover objects from a previous run
        scene.remove_world_object(target_id)
        scene.remove_world_object(table_id)
        #scene.remove_world_object(obstacle1_id)

        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, target_id)

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
        scene.add_box(target_id, target_pose, target_size)

        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose = self.pwh.pose[self.tid]
        table_pose.pose.position.z += 1
        scene.add_box(table_id, table_pose, table_size)
        
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
        scene.add_box(target_id, target_pose, target_size)
                
        ### Make the target purple ###
        self.setColor(target_id, 0.6, 0, 1, 1.0)

        # Send the colors to the planning scene
        self.sendColors()
        
        #print target_pose
        self.object_frames_pub.publish(target_pose)
        rospy.sleep(2)

        print "==================== Generating Transformations ==========================="

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


#        # Initialize the grasp object
#        g = Grasp()
#        grasps = []
#        # Set the first grasp pose to the input pose
#        g.grasp_pose = pre_grasping
#        g.allowed_touch_objects = [target_id]
#        grasps.append(deepcopy(g))

#        right_arm.pick(target_id, grasps)


        #Change the frame_id for the planning to take place!
        #target_pose.header.frame_id = 'gazebo_world'
        
        self.p_pub.publish(pre_grasping)
        right_arm.set_pose_target(pre_grasping, GRIPPER_FRAME)
        plan = right_arm.plan()
        rospy.sleep(2)
        right_arm.go(wait=True)



        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)

    
    #Get pose from Gazebo
    def model_state_callback(self,msg):

        self.pwh = ModelStates()
        self.pwh = msg


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

