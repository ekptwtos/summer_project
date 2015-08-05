#!/usr/bin/env python

import rospy, sys
import actionlib
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
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

GRIPPER_FRAME = 'r_gripper_tool_frame'  # 'r_gripper_tool_frame' // 'r_wrist_roll_link'

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

        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # Create a publisher for displaying object frames
        self.object_frames_pub = rospy.Publisher('object_frames', PoseStamped, queue_size=10)

        # Create a dictionary to hold object colors
        self.colors = dict()

        robot = RobotCommander()
        right_arm = MoveGroupCommander("right_arm")


        self.ac = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',pr2c.Pr2GripperCommandAction)
        self.ac.wait_for_server()
        

        rf = robot.get_planning_frame()
        print rf
        open = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(0.087, 100))
        close = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(0.03, -1))
        self.ac.send_goal(open)
        self.ac.wait_for_result()
        #print self.ac.get_result()


        # Prepare Gazebo Subscriber
        self.pwh = None
        self.bl = ['ground_plane','pr2']
        self.pa = []
        self.idx = []
        self.gazebo_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)

        # Give the scene a chance to catch up
        rospy.sleep(2)



       	while self.pwh is None:
            rospy.sleep(0.05)

        #self.new = [ x for x in self.pwh.name if x not in self.bl ]
        #for i in self.new: 
            #self.idx.append(self.pwh.name.index(i))

        #for j in self.idx:
            #self.pa.append(self.pwh.pose[j])
        ##print self.pa

        #self.pose_array = PoseArray()
        #self.pose_array.header.frame_id = REFERENCE_FRAME 
        #self.pose_array.poses = self.pa


        # Set the target size [l, w, h]
        target_size = [0.05, 0.05, 0.05]
        table_size = [1.5, 0.8, 0.03]
        #obstacle1_size = [0.1, 0.025, 0.01]

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
            
        ## Set the target pose on the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose = self.pwh.pose[self.taid]
        target_pose.pose.position.z += 0.019
        # Add the target object to the scene
        scene.add_box(target_id, target_pose, target_size)
        
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose = self.pwh.pose[self.tid]
        table_pose.pose.position.z += 0.995
        scene.add_box(table_id, table_pose, table_size)
        
        #obstacle1_pose = PoseStamped()
        #obstacle1_pose.header.frame_id = REFERENCE_FRAME
        #obstacle1_pose.pose = self.pwh.pose[self.o1id]
        ## Add the target object to the scene
        #scene.add_box(obstacle1_id, obstacle1_pose, obstacle1_size)
        

                
        ### Make the target purple ###
        self.setColor(target_id, 0.6, 0, 1, 1.0)

        # Send the colors to the planning scene
        self.sendColors()

        #Publish target frame
        #self.object_frames_pub.publish(target_pose)
        
        # pick an object
        target_pose.pose.position.z += 0.15
        robot.right_arm.pick(target_id)


        self.ac.send_goal(close)
        self.ac.wait_for_result()
        #print self.ac.get_result()
        rospy.sleep(2)



    #Get pose from Gazebo
    def model_state_callback(self,msg):

        self.pwh = ModelStates()
        self.pwh = msg


        #self.pwh.pose = msg.pose[self.new]

        #find the index of the target
        #self.idx = msg.name.index('wood_cube_5cm')
        #self.idx_targ = msg.name.index('tube_2_25cm_0')
        #with the index get the pose from the target objext
        #self.pwh.header.frame_id = 'base_footprint'
        #self.pwh.pose = msg.pose[self.idx_targ]



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
