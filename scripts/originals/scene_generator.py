#!/usr/bin/env python

import rospy, sys
import actionlib
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import PlanningScene, ObjectColor , AttachedCollisionObject , MoveItErrorCodes ,MotionPlanRequest
from moveit_msgs.msg import Grasp, GripperTranslation
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy
from gazebo_msgs.msg import ModelStates
from tf import transformations
import numpy as np
import  pr2_controllers_msgs.msg as pr2c 
import time, threading
from shape_msgs.msg import SolidPrimitive




class scene_generator:
    def __init__(self):



        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('scene_generator')

        # Use the planning scene object to add or remove objects
        self.scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # Create a dictionary to hold object colors
        self.colors = dict()


        # Prepare Gazebo Subscriber
        self.pwh = None
        self.pwh_copy = None
        self.idx_targ = None
        self.gazebo_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)


        # PREPARE THE SCENE
        while self.pwh is None:
            rospy.sleep(0.05)

        ############## CLEAR THE SCENE ################



        # Run and keep in the BG the scene generator also add the ability to kill the code with ctrl^c
        timerThread = threading.Thread(target=self.scene_generator)
        timerThread.daemon = True
        timerThread.start()



################################################################# FUNCTIONS #################################################################################



    def model_state_callback(self,msg):

        self.pwh = ModelStates()
        self.pwh = msg


    def scene_generator(self):
#        print obj_att
        global target_pose
        global target_id
        global target_size

        next_call = time.time()
        while True:
            next_call = next_call+1
            target_id = 'target'
            self.taid = self.pwh.name.index('custom_1')
            table_id = 'table'
            self.tid = self.pwh.name.index('table') 
            obstacle1_id = 'obstacle1'
            self.o1id = self.pwh.name.index('wood_cube_5cm')
            obstacle2_id = 'obstacle2'
            self.o2id = self.pwh.name.index('custom_2')

            # Set the sizes [l, w, h]
            table_size = [1.5, 0.8, 0.03]
            target_size = [0.05, 0.05, 0.15]
            obstacle1_size = [0.05, 0.05, 0.05]
            obstacle2_size = [0.05, 0.05, 0.10]

            ## Set the target pose on the table
            target_pose = PoseStamped()
            target_pose.header.frame_id = REFERENCE_FRAME
            target_pose.pose = self.pwh.pose[self.taid]
            if self.obj_att is None:
                # Add the target object to the scene
                self.scene.add_box(target_id, target_pose, target_size)

                table_pose = PoseStamped()
                table_pose.header.frame_id = REFERENCE_FRAME
                table_pose.pose = self.pwh.pose[self.tid]
                table_pose.pose.position.z += 1
                self.scene.add_box(table_id, table_pose, table_size)
                
                obstacle1_pose = PoseStamped()
                obstacle1_pose.header.frame_id = REFERENCE_FRAME
                obstacle1_pose.pose = self.pwh.pose[self.o1id]
                obstacle1_pose.pose.position.z += 0.025
                # Add the target object to the scene 
                self.scene.add_box(obstacle1_id, obstacle1_pose, obstacle1_size)

                obstacle2_pose = PoseStamped()
                obstacle2_pose.header.frame_id = REFERENCE_FRAME
                obstacle2_pose.pose = self.pwh.pose[self.o2id]
                # Add the target object to the scene 
                self.scene.add_box(obstacle2_id, obstacle2_pose, obstacle2_size)

                ### Make the target purple ###
                self.setColor(target_id, 0.6, 0, 1, 1.0)

                # Send the colors to the planning scene
                self.sendColors()
            else: 
                self.scene.remove_world_object('target')

            time.sleep(next_call - time.time())
        #threading.Timer(0.5, self.scene_generator).start()

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
    scene_generator()

