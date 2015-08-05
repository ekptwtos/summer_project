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
import time, threading


GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper' 

GRIPPER_FRAME = 'r_wrist_roll_link'  # 'r_gripper_tool_frame' // 'r_wrist_roll_link'

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
        self.scene = PlanningSceneInterface()

        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=10)
        
        # Create a publisher for displaying object frames
        self.object_frames_pub = rospy.Publisher('object_frames', PoseStamped, queue_size=10)

        # Create a dictionary to hold object colors
        self.colors = dict()

        # Initialize the MoveIt! commander for the arm
        global right_arm
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)

        # Initialize the MoveIt! commander for the gripper
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

        # Get the name of the end-effector link
        eef = right_arm.get_end_effector_link()

        # Allow some leeway in position (meters) and orientation (radians)
#        right_arm.set_goal_position_tolerance(0.05)
#        right_arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)

        # Set the right arm reference frame
        right_arm.set_pose_reference_frame(REFERENCE_FRAME)

        # Allow 5 seconds per planning attempt
        right_arm.set_planning_time(5)

        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 5

        # Set a limit on the number of place attempts
        max_place_attempts = 5

        # Give the scene a chance to catch up
        rospy.sleep(2)
        
        # Prepare Gazebo Subscriber
        self.pwh = None
        self.pwh_copy = None
        self.idx_targ = None
        self.gazebo_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)

        # Prepare Gripper 
        self.ac = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',pr2c.Pr2GripperCommandAction)
        self.ac.wait_for_server()

        rospy.sleep(2)


        ### OPEN THE GRIPPER ###
        self.open_gripper()

        self.obj_att = None


        # PREPARE THE SCENE
        while self.pwh is None:
            rospy.sleep(0.05)

        ############## CLEAR THE SCENE ################

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



        print "==================== Generating Transformations ========================="

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





        #print target_pose
        #self.object_frames_pub.publish(target_pose)
        rospy.sleep(2)

        # Initialize the grasp pose to the target pose
        grasp_pose = target_pose
        grasp_pose.header.frame_id = 'gazebo_world'

        # Shift the grasp pose by half the width of the target to center it

        #grasp_pose.pose.position.y -= target_size[1] / 2.0
        #grasp_pose.pose.position.x = target_pose.pose.position.x / 2.0
#        grasp_pose.pose.position.x = target_pose.pose.position.x -0.07
#        grasp_pose.pose.position.z = target_pose.pose.position.z +0.025

        #Allowed touch object list
#        ato = []
#        ato.append(target_id)
#        ato.append('r_forearm_link')


        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, [target_id]) #### [target_id] , ato
        # Publish the grasp poses so they can be viewed in RViz
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)

        # Track success/failure and number of attempts for pick operation
        success = False
        n_attempts = 0

        # Repeat until we succeed or run out of attempts
        while success == False and n_attempts < max_pick_attempts:
            success = right_arm.pick(target_id, grasps)
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            rospy.sleep(0.2)



        ## If the pick was successful, attempt the place operation   
        #if success:
            #success = False
            #n_attempts = 0

            ## Generate valid place poses
            #places = self.make_places(place_pose)

            ## Repeat until we succeed or run out of attempts
            #while success == False and n_attempts < max_place_attempts:
                #for place in places:
                    #success = right_arm.place(target_id, place)
                    #if success:
                        #break
                #n_attempts += 1
                #rospy.loginfo("Place attempt: " +  str(n_attempts))
                #rospy.sleep(0.2)

            #if not success:
                #rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        #else:
            #rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")

        ## Return the arm to the "resting" pose stored in the SRDF file
        ##right_arm.set_named_target('resting')
        ##right_arm.go()

        ## Open the gripper to the neutral position
        #right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        #right_gripper.go()

        #rospy.sleep(1)

        #rospy.spin()


        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

        # Exit the script
        moveit_commander.os._exit(0)


############################################################### FUNCTIONS ################################################################



    def model_state_callback(self,msg):

        self.pwh = ModelStates()
        self.pwh = msg

    def plan_exec(self, pose):

        right_arm.clear_pose_targets()
        right_arm.set_pose_target(pose, GRIPPER_FRAME)
        right_arm.plan()
        rospy.sleep(5)
        right_arm.go(wait=True)


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

    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        
        t = JointTrajectory()

        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES

        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()

        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions

        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT

        tp.time_from_start = rospy.Duration(1.0)

        # Append the goal point to the trajectory points
        t.points.append(tp)

        return t

    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()

        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]

        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME

        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired

        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()

        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)

        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.05, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])

        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped


        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4, 1.57, -1.57]

        # Yaw angles to try
        yaw_vals = [0, 1.57, -1.57]
        

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            for p in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, p, y)

                # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]

                # Set an id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))

                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects

                # Don't restrict contact force
                g.max_contact_force = 0

                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(p)
                
                # Append the grasp to the list
                grasps.append(deepcopy(g))
     
        # Return the list
        return grasps

    # Generate a list of possible place poses
    #def make_places(self, init_pose):
        ## Initialize the place location as a PoseStamped message
        #place = PoseStamped()

        ## Start with the input place pose
        #place = init_pose

        ## A list of x shifts (meters) to try
        #x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]

        ## A list of y shifts (meters) to try
        #y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]       

        ## A list of pitch angles to try
        ##pitch_vals = [0, 0.005, -0.005, 0.01, -0.01, 0.02, -0.02]

        #pitch_vals = [0]

        ## A list of yaw angles to try
        #yaw_vals = [0]

        ## A list to hold the places
        #places = []

        ## Generate a place pose for each angle and translation
        #for y in yaw_vals:
            #for p in pitch_vals:
                #for y in y_vals:
                    #for x in x_vals:
                        #place.pose.position.x = init_pose.pose.position.x + x
                        #place.pose.position.y = init_pose.pose.position.y + y

                        ## Create a quaternion from the Euler angles
                        #q = quaternion_from_euler(0, p, y)

                        ## Set the place pose orientation accordingly
                        #place.pose.orientation.x = q[0]
                        #place.pose.orientation.y = q[1]
                        #place.pose.orientation.z = q[2]
                        #place.pose.orientation.w = q[3]

                        ## Append this place pose to the list
                        #places.append(deepcopy(place))

        ## Return the list
        #return places


    def scene_generator(self):
#        print obj_att
        global target_pose
        global target_id
        global target_size

        next_call = time.time()
        while True:
            next_call = next_call+1
            target_id = 'target'
            self.taid = self.pwh.name.index('wood_cube_5cm')
            table_id = 'table'
            self.tid = self.pwh.name.index('table') 
#            obstacle1_id = 'obstacle1'
#            self.o1id = self.pwh.name.index('wood_block_10_2_1cm')


            # Set the target size [l, w, h]
            target_size = [0.05, 0.05, 0.05]
            table_size = [1.5, 0.8, 0.03]
#            obstacle1_size = [0.1, 0.025, 0.01]

            ## Set the target pose on the table
            target_pose = PoseStamped()
            target_pose.header.frame_id = REFERENCE_FRAME
            target_pose.pose = self.pwh.pose[self.taid]
            target_pose.pose.position.z += 0.025
            if self.obj_att is None:
                # Add the target object to the scene
                self.scene.add_box(target_id, target_pose, target_size)

                table_pose = PoseStamped()
                table_pose.header.frame_id = REFERENCE_FRAME
                table_pose.pose = self.pwh.pose[self.tid]
                table_pose.pose.position.z += 1
                self.scene.add_box(table_id, table_pose, table_size)
                
#                obstacle1_pose = PoseStamped()
#                obstacle1_pose.header.frame_id = REFERENCE_FRAME
#                obstacle1_pose.pose = self.pwh.pose[self.o1id]
#                # Add the target object to the scene
#                self.scene.add_box(obstacle1_id, obstacle1_pose, obstacle1_size)

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
    MoveItDemo()
