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
from summer_project.msg import CGrasp 



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
#        eel = len(self.right_arm.get_end_effector_link())
#        print eel
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

#        self.m_error = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)


        ### OPEN THE GRIPPER ###
        self.open_gripper()

        self.obj_att = None


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



        print "==================== Generating Transformations ==========================="

        #################### PRE GRASPING / GRASPING POSES #########################
        test = []
        init_poses = []
        grasp_poses = []
        cgrasp = CGrasp
        for axis in range(0,5):
            pg = self.grasp_pose(target_pose, axis, 'pg')
            gp = self.grasp_pose(target_pose, axis, 'gp')
            cgrasp.grasp_poses = self.grasp_pose(target_pose, axis, 'pg')
            print cgrasp.grasp_poses
            print axis
            if axis == 0:
                print "zero"
                cgrasp.id = 'front'
                print cgrasp.id
            elif axis == 1:
                print "one"
                cgrasp.id = 'right'
            elif axis == 2:
                print "two"
                cgrasp.id = 'left'
            elif axis == 3:
                print "three"
                cgrasp.id = 'topx'
            else:
                print "four"
                cgrasp.id = 'topy'
            init_poses.append(pg)
            grasp_poses.append(gp)
            test.append(cgrasp)

        ################################## TESTING AREA #####################################
        print test[0].id

#        for i in range(0,5):
#            print init_poses[i]
#            print "=================================="
#            print (test[i].id , test[i].grasp_poses)



#        test =[]
#        test2 = []
#        cgrasp.id = "x"
#        cgrasp.grasp_pose = init_poses[0]
#        print init_poses[0]
#        print cgrasp.grasp_pose
#        test.append(cgrasp)
#        print test[0].grasp_pose
#        cgrasp.grasp_pose = init_poses[1]
#        test.append(cgrasp)
#        print init_poses[1]
#        print cgrasp.grasp_pose
#        print test[1].grasp_pose
#        cgrasp.id = "x"
#        cgrasp.grasp_pose = init_poses[1]
#        test.append(cgrasp)

##        print "============== test 1 =================="
##        print test[0].grasp_pose
##        print test[1].grasp_pose

#        for i in range(0,3):
#            cgrasp.grasp_pose = init_poses[i]
#            cgrasp.id = "x"
#            test2.append(cgrasp)
#        t = len(test)
#        for j in range(0,t):
#            print j
#            print "============== test 2 =================="
#            print (test2[j].id, test2[j].grasp_pose)
#            print "INTI POSES==================="
#            print (init_poses[j])




        ################# GENERATE GRASPS ###################


#        grasps = self.grasp_generator(init_poses)


#        for grasp in grasps:
#            print grasp.grasp_pose
##            print grasp.id 
#            self.gripper_pose_pub.publish(grasp.grasp_pose)
#            rospy.sleep(0.2)


#        for grasp in grasps:

#            pl = self.right_arm.plan(grasp)
#            print len(pl.joint_trajectory.points)
#            if len(pl.joint_trajectory.points) >=10:  #!= 0: ## FAILED PLANS HAVE LEN = 0 
#                self.right_arm.clear_pose_target(GRIPPER_FRAME)
#                self.right_arm.set_pose_target(grasp)
#                self.right_arm.shift_pose_target(0, 0.07, GRIPPER_FRAME)
#                if self.right_arm.go() == True:
#                    print "executing grasp"
#                    self.right_arm.go()
#                    break




#        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

#        # Exit the script
        moveit_commander.os._exit(0)



################################################################# FUNCTIONS #################################################################################



    def model_state_callback(self,msg):

        self.pwh = ModelStates()
        self.pwh = msg

    def grasp_pose(self, target_pose, axis, stage ):

        ############ TODO : GENERATE AUTOMATED PRE-GRASPING POSITIONS BASED ON THE PRIMITIVE #########

        if axis == 0:
            M1 = transformations.quaternion_matrix([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
            M1[0,3] = target_pose.pose.position.x
            M1[1,3] = target_pose.pose.position.y 
            M1[2,3] = target_pose.pose.position.z

            M2 = transformations.euler_matrix(0, 0, 0)
            if stage == 'pg':
                M2[0,3] = -0.25  # offset about x
            elif stage == 'gp':
                M2[0,3] = -0.07  # offset about x
            M2[1,3] = 0.0   # about y
            M2[2,3] = 0.0 # about z


        elif axis == 1:
            M1 = transformations.quaternion_matrix([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
            M1[0,3] = target_pose.pose.position.x
            M1[1,3] = target_pose.pose.position.y 
            M1[2,3] = target_pose.pose.position.z

            M2 = transformations.euler_matrix(0, 0, 1.57)
            M2[0,3] = 0.0  # offset about x
            if stage == 'pg':
                M2[1,3] = -0.25  # about y
            elif stage == 'gp':
                M2[1,3] = -0.07  # about y
            M2[2,3] = 0.0 # about z

        elif axis ==2:
            M1 = transformations.quaternion_matrix([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
            M1[0,3] = target_pose.pose.position.x
            M1[1,3] = target_pose.pose.position.y 
            M1[2,3] = target_pose.pose.position.z

            M2 = transformations.euler_matrix(0, 0, -1.57)
            M2[0,3] = 0.0  # offset about x
            if stage == 'pg':
                M2[1,3] = 0.25  # about y
            elif stage == 'gp':
                M2[1,3] = 0.07  # about y
            M2[2,3] = 0.0 # about z

        elif axis ==3:
            M1 = transformations.quaternion_matrix([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
            M1[0,3] = target_pose.pose.position.x
            M1[1,3] = target_pose.pose.position.y 
            M1[2,3] = target_pose.pose.position.z

            M2 = transformations.euler_matrix(0, 1.57, 0)
            M2[0,3] = 0.0  # offset about x
            M2[1,3] = 0.0  # about y
            if stage == 'pg':
                M2[2,3] = 0.30 # about z
            elif stage == 'gp':
                M2[2,3] = 0.10 # about z

        else:
            M1 = transformations.quaternion_matrix([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
            M1[0,3] = target_pose.pose.position.x
            M1[1,3] = target_pose.pose.position.y 
            M1[2,3] = target_pose.pose.position.z

            M2 = transformations.euler_matrix(1.57, 1.57, 0)
            M2[0,3] = 0.0  # offset about x
            M2[1,3] = 0.0  # about y
            if stage == 'pg':
                M2[2,3] = 0.30 # about z
            elif stage == 'gp':
                M2[2,3] = 0.10 # about z

        T1 = np.dot(M1,  M2)
        grasp_pose = deepcopy(target_pose)
        grasp_pose.pose.position.x = T1[0,3] 
        grasp_pose.pose.position.y = T1[1,3]
        grasp_pose.pose.position.z = T1[2,3]

        quat = transformations.quaternion_from_matrix(T1)
        grasp_pose.pose.orientation.x = quat[0]
        grasp_pose.pose.orientation.y = quat[1]
        grasp_pose.pose.orientation.z = quat[2]
        grasp_pose.pose.orientation.w = quat[3]
        grasp_pose.header.frame_id = REFERENCE_FRAME 

        return grasp_pose


    def grasp_generator(self, initial_poses):

        # A list to hold the grasps
        grasps = []
        o = []        # Original Pose of the object (o)
        O=[]


        i= 0
        while i < len(initial_poses):
            o.append(initial_poses[i])
            i+=1

        G = transformations.euler_matrix(0, 0, 0)


        # Generate a grasps for along z axis (x and y directions)

        k = 0
        while k <= 4:

            O.append(transformations.quaternion_matrix([o[k].pose.orientation.x, o[k].pose.orientation.y, o[k].pose.orientation.z, o[k].pose.orientation.w]))
            O[k][0,3] = o[k].pose.position.x
            O[k][1,3] = o[k].pose.position.y 
            O[k][2,3] = o[k].pose.position.z

            if k in range(0,3):
                for z in self.drange(-target_size[2]/2, target_size[2]/2, 0.02):

                    if k == 0:
                        cgrasp.id = 'front'
                    elif k ==1:
                        cgrasp.id = 'right'
                    else:
                        cgrasp.id = 'left'

                    T = np.dot(O[k], G)
                    cgrasp.grasp_pose = deepcopy(o[k])

                    cgrasp.grasp_pose.pose.position.x = T[0,3]
                    cgrasp.grasp_pose.pose.position.y = T[1,3]
                    cgrasp.grasp_pose.pose.position.z = T[2,3] +z

                    quat = transformations.quaternion_from_matrix(T)
                    cgrasp.grasp_pose.pose.orientation.x = quat[0]
                    cgrasp.grasp_pose.pose.orientation.y = quat[1]
                    cgrasp.grasp_pose.pose.orientation.z = quat[2]
                    cgrasp.grasp_pose.pose.orientation.w = quat[3]
                    cgrasp.grasp_pose.header.frame_id = REFERENCE_FRAME

                    # Append the grasp to the list
                    grasps.append(deepcopy(cgrasp))

            elif k == 3:
                for x in self.drange(-target_size[1]/2, target_size[1]/2, 0.01):
#                    print z
                    cgrasp.id = 'topx'
                    T = np.dot(O[k], G)

                    cgrasp.grasp_pose = deepcopy(o[k])

                    cgrasp.grasp_pose.pose.position.x = T[0,3] +x
                    cgrasp.grasp_pose.pose.position.y = T[1,3]
                    cgrasp.grasp_pose.pose.position.z = T[2,3] 

                    quat = transformations.quaternion_from_matrix(T)
                    cgrasp.grasp_pose.pose.orientation.x = quat[0]
                    cgrasp.grasp_pose.pose.orientation.y = quat[1]
                    cgrasp.grasp_pose.pose.orientation.z = quat[2]
                    cgrasp.grasp_pose.pose.orientation.w = quat[3]
                    cgrasp.grasp_pose.header.frame_id = REFERENCE_FRAME

                    # Append the grasp to the list
                    grasps.append(deepcopy(cgrasp))
            else:
                for y in self.drange(-target_size[1]/2, target_size[1]/2, 0.01):
#                    print z
                    cgrasp.id = 'topy'
                    T = np.dot(O[k], G)

                    grasp = deepcopy(o[k])

                    cgrasp.grasp_pose.pose.position.x = T[0,3] 
                    cgrasp.grasp_pose.pose.position.y = T[1,3] +y
                    cgrasp.grasp_pose.pose.position.z = T[2,3] 

                    quat = transformations.quaternion_from_matrix(T)
                    cgrasp.grasp_pose.pose.orientation.x = quat[0]
                    cgrasp.grasp_pose.pose.orientation.y = quat[1]
                    cgrasp.grasp_pose.pose.orientation.z = quat[2]
                    cgrasp.grasp_pose.pose.orientation.w = quat[3]
                    cgrasp.grasp_pose.header.frame_id = REFERENCE_FRAME

                    # Append the grasp to the list
                    grasps.append(deepcopy(cgrasp))

            k+=1

        # Return the list
        return grasps

    def drange(self, start, stop, step):
        r = start
        while r < stop:
            yield r
            r += step


    def plan_exec(self, pose):

        self.right_arm.clear_pose_targets()
        self.right_arm.set_pose_target(pose, GRIPPER_FRAME)
        self.right_arm.plan()
        rospy.sleep(5)
        self.right_arm.go(wait=True)

#    def grasp_plan(self, pose):

#        # put here the grasping lines



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

        next_call = time.time()
        while True:
            next_call = next_call+1
            target_id = 'target'
            self.taid = self.pwh.name.index('custom_0')
            table_id = 'table'
            self.tid = self.pwh.name.index('table') 
            obstacle1_id = 'obstacle1'
            self.o1id = self.pwh.name.index('custom_1')
            obstacle2_id = 'obstacle2'
            self.o2id = self.pwh.name.index('custom_2')

            # Set the sizes [l, w, h]
            table_size = [1.5, 0.8, 0.03]
            target_size = [0.025, 0.025, 0.15]
            obstacle1_size = [0.05, 0.05, 0.15]
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
                # Add the target object to the scene 
                self.scene.add_box(obstacle1_id, obstacle1_pose, obstacle1_size)

                obstacle2_pose = PoseStamped()
                obstacle2_pose.header.frame_id = REFERENCE_FRAME
                obstacle2_pose.pose = self.pwh.pose[self.o2id]
                # Add the target object to the scene 
                self.scene.add_box(obstacle2_id, obstacle2_pose, obstacle2_size)

                ### Make the target purple and obstacles orange ###
                self.setColor(target_id, 0.6, 0, 1, 1.0)
                self.setColor(obstacle1_id, 1, 0.623, 0, 1.0)
                self.setColor(obstacle2_id, 1, 0.623, 0, 1.0)

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

