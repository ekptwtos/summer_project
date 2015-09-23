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
from summer_project.msg import CGrasp as cgrasp
from itertools import izip
import xml.etree.ElementTree as ET


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
        self.left_arm = MoveGroupCommander('left_arm')

        # Initialize the MoveIt! commander for the gripper
        self.right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        self.left_gripper = MoveGroupCommander('left_gripper')

#        eel = len(self.right_arm.get_end_effector_link())
#        print eel
        # Allow 5 seconds per planning attempt
#        self.right_arm.set_planning_time(5)

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

        ### Attach / Remove Object Flag ###
        self.aro = None

        # Run and keep in the BG the scene generator with ctrl^c kill ### 
        timerThread = threading.Thread(target=self.scene_generator)
        timerThread.daemon = True
        timerThread.start()
        ## Give some time to ensure the thread starts!! ##
        rospy.sleep(5)



        ### GENERATE THE BLACKLIST AND REMOVE ATTACHED OBJECTS FROM PREVIOUS RUNS ###
        self.idx_list = self.bl()

        ### GIVE SCENE TIME TO CATCH UP ###
        rospy.sleep(5)


        ################################## GRASP EXECUTION #####################################
        print "==================== Executing ==========================="


        start_time = time.time()


        ### PERSONAL REMINDER!!! WHAT IS WHAT!!! ###
#        print obj_id[obj_id.index('target')]
#        print obj_id.index('target')


        ### MOVE LEFT ARM OUT OF THE WAY ###
        self.lasp()


        success = False
        while success == False and len(self.idx_list)>0:



            success, pgr_target = self.grasp_attempt()
            print ("GA Returns:", success)
            if success is not False:
                self.flag = 0 # To let the planning scene know when to remove the object
                self.post_grasp(pgr_target, obj_id.index('target'),'true')
                self.place_object(obj_id.index('target'))
                break

            else:
                idx = self.idx_list[0]
                ds, pgr_col_obj = self.declutter_scene(idx)
                print ("DS Returns:", ds)

                if ds == True:
                    self.flag = 0 # To let the planning scene know when to remove the object
                    self.post_grasp(pgr_col_obj, obj_id.index(obj_id[idx]),'true')
                    self.place_object(obj_id.index(obj_id[idx]))


                self.idx_list.pop(0) 


        print "==================== THE END! ======================"
        print("--- %s seconds ---" % (time.time() - start_time))
        rospy.sleep(5)

#        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()

#        # Exit the script
        moveit_commander.os._exit(0)



################################################################# FUNCTIONS #################################################################################


    def grasp_attempt(self):

#        start_time = time.time()

        retreat = None
        init_poses = []
        grasp_poses = []
        for axis in range(0,6):
#            while obj_id[obj_id.index('target')] is not 'target':
#                print '!!!!!'
#                rospy.sleep(0.05)
            pg = self.grasp_pose(obj_pose[obj_id.index('target')], axis, 'pg')
            gp = self.grasp_pose(obj_pose[obj_id.index('target')], axis, 'gp')
            init_poses.append(pg)
            grasp_poses.append(gp)

        pre_grasps = self.grasp_generator(init_poses)
        grasps = self.grasp_generator(grasp_poses)
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp)
            rospy.sleep(0.05)

        success = False
        i = 1
        for pg, gr in izip(pre_grasps, grasps):
            self.gripper_pose_pub.publish(gr)
            print ("G Attempt: ", i)
            plp = self.right_arm.plan(pg.pose)
            if len(plp.joint_trajectory.points) == 0:
                print "No valid pregrasp Position, continuing on next one"
                i+=1
                continue

            i+=1
            self.right_arm.plan(pg.pose)
            self.right_arm.go(wait=True)
            rospy.sleep(5)

            plg = self.right_arm.plan(gr.pose)
            if len(plg.joint_trajectory.points) >= 10:
                self.right_arm.go()
                success = True
                retreat = gr
                print "Grasping"
                break

#        print("--- %s seconds ---" % (time.time() - start_time))
        return success , retreat


    def declutter_scene(self,index):

        retreat = None
        init_poses = []
        grasp_poses = []
        for axis in range(0,6):
            pg = self.grasp_pose(obj_pose[index], axis, 'pg')
            gp = self.grasp_pose(obj_pose[index], axis, 'gp')
            init_poses.append(pg)
            grasp_poses.append(gp)

        pre_grasps = self.grasp_generator(init_poses)
        grasps = self.grasp_generator(grasp_poses)
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp)
            rospy.sleep(0.05)

        success = False
        i= 1
        for pg, gr in izip(pre_grasps, grasps):

            plp = self.right_arm.plan(pg.pose)
            print (" DS Attempt: ", i)
            self.gripper_pose_pub.publish(gr)
            self.right_arm.plan(pg.pose)
            if len(plp.joint_trajectory.points) == 0:
                print "No valid pregrasp Position, continuing on next one"
                i+=1
                continue

            i+=1
            self.right_arm.plan(pg.pose)
            self.right_arm.go()
            rospy.sleep(5)

            plg = self.right_arm.plan(gr.pose)
            if len(plg.joint_trajectory.points) >= 10:
                self.right_arm.go()
                print "Grasping"
                success = True
                retreat = gr
                break
        return success, retreat

    def place_object(self, obj_idx):


        self.aro = obj_idx

        ### GENERATE PLACE POSES ###
        places = self.place_generator()

        ### TRY THESE POSES ###
        i = 1
        for place in places:
            print (" Place Attempt: ", i)
            plpl = self.right_arm.plan(place.pose)
            print len(plpl.joint_trajectory.points)
            if len(plpl.joint_trajectory.points) == 0:
                i+=1
                continue
            
            self.right_arm.plan(plpl)
            self.right_arm.go(wait=True)



            ### INFORM SCENE ###
#            self.open_gripper()
#            self.aro = None

            ### RETURN HAND TO STARTING POSITION ###
            self.post_grasp(place,obj_idx, 'false')
            self.rasp()

            break



    def post_grasp(self,new_pose, obj_idx, fl):

        ######### GRASP OBJECT/ REMOVE FROM SCENE ######.

        if fl == 'true':
            self.close_gripper()
            self.aro = obj_idx
        else:
            self.open_gripper()
            self.aro = None
        rospy.sleep(2)

        ### POST GRASP RETREAT ###
        M1 = transformations.quaternion_matrix([new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w])
        M1[0,3] = new_pose.pose.position.x
        M1[1,3] = new_pose.pose.position.y 
        M1[2,3] = new_pose.pose.position.z

        M2 = transformations.euler_matrix(0, 0, 0)

        M2[0,3] = 0.0  # offset about x
        M2[1,3] = 0.0   # about y
        M2[2,3] = 0.25 # about z


        T1 = np.dot(M2, M1)
        npo = deepcopy(new_pose)
        npo.pose.position.x = T1[0,3] 
        npo.pose.position.y = T1[1,3]
        npo.pose.position.z = T1[2,3]

        quat = transformations.quaternion_from_matrix(T1)
        npo.pose.orientation.x = quat[0]
        npo.pose.orientation.y = quat[1]
        npo.pose.orientation.z = quat[2]
        npo.pose.orientation.w = quat[3]
        npo.header.frame_id = REFERENCE_FRAME
        self.right_arm.plan(npo.pose) 
        self.right_arm.go(wait=True)



    def place_generator(self):

        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = 0.57
        place_pose.pose.position.y = 0.16
        place_pose.pose.position.z = 0.56
        place_pose.pose.orientation.w = 1.0

        P = transformations.quaternion_matrix([place_pose.pose.orientation.x, place_pose.pose.orientation.y, place_pose.pose.orientation.z, place_pose.pose.orientation.w])
        P[0,3] = place_pose.pose.position.x
        P[1,3] = place_pose.pose.position.y 
        P[2,3] = place_pose.pose.position.z

        places =[]
        yaw_angles = [0, 1,57, -1,57 , 3,14]
        x_vals = [0, 0.05 ,0.1 , 0.15]
        z_vals = [0.05 ,0.1 , 0.15]
        for y in yaw_angles:
            G = transformations.euler_matrix(0, 0, y)

            G[0,3] = 0.0  # offset about x
            G[1,3] = 0.0   # about y
            G[2,3] = 0.0 # about z

            for z in z_vals:
                for x in x_vals:

                    TM = np.dot(P,  G)
                    pl = deepcopy(place_pose)
                    pl.pose.position.x = TM[0,3] +x
                    pl.pose.position.y = TM[1,3]
                    pl.pose.position.z = TM[2,3] +z

                    quat = transformations.quaternion_from_matrix(TM)
                    pl.pose.orientation.x = quat[0]
                    pl.pose.orientation.y = quat[1]
                    pl.pose.orientation.z = quat[2]
                    pl.pose.orientation.w = quat[3]
                    pl.header.frame_id = REFERENCE_FRAME
                    places.append(deepcopy(pl))

        return places


    def grasp_pose(self, target_pose, axis, stage):

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
                M2[0,3] = -0.18  # offset about x
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
                M2[1,3] = -0.18  # about y
            M2[2,3] = 0.0 # about z

        elif axis == 2:
            M1 = transformations.quaternion_matrix([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
            M1[0,3] = target_pose.pose.position.x
            M1[1,3] = target_pose.pose.position.y 
            M1[2,3] = target_pose.pose.position.z

            M2 = transformations.euler_matrix(0, 0, -1.57)
            M2[0,3] = 0.0  # offset about x
            if stage == 'pg':
                M2[1,3] = 0.25  # about y
            elif stage == 'gp':
                M2[1,3] = 0.18  # about y
            M2[2,3] = 0.0 # about z

        elif axis == 3:
            M1 = transformations.quaternion_matrix([target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w])
            M1[0,3] = target_pose.pose.position.x
            M1[1,3] = target_pose.pose.position.y 
            M1[2,3] = target_pose.pose.position.z

            M2 = transformations.euler_matrix(0, 0, 3.14)
            if stage == 'pg':
                M2[0,3] = 0.25  # offset about x
            elif stage == 'gp':
                M2[0,3] = 0.18  # offset about x
            M2[1,3] = 0.0  # about y
            M2[2,3] = 0.0 # about z

        elif axis == 4:
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
                M2[2,3] = 0.23 # about z

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
                M2[2,3] = 0.23 # about z



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
        while k <= 5:

            O.append(transformations.quaternion_matrix([o[k].pose.orientation.x, o[k].pose.orientation.y, o[k].pose.orientation.z, o[k].pose.orientation.w]))
            O[k][0,3] = o[k].pose.position.x
            O[k][1,3] = o[k].pose.position.y 
            O[k][2,3] = o[k].pose.position.z

            if k in range(0,4):
                for z in self.drange(0.005-obj_size[obj_id.index('target')][2]/2,-0.005 + obj_size[obj_id.index('target')][2]/2, 0.02):  ### TODO: USE EACH OBJECTS SIZE NOT ONLY THE TARGETS ###
#                    print z

                    T = np.dot(O[k], G)

                    grasp = deepcopy(o[k])

                    grasp.pose.position.x = T[0,3]
                    grasp.pose.position.y = T[1,3]
                    grasp.pose.position.z = T[2,3] +z

                    quat = transformations.quaternion_from_matrix(T)
                    grasp.pose.orientation.x = quat[0]
                    grasp.pose.orientation.y = quat[1]
                    grasp.pose.orientation.z = quat[2]
                    grasp.pose.orientation.w = quat[3]
                    grasp.header.frame_id = REFERENCE_FRAME

                    # Append the grasp to the list
                    grasps.append(deepcopy(grasp))


            elif k == 4:
                for x in self.drange(-obj_size[obj_id.index('target')][1]/2, obj_size[obj_id.index('target')][1]/2, 0.02):
#                    print z

                    T = np.dot(O[k], G)

                    grasp = deepcopy(o[k])

                    grasp.pose.position.x = T[0,3] +x
                    grasp.pose.position.y = T[1,3]
                    grasp.pose.position.z = T[2,3] 

                    quat = transformations.quaternion_from_matrix(T)
                    grasp.pose.orientation.x = quat[0]
                    grasp.pose.orientation.y = quat[1]
                    grasp.pose.orientation.z = quat[2]
                    grasp.pose.orientation.w = quat[3]
                    grasp.header.frame_id = REFERENCE_FRAME

                    # Append the grasp to the list
                    grasps.append(deepcopy(grasp))
            else:
                for y in self.drange(-obj_size[obj_id.index('target')][1]/2, obj_size[obj_id.index('target')][1]/2, 0.02):
#                    print z

                    T = np.dot(O[k], G)

                    grasp = deepcopy(o[k])

                    grasp.pose.position.x = T[0,3] 
                    grasp.pose.position.y = T[1,3] +y
                    grasp.pose.position.z = T[2,3] 

                    quat = transformations.quaternion_from_matrix(T)
                    grasp.pose.orientation.x = quat[0]
                    grasp.pose.orientation.y = quat[1]
                    grasp.pose.orientation.z = quat[2]
                    grasp.pose.orientation.w = quat[3]
                    grasp.header.frame_id = REFERENCE_FRAME

                    # Append the grasp to the list
                    grasps.append(deepcopy(grasp))
            k+=1
        print len(grasps)
        # Return the list
        return grasps



    def scene_generator(self):
        while True:
#            print "happening"
            obj_pose =[]
            obj_id = []
            obj_size = []
            bl = ['ground_plane','pr2'] 
            global obj_pose, obj_id , obj_size

            ops = PoseStamped()
            ops.header.frame_id = REFERENCE_FRAME


            for model_name in self.pwh.name:
                if model_name not in bl:
                    obj_id.append(model_name)
                    ops.pose = self.pwh.pose[self.pwh.name.index(model_name)]
                    obj_pose.append(deepcopy(ops))
                    obj_size.append([0.05, 0.05, 0.15])


#            obj_id[obj_id.index('custom_1')] = 'target'
            obj_size[obj_id.index('custom_2')] = [0.05, 0.05, 0.10]
            obj_size[obj_id.index('custom_3')] = [0.05, 0.05, 0.05]
            obj_size[obj_id.index('custom_table')] = [1.5, 0.8, 0.03]



            if self.aro is None:
                for i in range(0, len(obj_id)):
                    ### CREATE THE SCENE ###
                    self.scene.add_box(obj_id[i], obj_pose[i], obj_size[i])
                    self.setColor(obj_id[i], 1, 0.623, 0, 1.0)

                ### Make the target purple and table green ###
                self.setColor(obj_id[obj_id.index('target')], 0.6, 0, 1, 1.0)
                self.setColor(obj_id[obj_id.index('custom_table')], 0.3, 1, 0.3, 1.0)

                self.scene.remove_attached_object(GRIPPER_FRAME)


                # Send the colors to the planning scene
                self.sendColors()

            else:
                if self.flag == 0:
                    touch_links = [GRIPPER_FRAME, 'r_gripper_l_finger_tip_link','r_gripper_r_finger_tip_link', 'r_gripper_r_finger_link', 'r_gripper_l_finger_link', 'r_wrist_roll_link', 'r_upper_arm_link']
                    #print touch_links
                    self.scene.attach_box(GRIPPER_FRAME, obj_id[self.aro], obj_pose[self.aro], obj_size[self.aro], touch_links)

                    ### REMOVE SPECIFIC OBJECT AFTER IT HAS BEEN ATTACHED TO GRIPPER ###
                    self.scene.remove_world_object(obj_id[self.aro])
                    self.flag +=1 

            time.sleep(0.5)




    def model_state_callback(self,msg):

        self.pwh = ModelStates()
        self.pwh = msg

    def bl(self):
        blist = ['target','custom_2','custom_3', 'custom_table'] 
        self.blist = []
        for name in obj_id:
            if name not in blist:
                self.blist.append(obj_id.index(name))
                # Remove any attached objects from a previous session
                self.scene.remove_attached_object(GRIPPER_FRAME, obj_id[obj_id.index(name)])

        self.scene.remove_attached_object(GRIPPER_FRAME, 'target')
        return self.blist

    def drange(self, start, stop, step):
        r = start
        while r < stop:
            yield r
            r += step

    def close_gripper(self):

        g_close = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(0.044, 100))
        self.ac.send_goal(g_close)
        self.ac.wait_for_result()
        rospy.sleep(15) # Gazebo requires up to 15 seconds to attach object


    def open_gripper(self):

        g_open = pr2c.Pr2GripperCommandGoal(pr2c.Pr2GripperCommand(0.0899, 100))
        self.ac.send_goal(g_open)
        self.ac.wait_for_result()
        rospy.sleep(5) # And up to 20 to detach it


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

    def lasp(self):

        sp = PoseStamped()
        sp.header.frame_id = REFERENCE_FRAME

        sp.pose.position.x = 0.3665 
        sp.pose.position.y = 0.74094
        sp.pose.position.z = 1.1449
        sp.pose.orientation.x = 0.80503 
        sp.pose.orientation.y =  -0.18319
        sp.pose.orientation.z = 0.31988
        sp.pose.orientation.w =  0.46481

        self.left_arm.plan(sp)
        self.left_arm.go(wait=True)

    def rasp(self):

        sp = PoseStamped()
        sp.header.frame_id = REFERENCE_FRAME

        sp.pose.position.x = 0.39571
        sp.pose.position.y = -0.40201
        sp.pose.position.z = 1.1128
        sp.pose.orientation.x =0.00044829
        sp.pose.orientation.y =  0.57956
        sp.pose.orientation.z = 9.4878e-05
        sp.pose.orientation.w = 0.81493

        self.right_arm.plan(sp)
        self.right_arm.go(wait=True)


if __name__ == "__main__":
    MoveItDemo()

