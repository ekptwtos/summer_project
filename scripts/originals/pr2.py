#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


from std_msgs.msg import String

def move_group_python_interface_tutorial():

  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  arm = moveit_commander.MoveGroupCommander("right_arm")
  gripper = moveit_commander.MoveGroupCommander("right_gripper")
  eef = arm.get_end_effector_link()


  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=10)

  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Arm rf: %s" % arm.get_planning_frame()

  ## We can get the name of the reference frame for this robot
  print "============ Gripper rf: %s" % gripper.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % arm.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()


  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
#  print "============ Generating plan 1"
#  pose_target = geometry_msgs.msg.Pose()
#  pose_target.position.x = 0.076737
#  pose_target.position.y = -0.88595
#  pose_target.position.z =  0.66625
#  pose_target.orientation.x = 0.00045992
#  pose_target.orientation.y = -0.37246
#  pose_target.orientation.z = 9.7196e-06
#  pose_target.orientation.w = 0.92805
  
  pose_target = geometry_msgs.msg.Pose()
  pose_target.position.x = -0.6965150003499676
  pose_target.position.y =  0.4836979971883166
  pose_target.position.z =  -0.45562500220443225
  pose_target.orientation.x =-3.0855171310443128e-09
  pose_target.orientation.y = 3.8400780663349104e-10
  pose_target.orientation.z =  2.1227078899034756e-14
  pose_target.orientation.w =  1.0


  arm.set_pose_target(pose_target,eef)
  plan = arm.plan()
  rospy.sleep(2)
  arm.go(wait=True)

  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(5)

  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()





  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass


