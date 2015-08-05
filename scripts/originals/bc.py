#!/usr/bin/env python  
import roslib
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.msg import ModelStates
from moveit_msgs.msg import PlanningScene

import numpy as np
import tf

class Demo:
    def __init__(self):
        rospy.init_node('turtle_tf_broadcaster')
        self.idx_targ = None
        self.pwh = None
        self.gazebo_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_callback)

        while self.pwh is None:
            rospy.sleep(0.05)

        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            br.sendTransform((self.pwh.pose.position.x, self.pwh.pose.position.y, self.pwh.pose.position.z),
                             (self.pwh.pose.orientation.x, self.pwh.pose.orientation.y, self.pwh.pose.orientation.z, self.pwh.pose.orientation.w),
                             rospy.Time.now(),
                             "odom_combined",
                             "gazebo_world")
            rate.sleep()



#        frame = "odom_combined"
#        rospy.Subscriber('/%s/pose' % frame,
#                         ModelStates.pose,
#                         self.handle_gazebo_pose,
#                         frame)


    def model_state_callback(self,msg):

        #find the index of the target
        self.idx_targ = msg.name.index('wood_cube_5cm')
        self.pwh = PoseStamped()
        self.pwh.header.frame_id = "world"
        #with the index get the pose from the target objext
        self.pwh.pose = msg.pose[self.idx_targ]

#    def handle_gazebo_pose(self,msg):
#        br = tf.TransformBroadcaster()
#        br.sendTransform((msg.x, msg.y, 0),
#                         tf.transformations.quaternion_from_euler(0, 0, msg.theta),
#                         rospy.Time.now(),
#                         frame,
#                         "world")

if __name__ == "__main__":
    Demo()

