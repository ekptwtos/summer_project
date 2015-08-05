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
        rospy.init_node('tf_broadcaster')
#        self.ps = None
#        self.ps_subscriber = rospy.Subscriber("/planning_scene", PlanningScene, self.ps_callback)

#        while self.ps is None:
#            rospy.sleep(0.05)

#        print self.ps

        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            br.sendTransform((0, 0, 0),
                             (0, 0, 0, 1),
                             rospy.Time.now(),
                             "odom_combined",
                             "wolrd")
            rate.sleep()



#        frame = "odom_combined"
#        rospy.Subscriber('/%s/pose' % frame,
#                         ModelStates.pose,
#                         self.handle_gazebo_pose,
#                         frame)


#    def ps_callback(self,msg):

#        self.ps = PlanningScene()

#    def handle_gazebo_pose(self,msg):
#        br = tf.TransformBroadcaster()
#        br.sendTransform((msg.x, msg.y, 0),
#                         tf.transformations.quaternion_from_euler(0, 0, msg.theta),
#                         rospy.Time.now(),
#                         frame,
#                         "world")

if __name__ == "__main__":
    Demo()

