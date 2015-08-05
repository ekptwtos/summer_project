#!/usr/bin/env python  
import roslib
import rospy

from geometry_msgs.msg import PoseStamped, Pose
from gazebo_msgs.msg import ModelStates

import numpy as np
import tf


if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (posit,orient) = listener.lookupTransform('world', 'odom_combined', rospy.Time.now())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        

        print posit
        print orient
        rate.sleep()




##    rospy.wait_for_service('spawn')
##    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
##    spawner(4, 2, 0, 'turtle2')

#    world_frame = rospy.Publisher('world frame', Pose,queue_size=1)

#    rate = rospy.Rate(10.0)
#    while not rospy.is_shutdown():
#        try:
#            (posit,orient) = listener.lookupTransform('world', 'odom_combined', rospy.Time(0))
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            continue

#        angular = 4 * math.atan2(trans[1], trans[0])
#        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
#        cmd = PoseStamped()
#        cmd.linear.x = linear
#        cmd.angular.z = angular
#        world_frame.publish(cmd)

#        rate.sleep()
