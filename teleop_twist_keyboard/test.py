#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool

def cb(data):
    rospy.loginfo(data)

def listener():
    rospy.init_node('listener_for_bool', anonymous=True)
    rospy.Subscriber("go_switch", Bool, cb)
    rospy.spin()

if __name__=="__main__":

    listener()
