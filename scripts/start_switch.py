#!/usr/bin/python
import rospy
from std_msgs.msg import Bool

if __name__ == "__main__":
    rospy.init_node('start_switch')
    pub = rospy.Publisher('start_switch', Bool, queue_size=5)

    rospy.sleep(1)
    for i in xrange(5):
        pub.publish(True)

    rospy.sleep(1)
