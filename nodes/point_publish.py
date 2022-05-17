#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def publish_point(x, y, z):
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('fake_clicker')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        point = PoseStamped()
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = z
        point.pose.orientation.w = 1
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"
        print(point)
        pub.publish(point)
        rate.sleep()
        break

if __name__ == '__main__':
    try:
        publish_point(0, 0, 0)
    except rospy.ROSInternalException:
        pass