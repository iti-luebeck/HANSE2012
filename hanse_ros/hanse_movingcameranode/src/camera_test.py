#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_movingcameranode')
import rospy
from std_msgs.msg import Int8

def camera_test():
    pub = rospy.Publisher('hanse_camera_pos', Int8)
    rospy.init_node('camera_test')
    if not rospy.is_shutdown():
        for i in [-90,0,90]:
            pub.publish(i)
            rospy.sleep(2)         
        for i in range(-90,90):
            pub.publish(i)
            rospy.sleep(0.01)
        pub.publish(0)

if __name__ == '__main__':
    try:
        camera_test()
    except rospy.ROSInterruptException:
        pass