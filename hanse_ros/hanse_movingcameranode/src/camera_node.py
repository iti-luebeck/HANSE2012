#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_movingcameranode')
import rospy
from std_msgs.msg import Int8
import serial

def callback(data):
    if data.data > 90:
        data.data = 90
    elif data.data < -90:
        data.data = -90
    data.data = data.data % 256
    ser = serial.Serial('/dev/ttyUSB_atmega', 57600, timeout=0)
    position = int(data.data)
    #rospy.loginfo(rospy.get_name() + ": Sende %d" % position)
    position = chr(position)
    #rospy.loginfo(rospy.get_name() + ": Sende %s" % position)
    send = ser.write(position)
    #rospy.loginfo(rospy.get_name() + ": Gesendet %d Byte" % send)
    ser.close()

def camera_node():
    rospy.init_node('camera', anonymous=False)
    rospy.Subscriber("hanse_camera_pos", Int8, callback)
    rospy.spin()


if __name__ == '__main__':
    camera_node()
