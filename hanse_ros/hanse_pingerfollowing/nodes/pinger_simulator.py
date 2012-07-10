#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_pingerfollowing')
import rospy
import math
import dynamic_reconfigure.server
from geometry_msgs.msg import PoseStamped
from hanse_msgs.msg import PingerDetection
from std_msgs.msg import Header
from hanse_pingerfollowing.cfg import PingerSimulatorConfig

class Node:
    goalX = 0
    goalY = 0
    lastPingTime = None

class Config:
    maxAmplitude = 4.0
    baseAmplitude = 1.0
    dampingFactor = 1.0
    dampingExponent = 2.0

def configCallback(config, level):
    Config.maxAmplitude = config['max_amplitude']
    Config.baseAmplitude = config['base_amplitude']
    Config.dampingFactor = config['damping_factor']
    Config.dampingExponent = config['damping_exponent']
    return config

def goalCallback(msg):
    Node.goalX, Node.goalY = msg.pose.position.x, msg.pose.position.y

def posemeterCallback(msg):
    if Node.lastPingTime == None:
        Node.lastPingTime = rospy.Time.now()
    if (rospy.Time.now() - Node.lastPingTime).secs < 1:
        return
    Node.lastPingTime += rospy.Duration(1)
    x, y = msg.pose.position.x, msg.pose.position.y
    dx, dy = Node.goalX - x, Node.goalY - y
    dist = math.hypot(dx, dy)
    msg = PingerDetection()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    volume = min(Config.maxAmplitude, Config.baseAmplitude / math.pow(
        max(0.0001, Config.dampingFactor * dist), Config.dampingExponent))
    msg.leftAmplitude = volume
    msg.rightAmplitude = volume
    Node.publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pingersim')
    dynamic_reconfigure.server.Server(PingerSimulatorConfig, configCallback)
    rospy.Subscriber('/hanse/posemeter', PoseStamped, posemeterCallback)
    rospy.Subscriber('/goal', PoseStamped, goalCallback)
    Node.publisher = rospy.Publisher('/hanse/pinger', PingerDetection)
    rospy.spin()
