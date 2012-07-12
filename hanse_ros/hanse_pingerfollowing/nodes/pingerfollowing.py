#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_pingerfollowing')
import rospy
import math
import dynamic_reconfigure.server
import time
from random import random
from geometry_msgs.msg import PoseStamped, Vector3
from geometry_msgs.msg import Twist
from hanse_msgs.msg import PingerDetection
from std_msgs.msg import Header
from std_msgs.msg import String
from hanse_pingerfollowing.cfg import PingerFollowingConfig

class Node:
    target = None
    lastTurnDistance = None
    lastTurnDirection = None
    bias = None
    lastWasTurn = False

class Config:
    pass

class Global:
    status = 'stop'

def configCallback(config, level):
    Config.initStep = config['init_step']
    Config.step = config['step']
    Config.biasStep = config['bias_step']
    Config.mu = config['mu']
    Config.hydrophoneDirection = config['hydrophone_direction']
    Config.bias = config['bias']
    Config.angularSpeed = config['angular_speed']
    Config.turnTime = config['turn_time']
    Config.linearSpeed = config['linear_speed']
    Config.minOneStep = config['min_one_step']
    return config

def statusCallback(msg):
    # rospy.loginfo(msg.data)
    Global.status = msg.data

def pingerCallback(msg):
    if status == stop:
        return

    avg = msg.leftAmplitude * (1-Config.hydrophoneDirection) + msg.rightAmplitude * Config.hydrophoneDirection
    distance = 1 / math.sqrt(avg)
    if Node.target == None:
        rospy.loginfo(repr(distance))
        rospy.loginfo(repr(Config.initStep))
        Node.target = distance + Config.initStep
    lastWasTurn = Node.lastWasTurn
    Node.lastWasTurn = False
    if distance > Node.target and not (Config.minOneStep and lastWasTurn):
        rospy.loginfo("richtungswechsel")
        Node.lastWasTurn = True
        lastTurnDistance = Node.lastTurnDistance
        Node.lastTurnDistance = distance - Config.biasStep
        bias = 0.5
        if lastTurnDistance != None:
            if lastTurnDistance < distance and Node.bias == None:
                Node.bias = Node.lastTurnDirection
                rospy.loginfo("setting bias %f" % Node.bias)
            if lastTurnDistance > distance:
                Node.bias = None
                rospy.loginfo("resetting bias")
        if Node.bias != None:
            bias += Config.bias * Node.bias
            rospy.loginfo("bias %f" % bias)
        angSpeed = 0
        if random() < bias:
            angSpeed = -1
            Node.lastTurnDirection = -1
        else:
            angSpeed = 1
            Node.lastTurnDirection = 1
        rospy.loginfo("turning %s" % Node.lastTurnDirection)
        setMotorSpeed(0, Config.angularSpeed * angSpeed)
        time.sleep(Config.turnTime)
        Node.target = distance + Config.initStep
    setMotorSpeed(Config.linearSpeed, 0)
    Node.target = (1 - Config.mu) * Node.target + Config.mu * (distance - Config.step)
    rospy.loginfo("weiter %s %s" % (Node.target, distance))


def setMotorSpeed(lin, ang):
    linearVector = Vector3(x=lin,z=0)
    angularVector = Vector3(z=ang)
    twist = Twist(linear=linearVector, angular=angularVector)
    pub_cmd_vel.publish(twist)

if __name__ == '__main__':
    rospy.init_node('pingerfollowing')
    dynamic_reconfigure.server.Server(PingerFollowingConfig, configCallback)
    rospy.Subscriber('/hanse/pinger', PingerDetection, pingerCallback)
    rospy.Subscriber('/hanse/pinger/status', String, statusCallback)
    Node.cmd_vel = pub_cmd_vel = rospy.Publisher('/hanse/commands/cmd_vel_behaviour', Twist)
    rospy.spin()
