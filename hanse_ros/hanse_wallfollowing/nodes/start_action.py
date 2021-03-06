#! /usr/bin/env python

import roslib; roslib.load_manifest('hanse_wallfollowing')
import rospy
import actionlib
import signal, os
from actionlib_msgs.msg import GoalStatus
from hanse_wallfollowing.msg import WallFollowingGoal, WallFollowingAction


def SignalHandler(signum, frame):
    print "Sighandler CTRL+C, CTRL+V"

if __name__ == '__main__':   
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('start_wallfollowing_action')

        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        client = actionlib.SimpleActionClient('wallfollowing', WallFollowingAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        signal.signal(signal.SIGINT, lambda signum, frame: client.cancel_goal())

        state = client.send_goal_and_wait(WallFollowingGoal())

        goalStatusDict = {
            GoalStatus.PENDING : "PENDING",
            GoalStatus.ACTIVE : "ACTIVE",
            GoalStatus.PREEMPTED : "PREEMPTED",
            GoalStatus.SUCCEEDED : "SUCCEEDED",
            GoalStatus.ABORTED : "ABORTED",
        }
        rospy.loginfo('Result: ' + goalStatusDict[state])
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")