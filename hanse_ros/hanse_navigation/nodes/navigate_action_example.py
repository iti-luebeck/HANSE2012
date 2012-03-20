#! /usr/bin/env python
import roslib; roslib.load_manifest('hanse_navigation')
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point
from hanse_navigation.msg import NavigateGoal, NavigateAction

def create_nav_goal(x, y, z):
	ps = PoseStamped()
	ps.pose.position = Point(x=x, y=y, z=z)
	return NavigateGoal(goal = ps)

if __name__ == '__main__':
	rospy.init_node('navigate_action_example')

	# Creates the SimpleActionClient, passing the type of the action
	# (NavigateAction) to the constructor.
	client = actionlib.SimpleActionClient('/hanse/navigation', NavigateAction)
	client.wait_for_server()

	# Creates a goal to send to the action server.
	# And wait for the server to finish performing the action.
	goal = create_nav_goal(2.0, 1.0, 0.0)
	state = client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(6))

	# Prints out the result of executing the action
	if state == GoalStatus.SUCCEEDED:
		rospy.loginfo('navigation succeeded')
	else:
		rospy.loginfo('navigation failed: ' + GoalStatus.to_string(state))
