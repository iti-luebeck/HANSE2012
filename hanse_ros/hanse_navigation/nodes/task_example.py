#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_navigation')
import rospy
from smach import StateMachine
from smach_ros import SimpleActionState, IntrospectionServer
from hanse_navigation.msg import NavigateGoal, NavigateAction
from geometry_msgs.msg import Point, PoseStamped

NAV_ACTION_NS = '/hanse/navigation'
SUCCEEDED = 'succeeded'
ABORTED = 'aborted'
PREEMPTED = 'preempted'

def create_nav_goal(x, y, z):
	ps = PoseStamped()
	ps.pose.position = Point(x=x, y=y, z=z)
	return NavigateGoal(goal = ps)

## @param point Navigationsziel
## @param timeout In Sekunden
def create_nav_state(point, timeout=None):
	goal = create_nav_goal(point.x, point.y, point.z)
	if timeout != None:
		timeout = rospy.Duration(timeout)
	return SimpleActionState(NAV_ACTION_NS, NavigateAction,
	                            goal=goal, exec_timeout=timeout)

if __name__ == '__main__':
	rospy.init_node('task_example')	
	sm = StateMachine([SUCCEEDED, ABORTED, PREEMPTED])
	with sm:
		####################################################
		StateMachine.add('navigate_to_2_1',
		              		create_nav_state(Point(x=2.0,y=1.0,z=0.0), timeout=60),
		              		transitions={SUCCEEDED:'navigate_to_0_0'})

		####################################################  
 		StateMachine.add('navigate_to_0_0',
							create_nav_state(Point(x=0.0,y=0.0,z=0.0), timeout=60),
							transitions={})

	# Create and start the introspection servers
	sis = IntrospectionServer('task_example', sm, '/TASK_EXAMPLE')
	sis.start()

    # Execute SMACH plan
	outcome = sm.execute()
	rospy.loginfo('state machine stopped')

	sis.stop()		                  
