#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_behaviourctrl')
import rospy
import smach
import smach_ros
import signal
import os
import actionlib
from hanse_msgs.msg import sollSpeed, pressure
from std_msgs.msg import Float64, Float32, String
from geometry_msgs.msg import PoseStamped, Point, Twist, Vector3
from sensor_msgs.msg import Imu
from datetime import datetime
from actionlib_msgs.msg import GoalStatus
from hanse_navigation.msg import NavigateGoal, NavigateAction
from hanse_pipefollowing.msg import PipeFollowingGoal, PipeFollowingAction
from hanse_wallfollowing.msg import WallFollowingGoal, WallFollowingAction
from hanse_srvs.srv import *


class Global:
	SIMULATOR = True
	time_initial = 0.0
	duration = rospy.Duration(0,0)
	logfile = " "
	currentPosition = Point()
	pressure_initial = 0
	pressure = 0
	action = " "
	action_state = " "
	depth = 120

class States:
	Init = 'Init'
	Submerge = 'Submerge'
	valGate = 'valGate'
	pipeFollow = 'pipeFollow'
	navigateToWall = 'navigateToWall'
	midWater = 'midWater'
	wallFollow = 'wallFollow'
	Surface = 'Surface'

class Transitions:
	Init_Finished = 'Init_Finished'
	Submerged = 'Submerged'
	Submerge_failed = 'Submerged_failed'
	Goal_failed = 'Goal_failed'
	Goal_passed = 'Goad_passed'
	Pipe_passed = 'Pipe_passed'
	Pipe_failed = 'Pipe_failed'
	navigateToWall = 'navigateToWall'
	navigatewall_passed = 'navigatewall_passed'
	navigatewall_failed = 'navigatewall_failed'
	Wall_passed = 'Wall_passed'
	Wall_failed = 'Wall_failed'
	Surfaced = 'Surfaced'

#################################
# define state Init
#################################
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[Transitions.Init_Finished])
	Global.time_initial = rospy.Time.now()
	
    def execute(self, userdata):
	rospy.loginfo('init')
	createfile(0)
        return Transitions.Init_Finished


#################################
# define state Submerge
#################################
class submerge(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[Transitions.Submerged,Transitions.Submerge_failed])
        
    def execute(self, userdata):
	Global.action = "auv submerging"
	# Auf Service warten und PID-Regler aktivieren
	rospy.loginfo('Init: Waiting for depth_target service...')
	rospy.wait_for_service('/hanse/engine/depth/setDepth')
	rospy.loginfo('Init: Waiting for depth_target service... FINISHED')
	call_submerge = rospy.ServiceProxy('/hanse/engine/depth/setDepth', SetTarget)
	call_submerge(depth)
	while Global.duration.secs < 60:
		#rospy.loginfo(str(Global.pressure-Global.pressure_initial))
		if (Global.pressure-Global.pressure_initial) > 80:
			rospy.loginfo('success')
			return Transitions.Submerged
	return Transitions.Submerge_failed			
	
		

#################################
# define state validationGate
#################################
class validationGate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[Transitions.Goal_passed,Transitions.Goal_failed])
        

    def execute(self, userdata):
	Global.action = "navigate to validation gate"
	client = actionlib.SimpleActionClient('/hanse/navigation', NavigateAction)
	rospy.loginfo("client started and waiting")
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()
	rospy.loginfo("server listening for goals")
	signal.signal(signal.SIGINT, lambda signum, frame: client.cancel_goal())
	##############
	# enter nav goal
	##############
	goal = create_nav_goal(-4.0, 6.0, 0.0)
	state = client.send_goal_and_wait(goal, rospy.Duration(5))
        if state == GoalStatus.SUCCEEDED and Global.duration.secs < 60:
		rospy.loginfo('navigation succeeded')
		goal = create_nav_goal(4.0, 6.0, 0.0)
		state = client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(240))
		if state == GoalStatus.SUCCEEDED:
			##############
			# enter nav goal
			##############
			rospy.loginfo('navigation succeeded')
			goal = create_nav_goal(2.0, 6.0, 0.0)
			state = client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(240))
			return Transitions.Goal_passed
		else:
			rospy.loginfo('navigation failed: ' + GoalStatus.to_string(state))
			return Transitions.Goal_failed
	else:
		rospy.loginfo('navigation failed: ' + GoalStatus.to_string(state))
		return Transitions.Goal_failed

#################################
# define state PipeFollowing
#################################
class PipeFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[Transitions.Pipe_passed,Transitions.Pipe_failed])

    def execute(self, userdata):
	start_time = Global.duration.secs
	Global.action = "follow pipe"
	client = actionlib.SimpleActionClient('pipefollowing', PipeFollowingAction)
	rospy.loginfo("client started and waiting")
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()
	rospy.loginfo("server listening for goals")
        signal.signal(signal.SIGINT, lambda signum, frame: client.cancel_goal())
        state = client.send_goal_and_wait(PipeFollowingGoal(), execute_timeout = rospy.Duration(300))

        goalStatusDict = {
            GoalStatus.PENDING : "PENDING",
            GoalStatus.ACTIVE : "ACTIVE",
            GoalStatus.PREEMPTED : "PREEMPTED",
            GoalStatus.SUCCEEDED : "SUCCEEDED",
            GoalStatus.ABORTED : "ABORTED",
        }
        rospy.loginfo('Result: ' + goalStatusDict[state])
	if state == GoalStatus.SUCCEEDED:
	    return Transitions.Pipe_passed
        else :
	    return Transitions.Pipe_failed




#################################
# define state navigateToWall
#################################
class navigateToWall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[Transitions.navigatewall_passed,Transitions.navigatewall_failed])

    def execute(self, userdata):
	Global.action = "navigate to wallfollowing"
	client = actionlib.SimpleActionClient('/hanse/navigation', NavigateAction)
	rospy.loginfo("client started and waiting")
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()
	rospy.loginfo("server listening for goals")
	signal.signal(signal.SIGINT, lambda signum, frame: client.cancel_goal())
	############
	# enter nav goal for wall
	############
	goal = create_nav_goal(8, -11.0, 0.0)
	state = client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(240))
        if state == GoalStatus.SUCCEEDED:
		return Transitions.navigatewall_passed
	else:
		return Transitions.navigatewall_failed
		
#################################
# define state WallFollowing
#################################
class WallFollowing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[Transitions.Wall_passed,Transitions.Wall_failed])

    def execute(self, userdata):
	start_time = Global.duration.secs
	Global.action = "follow wall"
	# Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.
        client = actionlib.SimpleActionClient('/wallfollowing', WallFollowingAction)
	rospy.loginfo("client started and waiting")
        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()
	rospy.loginfo("server listening for goals")
	
        signal.signal(signal.SIGINT, lambda signum, frame: client.cancel_goal())

	#Setting the time limit for wallfollowing
	while Global.duration.secs-start_time < 360:
		state = client.send_goal_and_wait(WallFollowingGoal(),rospy.Duration(5))
		#client.cancel_goal()
		if(Global.currentPosition.y > 15):
			return Transitions.Wall_passed 	
	
	return Transitions.Wall_failed


#################################
# define state Surface
#################################
class surface(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[Transitions.Surfaced])
        
    def execute(self, userdata):
	Global.action = "auv surfacing"
	# Auf Service warten und PID-Regler aktivieren
	rospy.loginfo('Init: Waiting for depth_target service...')
	rospy.wait_for_service('/hanse/engine/depth/setDepth')
	rospy.loginfo('Init: Waiting for depth_target service... FINISHED')
	call_surface = rospy.ServiceProxy('/hanse/engine/depth/setDepth', SetTarget)
	call_surface(0.0)
	rospy.sleep(10)
	
	return Transitions.Surfaced

#timerCallback calls the routine for creating the logfile
def timerCallback(event): 
    Global.duration = rospy.Time.now() - Global.time_initial
    f = open(Global.logfile,"a")
    f.write('('+str(Global.duration.secs)+','+str(round(Global.currentPosition.x,2))+','
	+str(round(Global.currentPosition.y,2))+','+str(Global.pressure-Global.pressure_initial)+','+Global.action+')\n')
    f.close()

def actionstateCallback(msg): 
    Global.action_state = msg.data

#reading data from the pressure sensor
def pressureCallback(msg): 
    if not(Global.pressure_initial):
		Global.pressure_initial = msg.data
    Global.pressure = msg.data

#getting the position from the localisation
def positionCallback(msg):	
	Global.currentPosition = msg.pose.position

#creating a goal that can be used as a target for the navigation with absolute coordinates x,y and z
#(z may be set by directly using depth_control	
def create_nav_goal(x, y, z):
	ps = PoseStamped()
	ps.pose.position = Point(x=x, y=y, z=z)
	return NavigateGoal(goal = ps)	


#creates a logfile that is used for one mission
def createfile(i):
    Global.logfile = "hanselog_"+str(i)+".log"
    if os.path.isfile(Global.logfile):
	createfile(i+1)
    else:
	date = datetime.fromtimestamp(Global.time_initial.secs)
	f = open(Global.logfile,"w")
	f.write('log file hanse: ' + str(date.year) + '-' + str(date.month) + '-' + str(date.day) + '  ' + str(date.hour))  

	if date.minute < 10:
	   f.write(':0' + str(date.minute))
	else:
	   f.write(':' + str(date.minute))
	
	if date.second < 10:
	   f.write(':0' + str(date.second)+'\n')
	else:
	   f.write(':' + str(date.second)+'\n')

	f.close()
 	
def main():
    rospy.init_node('behaviourctrl')

    if Global.SIMULATOR:
		rospy.Subscriber('/hanse/posemeter', PoseStamped, positionCallback)
    else:
		rospy.Subscriber('position/estimate', PoseStamped, positionCallback)
    rospy.Subscriber('/hanse/pressure/depth', pressure, pressureCallback)
    rospy.Subscriber('/hanse/actionstate', String, actionstateCallback)
	


    rospy.Timer(rospy.Duration(1.0), timerCallback)
	
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome'])

    # Open the container
    with sm:
        # Add states to the container
	smach.StateMachine.add(States.Init, Init(), 
                               transitions={Transitions.Init_Finished:States.Submerge})
	smach.StateMachine.add(States.Submerge, submerge(), 
                               transitions={Transitions.Submerged:States.valGate, Transitions.Submerge_failed:States.Surface})
        smach.StateMachine.add(States.valGate, validationGate(), 
                               transitions={Transitions.Goal_passed:States.pipeFollow, Transitions.Goal_failed:States.Surface})
        smach.StateMachine.add(States.pipeFollow, PipeFollowing(), 
                              transitions={Transitions.Pipe_passed:States.navigateToWall, Transitions.Pipe_failed : States.valGate})
	smach.StateMachine.add(States.navigateToWall, navigateToWall(), 
                               transitions={Transitions.navigatewall_passed:States.wallFollow, Transitions.navigatewall_failed : States.navigateToWall})
	smach.StateMachine.add(States.wallFollow, WallFollowing(), 
                               transitions={Transitions.Wall_passed:States.Surface, Transitions.Wall_failed : States.navigateToWall})
	smach.StateMachine.add(States.Surface, surface(), 
                               transitions={Transitions.Surfaced:'outcome'})

    # Execute SMACH plan
    outcome = sm.execute()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Wait for ctrl-c to stop the application
    sis.stop()
    #rospy.spin()


if __name__ == '__main__':
    main()







