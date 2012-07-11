#!/usr/bin/env python
PACKAGE = 'hanse_pipefollowing'
import roslib; roslib.load_manifest('hanse_pipefollowing')
import rospy
import dynamic_reconfigure.server
import math
import smach
import smach_ros
import numpy
from geometry_msgs.msg import PoseStamped, Point, Twist, Vector3
from hanse_msgs.msg import Object, sollSpeed
from std_msgs.msg import  String
from hanse_pipefollowing.cfg import PipeFollowingConfig
from hanse_pipefollowing.msg import PipeFollowingAction

#################
IMAGE_COLS = 640
IMAGE_ROWS = 480
#################

# TODO Global.x/y/lastX/lastY locken?


# The pipe is seen if:
#   1. at least 1/20th of the image is "pipe"
#   2. at most half of the image is "pipe"
# We have successfully passed the pipe if
#   1. we see the pipe
#   2. the pipe angle is about 0
#   3. the pipe center is in the lower quater of the image.


class Config:
	minSize = 0.05
	maxSize = 0.4
	fwSpeed = 0.8
	deltaAngle = 0.192   # 0.192 radians = 11 degrees
	deltaDist = 100
	kpAngle = 1.0
	kpDist = 1.0
	robCenterX = 320
	robCenterY = 240
	maxDistance = 320
	mirror = False
	pipe_passed = 63.6

class Global:
	x = 0.0
	y = 0.0
	size = 0.0
	orientation = 0.0
	lastX = 0.0
	lastY = 0.0
	isSizeTooSmall = False
	currentPosition = Point()
	is_seen = True
	state = " "
	distance = 0


#==============================================================================
# Constants
#==============================================================================
class States:
	NotSeenYet = 'NotSeenYet'
	Passed = 'Passed'
	IsSeen = 'IsSeen'
	Lost = 'Lost'

class Transitions:
	IsSeen = 'IsSeen'
	Passed = 'Passed'
	Lost = 'Lost'
	Aborted = 'Aborted'

class LostTypes:
	LostLeft = 'LostLeft'
	LostRight = 'LostRight'
	LostBottom = 'LostBottom'
	LostTop = 'LostTop'
	Lost = 'Lost'


#==============================================================================
# State classes
#==============================================================================
class AbortableState(smach.State):
	def abort(self):
		setMotorSpeed(0,0)
		self.service_preempt()
		return Transitions.Aborted


class NotSeenYet(AbortableState):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.IsSeen, Transitions.Aborted])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.NotSeenYet)

		while not rospy.is_shutdown() and not self.preempt_requested():
			# if size between min und max..
			if Config.minSize < Global.size < Config.maxSize:
				return Transitions.IsSeen

			setMotorSpeed(Config.fwSpeed, 0.0)
			rospy.sleep(0.2)

		return self.abort()


class IsSeen(AbortableState):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.Lost, Transitions.Passed, Transitions.Aborted],
								output_keys=['lost_type'])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.IsSeen)

		while not rospy.is_shutdown() and not self.preempt_requested():

			# if size between min und max..
			if Global.is_seen:
				# end of pipe reached?
				#Coordiantes for end of pipe
				if Global.currentPosition.y > Global.pipe_passed:
					setMotorSpeed(0,0)
					return Transitions.Passed
			# lost
			else:
				tmp_x = 0.0
				tmp_y = 0.0
				# lost if less than minSize is seen
				if Global.size <= Config.minSize:
					tmp_x = Global.x
					tmp_y = Global.y
				# lost if more than maxSize is seen
				elif Global.size >= Config.maxSize:
					tmp_x = Global.lastX
					tmp_y = Global.lastY

				tmp_x /= IMAGE_COLS
				tmp_y /= IMAGE_ROWS

				if   tmp_x < 0.5: userdata.lost_type = LostTypes.LostLeft
				elif tmp_x >= 0.5: userdata.lost_type = LostTypes.LostRight
				elif tmp_y < 0.5: userdata.lost_type = LostTypes.LostTop
				elif tmp_y >= 0.5: userdata.lost_type = LostTypes.LostBottom
				else:              userdata.lost_type = LostTypes.Lost
				return Transitions.Lost


			distanceY = computeIntersection(Global.x, Global.y, Global.orientation)
			#if not Config.mirror:
                        distanceY = -distanceY
			#rospy.loginfo('distanceY: ' + repr(distanceY))
			angularSpeed = 0.0
			if math.fabs(Global.orientation) > Config.deltaAngle:
				angularSpeed = Config.kpAngle * Global.orientation / (math.pi/2)
			if math.fabs(distanceY) > Config.deltaDist:
				angularSpeed += Config.kpDist * distanceY / Config.maxDistance

			#rospy.loginfo('angularSpeed: ' + repr(angularSpeed) + '\t\t ('+repr(Global.x)+','+repr(Global.y)+')')
			setMotorSpeed(Config.fwSpeed, angularSpeed)
			rospy.sleep(0.2)
				
		return self.abort()


class Lost(AbortableState):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.IsSeen, Transitions.Aborted],
									input_keys=['lost_type'])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.Lost+' ('+userdata.lost_type +')')


		if userdata.lost_type == LostTypes.Lost:
			while not rospy.is_shutdown():
				rospy.loginfo('PANIC: lost');
				return self.abort()
		else:
			# linear-/angularspeed tuples
			speedDict = {
				LostTypes.LostLeft:  (Config.fwSpeed, -0.2),
				LostTypes.LostRight: (Config.fwSpeed, 0.2),
				LostTypes.LostBottom:(0.0, 0.2),
				LostTypes.LostTop:   (Config.fwSpeed, 0.0),
			}

			while not rospy.is_shutdown() and not self.preempt_requested():
				if Config.minSize < Global.size < Config.maxSize:
					return Transitions.IsSeen

				setMotorSpeed(*speedDict[userdata.lost_type])
				rospy.sleep(0.2)

			return self.abort()


#==============================================================================
# Callback functions
#==============================================================================
def objectCallback(msg):
	#rospy.loginfo('objectCallback: size='+repr(msg.size)+'\t\t orientation='+repr(msg.orientation));
	Global.lastX = Global.x
	Global.lastY = Global.y
	Global.size = msg.size
	Global.is_seen = msg.is_seen
	if Config.mirror:
		Global.x = (IMAGE_COLS - msg.x)
		Global.y = (IMAGE_ROWS - msg.y)
		Global.orientation = -msg.orientation	
	else:
		Global.x = msg.x
		Global.y = msg.y
		Global.orientation = msg.orientation
	distanceY = computeIntersection(Global.x, Global.y, Global.orientation)
	#rospy.loginfo('distY: '+repr(distanceY / Config.maxDistance))


def configCallback(config, level):
	rospy.loginfo('Reconfigure Request: ')
	Config.minSize = config['minSize']
	Config.maxSize = config['maxSize']
	Config.fwSpeed = config['fwSpeed']
	Config.deltaAngle = config['deltaAngle']
	Config.deltaDist = config['deltaDist']
	Config.kpAngle = config['kpAngle']
	Config.kpDist = config['kpDist']
	Config.robCenterX = config['robCenterX']
	Config.robCenterY = config['robCenterY']
	Config.maxDistance = config['maxDistance']
	Config.mirror = config['mirror']
	return config

def positionCallback(msg):	
	Global.currentPosition = msg.pose.position

#==============================================================================
# Helper functions
#==============================================================================
def hasPassed():
	return (math.fabs(Global.orientation) < math.pi/6.0) and (Global.y > 0.75*IMAGE_ROWS) and (0.2*IMAGE_COLS < Global.x < 0.8*IMAGE_COLS)	


def computeIntersection(meanX, meanY, theta):
	robX = Config.robCenterX
	robY = Config.robCenterY

	nzero = (math.cos(theta), math.sin(theta))
	d = meanX * nzero[0] + meanY * nzero[1];

	# nzero * p - d
	return (nzero[0] * robX) + (nzero[1] * robY) - d;


# werte im bereich [-1, 1]
def setMotorSpeed(lin, ang):
	linearVector = Vector3(x=lin,z=0)
	angularVector = Vector3(z=ang)
	twist = Twist(linear=linearVector, angular=angularVector)
	pub_cmd_vel.publish(twist)
	#
	#ang = ang
	# geschwindigkeitswerte fuer thruster berechnen
	#left = lin*127 + ang*127
	#right = lin*127 - ang*127
	# auf den wertebereich -127 bis 127 beschraenken
	#left = numpy.clip(left, -127, 127)
	#right = numpy.clip(right, -127, 127)
	# nachrichten an motoren publishen
	#pub_motor_left.publish(sollSpeed(data = left))
	#pub_motor_right.publish(sollSpeed(data = right))

def timerCallback(event):
    pub_behaviour_info.publish(String(data = 'Orientation:  '+str(Global.orientation)+ ' x-Distance to pipe:  '+str(Global.distance)))



#==============================================================================
# main
#==============================================================================
if __name__ == '__main__':
	rospy.init_node('pipefollowing')

	# Config server
	dynamic_reconfigure.server.Server(PipeFollowingConfig, configCallback)
	
	# Subscriber
	rospy.Subscriber('/object', Object, objectCallback)
	rospy.Subscriber('position/estimate', PoseStamped, positionCallback)

	# Publisher
	#pub_cmd_vel = rospy.Publisher('/hanse/commands/cmd_vel', Twist)
	#pub_motor_left = rospy.Publisher('/hanse/motors/left', sollSpeed)
	#pub_motor_right = rospy.Publisher('/hanse/motors/right', sollSpeed)
	pub_cmd_vel = rospy.Publisher('/hanse/commands/cmd_vel_behaviour', Twist)
	pub_behaviour_info = rospy.Publisher('/hanse/behaviour/pipefollow_info', String)
	rospy.Timer(rospy.Duration(1.0 / 10), timerCallback)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=[Transitions.Passed, Transitions.Aborted])

    # Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(States.NotSeenYet, NotSeenYet(), 
                               transitions={Transitions.IsSeen : States.IsSeen})
		smach.StateMachine.add(States.IsSeen, IsSeen(), 
                               transitions={Transitions.Lost : States.Lost,
                               				Transitions.Passed : States.Passed})
		smach.StateMachine.add(States.Lost, Lost(), 
                               transitions={Transitions.IsSeen : States.IsSeen})

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('pipefollowing_introspection_server', sm, '/SM_ROOT')
	sis.start()

    # Execute SMACH plan
	#outcome = sm.execute()
	# Construct action server wrapper
	asw = smach_ros.ActionServerWrapper(
	    rospy.get_name(), PipeFollowingAction,
	    wrapped_container = sm,
	    succeeded_outcomes = [Transitions.Passed],
	    aborted_outcomes = [Transitions.Aborted],
	    preempted_outcomes = [])

	# Run the server
	asw.run_server()

	rospy.spin()
	sis.stop()
