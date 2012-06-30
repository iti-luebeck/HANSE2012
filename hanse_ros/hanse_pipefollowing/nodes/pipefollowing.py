#!/usr/bin/env python
PACKAGE = 'hanse_pipefollowing'
import roslib; roslib.load_manifest('hanse_pipefollowing')
import rospy
import dynamic_reconfigure.server
import math
import smach
import smach_ros
import numpy
from geometry_msgs.msg import Twist, Vector3
from hanse_msgs.msg import Object, sollSpeed
from hanse_pipefollowing.cfg import PipeFollowingConfig

#################
IMAGE_COLS = 640
IMAGE_ROWS = 480
#################

# bisher gemacht: 
# - logik aus PipeTracker::update auf smach uebertragen
# - logik aus Behaviour_PipeFollowing::controlPipeFollow uebernommen
# - dynamic_reconfigure

# TODO herausfinden wofuer der zweite p-regler ist
# TODO Global.x/y/lastX/lastY locken
# TODO generische Lost-Klasse um platz zu sparen
# TODO actionserver erstellen zum starten/stoppen des verhaltens



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

class Global:
	x = 0.0
	y = 0.0
	size = 0.0
	orientation = 0.0
	lastX = 0.0
	lastY = 0.0
	isSizeTooSmall = False

#=======================================
# Constants
#=======================================
class States:
	NotSeenYet = 'NotSeenYet'
	Passed = 'Passed'
	IsSeen = 'IsSeen'
	LostLeft = 'LostLeft'
	LostRight = 'LostRight'
	LostBottom = 'LostBottom'
	LostTop = 'LostTop'
	Lost = 'Lost'

class Transitions:
	IsSeen = 'IsSeen'
	Passed = 'Passed'
	LostLeft = 'LostLeft'
	LostRight = 'LostRight'
	LostBottom = 'LostBottom'
	LostTop = 'LostTop'
	Lost = 'Lost'


#=======================================
# State classes
#=======================================
class NotSeenYet(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.IsSeen, Transitions.Passed])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.NotSeenYet)

		while not rospy.is_shutdown():
			# if size between min und max..
			if Global.size < Config.maxSize and Global.size > Config.minSize:		
				# end of pipe reached?
				if hasPassed():
					return Transtions.Passed
				return Transitions.IsSeen

			setMotorSpeed(Config.fwSpeed, 0.0)
			rospy.sleep(0.2);



class IsSeen(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.Lost,Transitions.LostBottom,Transitions.LostTop,Transitions.LostRight,Transitions.LostLeft])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.IsSeen)

		while not rospy.is_shutdown():

			# if size between min und max..
			if Config.minSize < Global.size < Config.maxSize:
				# end of pipe reached?
				if hasPassed():
					return Transitions.Passed

			# lost if less than minSize is seen
			if Global.size <= Config.minSize:
				if   Global.x < 0.25: return Transitions.LostLeft
				elif Global.x > 0.75: return Transitions.LostRight
				elif Global.y < 0.25: return Transitions.LostTop
				elif Global.y > 0.75: return Transitions.LostBottom
				else:                 return Transitions.Lost

			# lost if more than maxSize is seen
			elif Global.size >= Config.maxSize:
				if   Global.lastX < 0.25: return Transitions.LostLeft
				elif Global.lastX > 0.75: return Transitions.LostRight
				elif Global.lastY < 0.25: return Transitions.LostTop
				elif Global.lastY > 0.75: return Transitions.LostBottom
				else:                     return Transitions.Lost


			distanceY = computeIntersection(Global.x, Global.y, Global.orientation)
			#rospy.loginfo('distanceY: ' + repr(distanceY))
			angularSpeed = 0.0
			if math.fabs(Global.orientation) > Config.deltaAngle:
				angularSpeed = Config.kpAngle * Global.orientation / (math.pi/2)
			#if math.fabs(distanceY) > Config.deltaDist:
			#	angularSpeed += Config.kpDist * distanceY / Config.maxDistance

			rospy.loginfo('angularSpeed: ' + repr(-angularSpeed) + '\t\t ('+repr(Global.x)+','+repr(Global.y)+')')
			setMotorSpeed(Config.fwSpeed- math.fabs(angularSpeed), -angularSpeed)
			rospy.sleep(0.2)
				

class LostLeft(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.IsSeen, Transitions.Passed])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.LostLeft)

		while not rospy.is_shutdown():
			transition = determineTransitionFromLostState()
			if transition != None: return transition

			setMotorSpeed(Config.fwSpeed, -0.2)
			rospy.sleep(0.2)


class LostRight(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.IsSeen, Transitions.Passed])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.LostRight)

		while not rospy.is_shutdown():
			transition = determineTransitionFromLostState()
			if transition != None: return transition

			setMotorSpeed(Config.fwSpeed, 0.2)
			rospy.sleep(0.2)


class LostTop(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.IsSeen, Transitions.Passed])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.LostTop)

		while not rospy.is_shutdown():
			transition = determineTransitionFromLostState()
			if transition != None: return transition

			setMotorSpeed(Config.fwSpeed, 0.0)
			rospy.sleep(0.2)


class LostBottom(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.IsSeen, Transitions.Passed])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.LostBottom)

		while not rospy.is_shutdown():
			transition = determineTransitionFromLostState()
			if transition != None: return transition

			setMotorSpeed(0.0, 0.2)
			rospy.sleep(0.2)


class Lost(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.IsSeen, Transitions.Passed])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.Lost)

		while not rospy.is_shutdown():
			rospy.loginfo('PANIC: lost');
			rospy.sleep(1.0)


class Passed(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.Passed)

		setMotorSpeed(0.0, 0.0)


#=======================================
# Callback functions
#=======================================
def objectCallback(msg):
	#rospy.loginfo('objectCallback: size='+repr(msg.size)+'\t\t orientation='+repr(msg.orientation));
	
	Global.lastX = Global.x
	Global.lastY = Global.y
	Global.x = msg.x / IMAGE_COLS
	Global.y = msg.y / IMAGE_ROWS
	Global.size = msg.size
	Global.orientation = msg.orientation

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
	return config


#=======================================
# Helper functions
#=======================================

def hasPassed():
	return (math.fabs(Global.orientation) < math.pi/6.0) and (Global.y > 0.75) and (0.2 < Global.x < 0.8)

def determineTransitionFromLostState():
	# size between min and max value
	if Config.minSize < Global.size < Config.maxSize:
		# end of pipe reached?
		if hasPassed():
			return Transtions.Passed
		return Transitions.IsSeen
	return None


def computeIntersection(meanX, meanY, theta):
	robX = Config.robCenterX
	robY = Config.robCenterY

	nzero = (math.cos(theta), math.sin(theta))
	d = meanX * nzero[0] + meanY * nzero[1];

	# nzero * p - d
	return (nzero[0] * robX) + (nzero[1] * robY) - d;


# werte im bereich [-1, 1]
def setMotorSpeed(lin, ang):
	#linearVector = Vector3(x=lin,z=0)
	#angularVector = Vector3(z=-ang)
	#twist = Twist(linear=linearVector, angular=angularVector)
	#pub_cmd_vel.publish(twist)
	#
	# geschwindigkeitswerte fuer thruster berechnen
	left = lin*127 + ang*127
	right = lin*127 - ang*127
	# auf den wertebereich -127 bis 127 beschraenken
	left = numpy.clip(left, -127, 127)
	right = numpy.clip(right, -127, 127)
	# nachrichten an motoren publishen
	pub_motor_left.publish(sollSpeed(data = left))
	pub_motor_right.publish(sollSpeed(data = right))



#=======================================
# main
#=======================================
if __name__ == '__main__':
	rospy.init_node('pipefollowing')

	# Config server
	dynamic_reconfigure.server.Server(PipeFollowingConfig, configCallback)
	
	# Subscriber
	rospy.Subscriber('/object', Object, objectCallback)

	# Publisher
	#pub_cmd_vel = rospy.Publisher('/hanse/commands/cmd_vel', Twist)
	pub_motor_left = rospy.Publisher('/hanse/motors/left', sollSpeed)
	pub_motor_right = rospy.Publisher('/hanse/motors/right', sollSpeed)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=[])

    # Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(States.NotSeenYet, NotSeenYet(), 
                               transitions={Transitions.IsSeen : States.IsSeen, 
											Transitions.Passed : States.Passed})
		smach.StateMachine.add(States.IsSeen, IsSeen(), 
                               transitions={Transitions.LostLeft : States.LostLeft, 
											Transitions.LostRight : States.LostRight,
											Transitions.LostTop : States.LostTop,
											Transitions.LostBottom : States.LostBottom,
											Transitions.Lost : States.Lost})

		smach.StateMachine.add(States.LostLeft, LostLeft(), 
                               transitions={Transitions.IsSeen : States.IsSeen, 
											Transitions.Passed : States.Passed})
		smach.StateMachine.add(States.LostRight, LostRight(), 
                               transitions={Transitions.IsSeen : States.IsSeen, 
											Transitions.Passed : States.Passed})
		smach.StateMachine.add(States.LostTop, LostTop(), 
                               transitions={Transitions.IsSeen : States.IsSeen, 
											Transitions.Passed : States.Passed})
		smach.StateMachine.add(States.LostBottom, LostBottom(), 
                               transitions={Transitions.IsSeen : States.IsSeen, 
											Transitions.Passed : States.Passed})
		smach.StateMachine.add(States.Lost, Lost(), 
                               transitions={Transitions.IsSeen : States.IsSeen, 
											Transitions.Passed : States.Passed})

		smach.StateMachine.add(States.Passed, Passed(), 
                               transitions={})

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('pipefollowing_introspection_server', sm, '/SM_ROOT')
	sis.start()

    # Execute SMACH plan
	outcome = sm.execute()

	rospy.spin()
	sis.stop()