#!/usr/bin/env python
PACKAGE = 'hanse_ballfollowing'
import roslib; roslib.load_manifest('hanse_ballfollowing')
import rospy
import dynamic_reconfigure.server
import math
import smach
import smach_ros
import numpy
from geometry_msgs.msg import PoseStamped, Point, Twist, Vector3
from hanse_msgs.msg import Object, sollSpeed
from hanse_ballfollowing.cfg import BallFollowingConfig
from hanse_ballfollowing.msg import BallFollowingAction

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
	kpAngle = 1.0
	lostFactor = 3
        offset = 0

class Global:
	x = 0.0
	y = 0.0
	size = 0.0
	isSizeTooSmall = False
	currentPosition = Point()
	currentHeading = 0.0
	startHeading = 0.0


#==============================================================================
# Constants
#==============================================================================
class States:
	NotSeenYet = 'NotSeenYet'
	Seen = 'Seen'
	Lost = 'Lost'
	Done = 'Done'

class Transitions:
	Seen = 'Seen'
	Lost = 'Lost'
	Done = 'Done'
	Aborted = 'Aborted'


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
		smach.State.__init__(self, outcomes=[Transitions.Seen, Transitions.Aborted])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.NotSeenYet)

		while not rospy.is_shutdown() and not self.preempt_requested():
			# if size between min und max..
			if Config.minSize < Global.size < Config.maxSize:
				Global.startHeading = Global.currentHeading
				return Transitions.Seen

			setMotorSpeed(0.0, Config.fwSpeed / Config.lostFactor)
			rospy.sleep(0.2)

		return self.abort()


class Seen(AbortableState):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.Lost, Transitions.Done, Transitions.Aborted])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.Seen)

		while not rospy.is_shutdown() and not self.preempt_requested():

			diffHeading = Global.currentHeading - Global.startHeading
			if diffHeading < 0:
				diffHeading += 2 * math.pi

			if diffHeading > 10 and diffHeading < 30:
				setMotorSpeed(0,0)
				return Transitions.Done

			# if size between min und max..
			if Config.minSize < Global.size < Config.maxSize:
				rospy.loginfo(str(Global.x)+ ' ' + str(IMAGE_COLS/2)) 
				################
				# gegebenenfalls an die kamera anpassen (division factor)
				################
				angularSpeed = Config.kpAngle * ((IMAGE_COLS / 4 - Global.x) / (IMAGE_COLS / 2) - Config.offset)
				rospy.loginfo('angular speed: '+str(angularSpeed))
				setMotorSpeed(Config.fwSpeed, angularSpeed)
				rospy.sleep(0.1)
			# lost
			else:
				setMotorSpeed(Config.fwSpeed, Config.fwSpeed / Config.lostFactor)
				rospy.sleep(0.1)
				return Transitions.Lost
				
		return self.abort()


class Lost(AbortableState):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.Seen, Transitions.Done, Transitions.Aborted])

	def execute(self, userdata):
		rospy.loginfo('Executing state '+States.Lost)


		while not rospy.is_shutdown() and not self.preempt_requested():
			if Config.minSize < Global.size < Config.maxSize:
				return Transitions.Seen

			if diffHeading > 10 and diffHeading < 30:
				setMotorSpeed(0,0)
				return Transitions.Done

			rospy.sleep(0.2)

		return self.abort()


#==============================================================================
# Callback functions
#==============================================================================
def objectCallback(msg):
	# rospy.loginfo('objectCallback: size='+repr(msg.size)+'\t\t orientation='+repr(msg.orientation));
	Global.size = msg.size
	Global.x = msg.x
	#rospy.loginfo('HIER WURDE X GEAENDERT' + str(Global.x))
	Global.y = msg.y
	#rospy.loginfo('distY: '+repr(distanceY / Config.maxDistance))


def configCallback(config, level):
	# rospy.loginfo('Reconfigure Request: ')
	Config.minSize = config['minSize']
	Config.maxSize = config['maxSize']
	Config.fwSpeed = config['fwSpeed']
	Config.kpAngle = config['kpAngle']
	Config.lostFactor = config['lostFactor']
	return config

def positionCallback(msg):	
	Global.currentPosition = msg.pose.position
	q = msg.pose.orientation	
	(roll,pitch,yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
	Global.currentHeading = yaw


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


#==============================================================================
# main
#==============================================================================
if __name__ == '__main__':
	rospy.init_node('ballfollowing')

	# Config server
	dynamic_reconfigure.server.Server(BallFollowingConfig, configCallback)
	
	# Subscriber
	rospy.Subscriber('/object', Object, objectCallback)
	rospy.Subscriber('position/estimate', PoseStamped, positionCallback)

	# Publisher
	#pub_cmd_vel = rospy.Publisher('/hanse/commands/cmd_vel', Twist)
	#pub_motor_left = rospy.Publisher('/hanse/motors/left', sollSpeed)
	#pub_motor_right = rospy.Publisher('/hanse/motors/right', sollSpeed)
	pub_cmd_vel = rospy.Publisher('/hanse/commands/cmd_vel_behaviour', Twist)

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=[Transitions.Done, Transitions.Aborted])

    # Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(States.NotSeenYet, NotSeenYet(), 
                               transitions={Transitions.Seen : States.Seen})
		smach.StateMachine.add(States.Seen, Seen(), 
                               transitions={Transitions.Lost : States.Lost,
                               				Transitions.Done : States.Done})
		smach.StateMachine.add(States.Lost, Lost(), 
                               transitions={Transitions.Seen : States.Seen,Transitions.Done : States.Done})

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('ballfollowing_introspection_server', sm, '/SM_ROOT')
	sis.start()

    # Execute SMACH plan
	#outcome = sm.execute()
	# Construct action server wrapper
	asw = smach_ros.ActionServerWrapper(
	    rospy.get_name(), BallFollowingAction,
	    wrapped_container = sm,
	    succeeded_outcomes = [Transitions.Done],
	    aborted_outcomes = [Transitions.Aborted],
	    preempted_outcomes = [])

	# Run the server
	asw.run_server()

	rospy.spin()
	sis.stop()
