#!/usr/bin/env python
PACKAGE = 'hanse_wallfollowing'
import roslib; roslib.load_manifest('hanse_wallfollowing')
import rospy
import smach
import smach_ros
import numpy
import math
from rospy.numpy_msg import numpy_msg
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float64, Float32
from hanse_wallfollowing.cfg import WallFollowingConfig
from hanse_msgs.msg import ScanningSonar, sollSpeed

######################################################
# Hier ist das Verhalten von Hanse2011 implementiert #
######################################################

class Global:
	distance = 0.0

class Config:
	corridorWidth = 0.2
	desiredDistance = 1.5
	forwardSpeed = 0.5
	angularSpeed = 0.4

class States:
	NoWall = 'NoWall'
	AdjustStart = 'AdjustStart'
	ControlWallfollow = 'ControlWallfollow'
	NoTurn = 'NoTurn'
	TurnLeft = 'TurnLeft'
	TurnRight = 'TurnRight'

class Transitions:
	WallTooClose = 'WallTooClose'
	WallTooFar = 'WallTooFar'
	InCorridor = 'InCorridor'
	Wall = 'Wall'
	NoWall = 'NoWall'
	

class AdjustStart(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['test','raus'])

	def execute(self, userdata):		
		return 'test'
		rospy.loginfo('Executing state AdjustStart')
		# 5 sekunden vorwaerts fahren
		setMotorSpeed(0.8, 0)
		rospy.sleep(5.0)
		setMotorSpeed(0, 0)
		return 'raus'


class NoWall(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.Wall])

	def execute(self, userdata):
		rospy.loginfo('Executing state NoWall')
		setMotorSpeed(0, 0)
		while Global.distance == 0:
			rospy.sleep(0.3)
		return Transitions.Wall


class NoTurn(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.WallTooClose, Transitions.WallTooFar, Transitions.InCorridor, Transitions.NoWall])

	def execute(self, userdata):
		rospy.loginfo('Executing state NoTurn')
		
		setMotorSpeed(Config.forwardSpeed, 0)
		# warten solange der roboter sich innerhalb des vorgegeben corridors befindet
		rospy.loginfo('diff = ' + repr(Global.distance-Config.desiredDistance))
		while isInCorridor():
			rospy.sleep(0.3)
			
		if Global.distance==0:
			return Transitions.NoWall
		if Global.distance < Config.desiredDistance:
			return Transitions.WallTooClose
		if Global.distance > Config.desiredDistance:
			return Transitions.WallTooFar
		return Transitions.InCorridor
	
class TurnRight(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.InCorridor, Transitions.NoWall])

	def execute(self, userdata):
		rospy.loginfo('Executing state TurnRight')
		setMotorSpeed(Config.forwardSpeed, Config.angularSpeed)
		# warten solange der roboter sich ausserhalb des vorgegeben corridors befindet
		while math.fabs(Global.distance-Config.desiredDistance) > Config.corridorWidth:
			rospy.sleep(0.3)
			
		if Global.distance==0: 
			return Transitions.NoWall
		return Transitions.InCorridor
	
class TurnLeft(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.InCorridor, Transitions.NoWall])

	def execute(self, userdata):
		rospy.loginfo('Executing state TurnLeft')
		setMotorSpeed(Config.forwardSpeed, -Config.angularSpeed)
		# warten solange der roboter sich ausserhalb des vorgegeben corridors befindet
		rospy.loginfo('fabdiff = ' + repr(math.fabs(Global.distance-Config.desiredDistance)))
		while math.fabs(Global.distance-Config.desiredDistance) > Config.corridorWidth:
			rospy.sleep(0.3)			
			
		if Global.distance==0: 
			return Transitions.NoWall
		return Transitions.InCorridor

def isInCorridor():
	return math.fabs(Global.distance-Config.desiredDistance) <= Config.corridorWidth

# werte im bereich [-1, 1]
def setMotorSpeed(linear, angular):
	# geschwindigkeitswerte fuer thruster berechnen
	left = linear*127 + angular*127
	right = linear*127 - angular*127
	# auf den wertebereich -127 bis 127 beschraenken
	left = numpy.clip(left, -127, 127)
	right = numpy.clip(right, -127, 127)
	# nachrichten an motoren publishen
	pub_motor_left.publish(sollSpeed(data = left))
	pub_motor_right.publish(sollSpeed(data = right))
	rospy.is_shutdown()
	rospy.is_shutdown()

def echoSounderCallback(msg):
	# compute wall distance
	einheit = len(msg.echoData) / float(msg.range);
	Global.distance = msg.echoData.index(max(msg.echoData)) / einheit
	#rospy.loginfo('Received echosounder message, computed distance = %s'%repr(distance))

def echoSounderAvgCallback(msg):
	Global.distance = msg.data
	
def configCallback(config, level):
	rospy.loginfo('Reconfiugre Request: ' + repr(config['desiredDistance']))
	Config.desiredDistance = config['desiredDistance']
	return config
	
if __name__ == '__main__':
	rospy.init_node('wallfollowing')
	srv = Server(WallFollowingConfig, configCallback)
	pub_motor_left = rospy.Publisher('/hanse/motors/left', sollSpeed)
	pub_motor_right = rospy.Publisher('/hanse/motors/right', sollSpeed)

	# Subscriber/Publisher
	#rospy.Subscriber('/hanse/sonar/echo', numpy_msg(ScanningSonar), echoSounderCallback)
	rospy.Subscriber('/echosounderaveragedistance', Float32, echoSounderAvgCallback)

   	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['raus'])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(States.AdjustStart, AdjustStart(),
								transitions={'test' : States.NoWall})
		smach.StateMachine.add(States.NoWall, NoWall(),
								transitions={Transitions.Wall : States.NoTurn})
		smach.StateMachine.add(States.NoTurn, NoTurn(),
								transitions={Transitions.WallTooClose : States.TurnLeft,
											Transitions.WallTooFar : States.TurnRight,
											Transitions.InCorridor : States.NoTurn,
											Transitions.NoWall : States.NoWall})
		smach.StateMachine.add(States.TurnLeft, TurnLeft(),
								transitions={Transitions.InCorridor : States.NoTurn,
											Transitions.NoWall : States.NoWall})
		smach.StateMachine.add(States.TurnRight, TurnRight(),
								transitions={Transitions.InCorridor : States.NoTurn,
											Transitions.NoWall : States.NoWall})								

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

    # Execute SMACH plan
	outcome = sm.execute()

	#rospy.spin()
	sis.stop()

