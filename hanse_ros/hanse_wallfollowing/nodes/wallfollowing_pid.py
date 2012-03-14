#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_wallfollowing')
import rospy
import smach
import smach_ros
import numpy
import math
import sys
from rospy.numpy_msg import numpy_msg
from dynamic_reconfigure.server import Server
from hanse_wallfollowing.cfg import WallFollowingConfig
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import Imu
from hanse_msgs.msg import EchoSounder, ScanningSonar, sollSpeed
from hanse_pidcontrol.srv import *

class Config:
	desiredDistance = 1.5
	maxSpeed = 0.7
	echoSounderAngle = 0.0

class Global:
	imuCallbackCalled = False
	currentDistance = 0.0
	currentHeading = 0.0
	scanningSonarDistance = 0.0
	angularSpeedOutput = 0.0

class States:
	Init = 'Init'
	Align = 'Align'
	FollowWall = 'FollowWall'
	NoWall = 'NoWall'

class Transitions:
	Wall = 'Wall'
	NoWall = 'NoWall'
	InitFinished = 'InitFinished'
	AlignFinished = 'AlignFinished'
	Ended = 'Ended'

# In diesem Zustand passieren Initialisierungen
class Init(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.InitFinished])

	def execute(self, userdata):
		rospy.loginfo('Executing state Init')

		# Auf Service warten und PID-Regler aktivieren
		rospy.loginfo('Waiting for pid service...')
		rospy.wait_for_service('/wallfollowing_angular_pid/enable')
		enable = rospy.ServiceProxy('/wallfollowing_angular_pid/enable', Enable)
		enable(True)

		# Warten bis es eine Imu-Message gab (damit Orientierung des AUV bekannt ist)
		rospy.loginfo('Waiting for imu message...')
		while not Global.imuCallbackCalled:
			rospy.sleep(0.2)

		return Transitions.InitFinished


# In diesem Zustand wird das auv parallel zur nahsten Wand ausgerichtet
class Align(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.AlignFinished,Transitions.NoWall])

	def execute(self, userdata):
		rospy.loginfo('Executing state Align')

		# 360 grad drehen und nahste wand suchen
		initHeading = Global.currentHeading
		lastHeading = initHeading
		rotated = 0.0
		smallestDistanceHeading = 0.0
		smallestDistance = sys.maxint
		setMotorSpeed(0, 1)
		rospy.loginfo('initHeading: ' + repr(initHeading))

		#rospy.loginfo('math.fabs(initHeading - currentHeading))
		while math.fabs(initHeading - Global.currentHeading) > 0.15 or rotated < 2*math.pi:
			#rospy.loginfo('rotated: ' + repr(rotated))
			#rospy.loginfo('distance: ' + repr(Global.currentDistance))			
			rotated += calcRadiansDiff(lastHeading, Global.currentHeading)
			lastHeading = Global.currentHeading
			if Global.currentDistance > 0.0 and Global.currentDistance < smallestDistance:
				rospy.loginfo('new smallestDistanceHeading: ' + repr(smallestDistanceHeading))
				smallestDistance = Global.currentDistance
				smallestDistanceHeading = Global.currentHeading
			rospy.sleep(0.1)

		if smallestDistance==sys.maxint:
			rospy.logwarn('keine wand in sicht')
			return Transitions.NoWall
		else:
			while math.fabs(smallestDistanceHeading - Global.currentHeading) > 0.15:
				rospy.loginfo(repr(smallestDistanceHeading) + '  -  ' + repr(Global.currentHeading))
				rospy.sleep(0.1)
			
		setMotorSpeed(0, 0)
		
		return Transitions.AlignFinished

 
class NoWall(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.Wall])

	def execute(self, userdata):
		rospy.loginfo('Executing state NoWall')
		setMotorSpeed(0, 1)
		while Global.currentDistance==0:
			rospy.sleep(0.1)
		setMotorSpeed(0, 0)
		return Transitions.Wall
		

# In diesem Zustand wird der Wand gefolgt
class FollowWall(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.NoWall, Transitions.Ended])

	def execute(self, userdata):
		rospy.loginfo('Executing state NoTurn')
		
		# pid geregelten angular wert benutzen
		rospy.loginfo('diff = ' + repr(Global.currentDistance-Config.desiredDistance))
		while Global.currentDistance != 0.0 and not rospy.is_shutdown():
			linearSpeed = -(Config.maxSpeed-0.1) * math.pow(Global.angularSpeedOutput,6) + Config.maxSpeed
			setMotorSpeed(linearSpeed, Global.angularSpeedOutput) # Config.maxSpeed-(Config.maxSpeed-0.1)*math.fabs(Global.angularSpeedOutput)
			rospy.sleep(0.1)

		if rospy.is_shutdown():
			return Transitions.Ended
		return Transitions.NoWall

# werte im bereich [-1, 1]
def setMotorSpeed(linear, angular):
	#rospy.loginfo('setMotorSpeed(' + repr(linear) + ', '  +repr(angular))
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

#def echoSounderCallback(msg):
#	# compute wall distance
#	unit = len(msg.echoData) / float(msg.range)
#	dist = msg.echoData.index(max(msg.echoData)) / unit
#	# abstand zur wand berechnen unter beachtung des drehwinkels des echosounders
#	Global.currentDistance = dist * math.cos(Config.echoSounderAngle)
#	rospy.loginfo('Received echosounder message, computed distance = %s'%repr(Global.currentDistance))
	
def echoSounderAvgCallback(msg):
	Global.currentDistance = msg.data

def scanningSonarCallback(msg):
	unit = len(msg.echoData) / float(msg.range)
	if math.fabs(msg.headPosition) < 0.1:
		dist = msg.echoData.index(max(msg.echoData)) / unit
		rospy.loginfo('SCANNING SONAR DISTANCE: ' + repr(dist))
		
def imuCallback(msg):
	Global.imuCallbackCalled = True
	quater = msg.orientation
	roll, Global.currentHeading, yaw = quatToAngles(quater.x, quater.y, quater.z, quater.w)
	#rospy.loginfo('imuCallback: ' + repr(Global.currentHeading))

def configCallback(config, level):
	rospy.loginfo('Reconfiugre Request: ' + repr(config['desiredDistance']))
	Config.desiredDistance = config['desiredDistance']
	Config.maxSpeed = config['maxSpeed']
	return config

def timerCallback(event):
    #print 'Timer called at ' + str(event.current_real)
    pub_angular_target.publish(Float64(data = Config.desiredDistance))
    pub_angular_input.publish(Float64(data = Global.currentDistance))

def angularPidOutputCallback(msg):
	#rospy.loginfo('angular pid output: ' + repr(msg.data))
	Global.angularSpeedOutput = -msg.data

# Gibt RPY fuer ein Quaternion zurueck
def quatToAngles(x,y,z,w):
	angles = [0,0,0]
	sqw = w * w
	sqx = x * x
	sqy = y * y
	sqz = z * z
	unit = sqx + sqy + sqz + sqw # if normalized is one, otherwise is correction factor
	test = x * y + z * w
	if test > 0.499 * unit: # singularity at north pole
		angles[1] = 2 * math.atan2(x, w)
		angles[2] = 0.5*math.pi
		angles[0] = 0
	elif test < -0.499 * unit: # singularity at south pole
		angles[1] = -2 * FastMath.atan2(x, w)
		angles[2] = -0.5*math.pi
		angles[0] = 0
	else:
		angles[1] = math.atan2(2 * y * w - 2 * x * z, sqx - sqy - sqz + sqw) #roll or heading
		angles[2] = math.asin(2 * test / unit) # pitch or attitude
		angles[0] = math.atan2(2 * x * w - 2 * y * z, -sqx + sqy - sqz + sqw) # yaw or bank	
	return angles

def calcRadiansDiff(a, b):
	while a <= 0: a += 2*math.pi
	while b <= 0: b += 2*math.pi
	while math.fabs(a-b) > math.pi:
		if a < b:
			a += 2*math.pi
		else:
			b += 2*math.pi
	return math.fabs(a-b)


if __name__ == '__main__':
	rospy.init_node('wallfollowing')	
	
	# Config server
	configSrv = Server(WallFollowingConfig, configCallback)

	# Subscriber/Publisher
	#rospy.Subscriber('/hanse/sonar/echo', numpy_msg(EchoSounder), echoSounderCallback)
	rospy.Subscriber('/echosounderaveragedistance', Float32, echoSounderAvgCallback)
	rospy.Subscriber('/hanse/sonar/scan', numpy_msg(ScanningSonar), scanningSonarCallback)
	rospy.Subscriber('/hanse/imu', Imu, imuCallback)
	pub_motor_left = rospy.Publisher('/hanse/motors/left', sollSpeed)
	pub_motor_right = rospy.Publisher('/hanse/motors/right', sollSpeed)
	pub_angular_target = rospy.Publisher('/wallfollowing_angular_pid/target', Float64)
	pub_angular_input = rospy.Publisher('/wallfollowing_angular_pid/input', Float64)
	rospy.Subscriber('/wallfollowing_angular_pid/output', Float64, angularPidOutputCallback)
	
	# pid timer
	rospy.Timer(rospy.Duration(1.0 / 10), timerCallback)

   	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=[Transitions.Ended])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(States.Init, Init(),
								transitions={Transitions.InitFinished : States.NoWall})
		smach.StateMachine.add(States.Align, Align(),
								transitions={Transitions.AlignFinished : States.FollowWall,
											Transitions.NoWall : States.NoWall})
		smach.StateMachine.add(States.NoWall, NoWall(),
								transitions={Transitions.Wall : States.FollowWall})
		smach.StateMachine.add(States.FollowWall, FollowWall(),
								transitions={Transitions.NoWall : States.NoWall})							

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

    # Execute SMACH plan
	outcome = sm.execute()
	rospy.loginfo('state machine stopped')
	
	# auv stoppen
	setMotorSpeed(0, 0)

	#rospy.spin()
	sis.stop()
