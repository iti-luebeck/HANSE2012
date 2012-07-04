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
from hanse_wallfollowing.msg import WallFollowingAction
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from hanse_msgs.msg import ScanningSonar, sollSpeed
from hanse_pidcontrol.srv import *

class Config:
	desiredDistance = 1.5
	maxSpeed = 0.7
	linearSpeedScaleDownFactor = 0.75
	#echoSounderAngle = 0.0

class Global:
	imuCallbackCalled = False
	noWallCounter = 0
	noWall = False
	currentDistance = 0.0
	currentHeadPosition = 0.0
	scanningSonarDistance = 0.0
	angularSpeedOutput = 0.0


#==============================================================================
# Constants
#==============================================================================
class States:
	#Align = 'Align'
	FollowWall = 'FollowWall'
	NoWall = 'NoWall'

class Transitions:
	Wall = 'Wall'
	NoWall = 'NoWall'
	InitFinished = 'InitFinished'
	AlignFinished = 'AlignFinished'
	Aborted = 'Aborted'


#==============================================================================
# State classes
#==============================================================================
class AbortableState(smach.State):
	def abort(self):
		setMotorSpeed(0,0)
		self.service_preempt()
		return Transitions.Aborted


# Auf der Stelle drehen und warten bis eine Wand zu sehen ist
class NoWall(AbortableState):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.Wall, Transitions.Aborted])

	def execute(self, userdata):
		rospy.loginfo('Executing state NoWall')		

		while not rospy.is_shutdown() and not self.preempt_requested():			
			setMotorSpeed(0, 0.5)
			if not Global.noWall:
				return Transitions.Wall
			rospy.sleep(0.2)

		return self.abort()
		

# In diesem Zustand wird der Wand gefolgt
class FollowWall(AbortableState):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.NoWall, Transitions.Aborted])

	def execute(self, userdata):
		rospy.loginfo('Executing state FollowWall')
		
#IMUTMP		startHeadPostion = Global.currentHeadPosition
		
		# pid geregelten angular wert benutzen
		rospy.loginfo('diff = ' + repr(Global.currentDistance-Config.desiredDistance))
		while not rospy.is_shutdown() and not self.preempt_requested():
			if Global.noWall:
				return Transitions.NoWall

			# lineare geschwindigkeit in abhaengigkeit von der abweichung von desiredDistance			
			abweichung = math.fabs(Global.currentDistance-Config.desiredDistance)
			linearSpeed = Config.maxSpeed - Config.linearSpeedScaleDownFactor * (abweichung / (Config.desiredDistance-0.2))
			linearSpeed = numpy.clip(linearSpeed, 0.0, Config.maxSpeed)
			angularSpeed = Global.angularSpeedOutput
			
#IMUTMP			if abweichung < 0.5:
#IMUTMP				startHeadPostion = Global.currentHeadPosition
			
#IMUTMP			if Global.angularSpeedOutput==1.0 and Global.currentDistance > Config.desiredDistance and calcRadiansDiff(startHeadPostion, Global.currentHeadPosition) > math.pi/8:
#IMUTMP				rospy.loginfo('nicht weiter drehen')
#IMUTMP				setMotorSpeed(Config.maxSpeed, 0)
#IMUTMP				while Global.currentDistance > Config.desiredDistance+0.1 and not Global.noWall:
#IMUTMP					rospy.sleep(0.1)
#IMUTMP				angularSpeed = numpy.clip(angularSpeed, -1.0, 0)
#IMUTMP				linearSpeed = Config.maxSpeed				
				
			setMotorSpeed(linearSpeed, angularSpeed) # Config.maxSpeed-(Config.maxSpeed-0.1)*math.fabs(Global.angularSpeedOutput)
			rospy.sleep(0.1)

		return self.abort();
			

#==============================================================================
# Callback functions
#==============================================================================
def echoSounderAvgCallback(msg):
	if msg.data == 0.0:
		Global.noWallCounter = Global.noWallCounter + 1
		if Global.noWallCounter > 3:
			Global.noWall = True
			#rospy.loginfo("NOWALL")
	else:
		Global.noWallCounter = 0
		Global.noWall = False
		Global.currentDistance = msg.data

def scanningSonarCallback(msg):
	unit = len(msg.echoData) / float(msg.range)
	if math.fabs(msg.headPosition) < 0.1:
		dist = msg.echoData.index(max(msg.echoData)) / unit
		rospy.loginfo('SCANNING SONAR DISTANCE: ' + repr(dist))
		
def imuCallback(msg):
	Global.imuCallbackCalled = True
	quater = msg.orientation
	roll, Global.currentHeadPosition, yaw = quatToAngles(quater.x, quater.y, quater.z, quater.w)
	#rospy.loginfo('imuCallback: ' + repr(Global.currentHeadPosition))

def configCallback(config, level):
	rospy.loginfo('Reconfiugre Request: ' + repr(config['desiredDistance']))
	Config.desiredDistance = config['desiredDistance']
	Config.maxSpeed = config['maxSpeed']
	Config.linearSpeedScaleDownFactor = config['linearSpeedScaleDownFactor']
	return config

def timerCallback(event):
    pub_angular_target.publish(Float64(data = Config.desiredDistance))
    pub_angular_input.publish(Float64(data = Global.currentDistance))

def angularPidOutputCallback(msg):
	Global.angularSpeedOutput = -msg.data

#==============================================================================
# Helper functions
#==============================================================================
# werte im bereich [-1, 1]
def setMotorSpeed(lin, ang):
	#rospy.loginfo("angularoutput: " + repr(-ang))
	#twist = Twist(linear=Vector3(x=lin,z=0), angular=Vector3(z=-ang))
	#pub_cmd_vel.publish(twist)
		# geschwindigkeitswerte fuer thruster berechnen
	left = lin*127 + ang*127
	right = lin*127 - ang*127
	# auf den wertebereich -127 bis 127 beschraenken
	left = numpy.clip(left, -127, 127)
	right = numpy.clip(right, -127, 127)
	# nachrichten an motoren publishen
	pub_motor_left.publish(sollSpeed(data = left))
	pub_motor_right.publish(sollSpeed(data = right))	

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
		angles[1] = -2 * math.atan2(x, w)
		angles[2] = -0.5*math.pi
		angles[0] = 0
	else:
		angles[1] = math.atan2(2 * y * w - 2 * x * z, sqx - sqy - sqz + sqw) #roll or heading
		angles[2] = math.asin(2 * test / unit) # pitch or attitude
		angles[0] = math.atan2(2 * x * w - 2 * y * z, -sqx + sqy - sqz + sqw) # yaw or bank	
	return angles

def calcRadiansDiff(a, b):
	return min(math.fabs(a-b), 2*math.pi - math.fabs(a-b))


#==============================================================================
# main
#==============================================================================
if __name__ == '__main__':
	rospy.init_node('wallfollowing')

	# Auf Service warten und PID-Regler aktivieren
	rospy.loginfo('Init: Waiting for pid service...')
	rospy.wait_for_service('/wallfollowing_angular_pid/enable')
	enable = rospy.ServiceProxy('/wallfollowing_angular_pid/enable', Enable)
	enable(True)
	
	# Config server
	configSrv = Server(WallFollowingConfig, configCallback)

	# Subscriber/Publisher
	rospy.Subscriber('/echosounderaveragedistance', Float32, echoSounderAvgCallback)
#	rospy.Subscriber('sonar/scan', numpy_msg(ScanningSonar), scanningSonarCallback)
#IMUTMP	rospy.Subscriber('imu', Imu, imuCallback)
	#pub_cmd_vel = rospy.Publisher('commands/cmd_vel', Twist)
	pub_motor_left = rospy.Publisher('/hanse/motors/left', sollSpeed)
	pub_motor_right = rospy.Publisher('/hanse/motors/right', sollSpeed)

	pub_angular_target = rospy.Publisher('/wallfollowing_angular_pid/target', Float64)
	pub_angular_input = rospy.Publisher('/wallfollowing_angular_pid/input', Float64)
	rospy.Subscriber('/wallfollowing_angular_pid/output', Float64, angularPidOutputCallback)
	
	# pid timer
	rospy.Timer(rospy.Duration(1.0 / 10), timerCallback)

   	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=[Transitions.Aborted])

	# Open the container
	with sm:
		# Add states to the container
#		smach.StateMachine.add(States.Align, Align(),
#								transitions={Transitions.AlignFinished : States.FollowWall,
#											Transitions.NoWall : States.NoWall})
		smach.StateMachine.add(States.NoWall, NoWall(),
								transitions={Transitions.Wall : States.FollowWall})
		smach.StateMachine.add(States.FollowWall, FollowWall(),
								transitions={Transitions.NoWall : States.NoWall})							
								
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/WALLFOLLOWING')
	sis.start()

	# Construct action server wrapper
	asw = smach_ros.ActionServerWrapper(
	    rospy.get_name(), WallFollowingAction,
	    wrapped_container = sm,
	    succeeded_outcomes = [],
	    aborted_outcomes = [Transitions.Aborted],
	    preempted_outcomes = [])

	# Run the server
	asw.run_server()

	rospy.spin()
	sis.stop()
