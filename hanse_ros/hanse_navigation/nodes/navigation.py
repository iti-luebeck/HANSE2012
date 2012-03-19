#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_navigation')
import rospy
import smach
import smach_ros
import math
import numpy
import collections
from hanse_navigation.cfg import NavigationConfig
from dynamic_reconfigure.server import Server
from hanse_msgs.msg import sollSpeed
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path

class Config:
	hysteresis_goal = 0
	hysteresis_heading = 0
	forward_max_speed = 0
	forward_max_dist = 0
	angular_min_speed = 0
	angular_max_speed = 0
	p_heading = 0

class Global:
	abortFlag = False
	currentHeading = 0.0
	currentPosition = Point()
	hasActiveGoal = False
	# goalzeug
	path = collections.deque() # enthaelt PoseStamped
	currentGoal = None
	headingToGoal = 0
	distanceToGoal = 0

class States:
	Idle = 'Idle'
	Paused = 'Paused'
	FailedToGotoGoal = 'FailedToGotoGoal' 
	ReachedGoal = 'ReachedGoal'
	AdjustDepth = 'AdjustDepth'
	AdjustHeading = 'AdjustHeading'
	MoveForward = 'MoveForward'

class Transitions:
	HeadingAdjusted = 'HeadingAdjusted'
	DepthAdjusted = 'DepthAdjusted'
	CloseEnoughToGoal = 'CloseEnoughToGoal'
	HasGoal = 'HasGoal'
	HeadingAdjustmentNeeded = 'HeadingAdjustmentNeeded'
	Idle = 'Idle'
	Aborted = 'Aborted'
	
class Idle(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.HasGoal])

	def execute(self, userdata):
		rospy.loginfo('Executing state Idle')	
		
		# warten bis ein pfad vorgegeben wird 
		while not rospy.is_shutdown():
			Global.abortFlag = False
			if len(Global.path) > 0:
				Global.currentGoal = Global.path.popleft()
				return Transitions.HasGoal
			rospy.sleep(0.1)					
	

class AdjustDepth(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.DepthAdjusted, Transitions.Aborted])

	def execute(self, userdata):
		rospy.loginfo('Executing state AdjustDepth')
				
		while not rospy.is_shutdown():
			# pruefen ob aktuelle navigation abgebrochen werden soll
			if Global.abortFlag:
				return Transitions.Aborted
			rospy.sleep(1.0)
			if True:
				return Transitions.DepthAdjusted

class AdjustHeading(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.HeadingAdjusted, Transitions.Aborted])

	def execute(self, userdata):
		rospy.loginfo('Executing state AdjustHeading')
		
		while not rospy.is_shutdown():
			# pruefen ob aktuelle navigation abgebrochen werden soll
			if Global.abortFlag:
				return Transitions.Aborted
			
			#diffHeading = Global.headingToGoal - Global.currentHeading
			diffHeading = calcRadiansDiff(Global.headingToGoal, Global.currentHeading)			
			# in bereich [-pi,pi) bringen
#			while diffHeading < -math.pi:
#				diffHeading += math.pi
#			while diffHeading >= math.pi:
#				diffHeading -= math.pi
			rospy.loginfo('diffHeading = ' + repr(diffHeading))

			# pruefen ob heading in akzeptablem bereich
			if math.fabs(diffHeading) < Config.hysteresis_heading: 
				return Transitions.HeadingAdjusted
			
			# angular speed berechnen (positive: rotate right (clockwise))
			maxAngSpeed = Config.angular_max_speed
			minAngSpeed = Config.angular_min_speed
			val = -Config.p_heading * diffHeading
			if val > 0: val = numpy.clip(val, minAngSpeed, maxAngSpeed)
			if val < 0: val = numpy.clip(val, -maxAngSpeed, -minAngSpeed)
			
			setMotorSpeed(0, val)
			rospy.sleep(0.1)	
		
class MoveForward(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.CloseEnoughToGoal, Transitions.HeadingAdjustmentNeeded, Transitions.Aborted])

	def execute(self, userdata):
		rospy.loginfo('Executing state MoveForward')
		
		while not rospy.is_shutdown():
			# pruefen ob aktuelle navigation abgebrochen werden soll
			if Global.abortFlag:
				return Transitions.Aborted
			
			# pruefen ob auv nah genug am 
			if closeEnoughToGoal():
				return Transitions.CloseEnoughToGoal
			
			# pruefen, ob heading korrigiert werden muss
			if calcRadiansDiff(Global.headingToGoal, Global.currentHeading) > Config.hysteresis_heading:
				return Transitions.HeadingAdjustmentNeeded
			
			forwardspeed = Config.forward_max_speed
			# Move slower if we are close to the goal.
			if Global.distanceToGoal < Config.forward_max_dist:
				forwardspeed -= 0.5 * Config.forward_max_speed * (1 - Global.distanceToGoal / Config.forward_max_dist)
			setMotorSpeed(forwardspeed, 0)
			rospy.sleep(0.1)

class ReachedGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.Idle])

	def execute(self, userdata):
		rospy.loginfo('Executing state ReachedGoal')

		setMotorSpeed(0,0)
		Global.currentGoal = None
		rospy.sleep(1)
		
		return Transitions.Idle

def closeEnoughToGoal():
	if Global.currentGoal == None:
		return False
	distanceToGoal = math.sqrt( math.pow(Global.currentPosition.x - Global.currentGoal.pose.position.x, 2)  +
					math.pow(Global.currentPosition.y - Global.currentGoal.pose.position.y, 2) )
	rospy.loginfo('distance to goal: ' + repr(distanceToGoal))
	return distanceToGoal < Config.hysteresis_goal

def imuCallback(msg):
	Global.imuCallbackCalled = True
	quater = msg.orientation
	roll, Global.currentHeading, yaw = quatToAngles(quater.x, quater.y, quater.z, quater.w)
	#rospy.loginfo('imuCallback: ' + repr(Global.currentHeading))
	
def posemeterCallback(msg):	
	Global.currentPosition = msg.pose.position
	if Global.currentGoal!=None:
		dx = Global.currentGoal.pose.position.x - Global.currentPosition.x
		dy = Global.currentGoal.pose.position.y - Global.currentPosition.y
		Global.headingToGoal = -math.atan2(dx, dy) + math.pi/2;
		Global.distanceToGoal = math.sqrt(dx*dx + dy*dy);
		rospy.loginfo('headingToGoal='+repr(Global.headingToGoal)+' ### currentHeading='+repr(Global.currentHeading))		
	
def goalCallback(msg):
	Global.abortFlag = True
	Global.path = collections.deque()
	Global.path.append(msg)
	
def pathCallback(msg):
	Global.abortFlag = True
	Global.path = collections.deque()
	for pose in msg.poses:
		Global.path.append(pose)	
		
def timerCallback(event):
	p = Path()
	p.header.frame_id = '/map'
	# aktuelle position hinzufuegen
	currentPose = PoseStamped()
	currentPose.header.frame_id = '/map'
	currentPose.pose.position = Global.currentPosition
	p.poses.append(currentPose)
	# wegpunkte hinzufuegen
	if Global.currentGoal != None:
		p.poses.append(Global.currentGoal)
	for goal in Global.path:
		p.poses.append(goal)
	pub_path.publish(p)	

def configCallback(config, level):
	rospy.loginfo('Reconfiugre Request: ' + repr(config['forward_max_speed']))
	Config.hysteresis_heading = config['hysteresis_heading']
	Config.hysteresis_goal = config['hysteresis_goal']
	Config.forward_max_speed = config['forward_max_speed']
	Config.forward_max_dist = config['forward_max_dist']
	Config.angular_min_speed = config['angular_min_speed']
	Config.angular_max_speed = config['angular_max_speed']
	Config.p_heading = config['p_heading']
	return config

# werte im bereich [-1, 1]
def setMotorSpeed(linear, angular):
	#rospy.loginfo('setMotorSpeed: ' + repr(linear) + ", " + repr(angular))
	# geschwindigkeitswerte fuer thruster berechnen
	left = linear*127 + angular*127
	right = linear*127 - angular*127
	# auf den wertebereich -127 bis 127 beschraenken
	left = numpy.clip(left, -127, 127)
	right = numpy.clip(right, -127, 127)
	# nachrichten an motoren publishen
	pub_motor_left.publish(sollSpeed(data = left))
	pub_motor_right.publish(sollSpeed(data = right))
	#rospy.is_shutdown()
	#rospy.is_shutdown()
	
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
	#return min(math.fabs(a-b), 2*math.pi - math.fabs(a-b))
	diff = b-a
	if diff > math.pi:
		diff = -(360-diff)
	elif diff < -math.pi:
 		diff = 2*math.pi+diff
	return diff

if __name__ == '__main__':
	rospy.init_node('navigation')	
	
	# Config server
	configSrv = Server(NavigationConfig, configCallback)

	# Subscriber/Publisher
	rospy.Subscriber('/hanse/imu', Imu, imuCallback)
	rospy.Subscriber('/hanse/posemeter', PoseStamped, posemeterCallback)
	rospy.Subscriber('/goal', PoseStamped, goalCallback)
	rospy.Subscriber('/waypoints', Path, pathCallback)
	pub_motor_left = rospy.Publisher('motors/left', sollSpeed)
	pub_motor_right = rospy.Publisher('motors/right', sollSpeed)
	
	pub_path = rospy.Publisher('/path', Path)
	
	# timer zum publishen des aktuellen path
	rospy.Timer(rospy.Duration(1.0), timerCallback)
	
	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=[])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(States.Idle, Idle(),
								transitions={Transitions.HasGoal : 'GotoGoal'})
		smach.StateMachine.add(States.ReachedGoal, ReachedGoal(),
								transitions={Transitions.Idle : States.Idle})								
		# Create the sub SMACH state machine 
		sm_sub = smach.StateMachine(outcomes=[Transitions.CloseEnoughToGoal, Transitions.Aborted])
        
		# Open the container 
		with sm_sub:
			# Add states to the container 
			smach.StateMachine.add(States.AdjustDepth, AdjustDepth(),
                                   transitions={Transitions.DepthAdjusted : States.AdjustHeading})
			smach.StateMachine.add(States.AdjustHeading, AdjustHeading(),
                                   transitions={Transitions.HeadingAdjusted : States.MoveForward})
			smach.StateMachine.add(States.MoveForward, MoveForward(),
                                   transitions={Transitions.HeadingAdjustmentNeeded : States.AdjustHeading})

		smach.StateMachine.add('GotoGoal', sm_sub,
							transitions={Transitions.CloseEnoughToGoal : States.ReachedGoal,
										Transitions.Aborted : States.Idle})

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

    # Execute SMACH plan
	outcome = sm.execute()
	rospy.loginfo('state machine stopped')

	#rospy.spin()
	sis.stop()
