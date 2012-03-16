#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_navigation')
import rospy
import smach
import smach_ros
import math
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Imu

class Config:
	hysteresis_goal = 0
	hysteresis_heading = 0
	forward_max_speed = 0
	forward_max_dist = 0
	angular_min_speed = 0
	angular_max_speed = 0
	p_heading = 0

class Global:
	currentHeadPosition = 0.0
	currentPosition = Point()
	hasActiveGoal = False
	# goalzeug
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
	CloseEnoughToGoal = 'CloseEnoughToGoal'
	HasGoal = 'HasGoal'
	
class Idle(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.HasGoal])

	def execute(self, userdata):
		rospy.loginfo('Executing state Idle')
		while not rospy.is_shutdown():
			if Global.currentGoal!=None:
				return Transitions.HasGoal
			rospy.sleep(0.1)					
	
#class GotoGoal(smach.State):
#	def __init__(self):
#		smach.State.__init__(self, outcomes=[])
#
#	def execute(self, userdata):
#		rospy.loginfo('Executing state GotoGoal')
#		Global.hasActiveGoal = True
#				
#		# zu headingToGoal drehen
#		...
#		# geradeaus fahren
#		...

class AdjustDepth(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[])

	def execute(self, userdata):
		rospy.loginfo('Executing state AdjustDepth')

class AdjustHeading(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.HeadingAdjusted])

	def execute(self, userdata):
		rospy.loginfo('Executing state AdjustHeading')
		
		while not rospy.is_shutdown():
			# pruefen ob heading in akzeptablem bereich
			if math.fabs(diffHeading) > Config.hysteresis_heading: 
				return Transitions.HeadingAdjusted
			
			diffHeading = Global.GeadingToGoal - Global.currentHeading
			# in bereich [-pi,pi) bringen
			if diffHeading < math.pi:
				diffHeading += math.pi
			# angular speed berechnen (positive: rotate right (clockwise))
			maxAngSpeed = Config.angular_max_speed
			minAngSpeed = Config.angular_min_speed
			val = -Config.p_heading * diffHeading
			if val > maxAngSpeed: val = maxAngSpeed;
			if val < -maxAngSpeed: val = -maxAngSpeed;
			if val > 0 and val < minAngSpeed: val = minAngSpeed;
			if val < 0 and val > -minAngSpeed: val = -minAngSpeed;

			setMotorSpeed(0, val)
			rospy.sleep(0.1)	
		
class MoveForward(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.CloseEnoughToGoal])

	def execute(self, userdata):
		rospy.loginfo('Executing state MoveForward')
		
		while not rospy.is_shutdown():
			if closeEnoughToGoal():
				return Transitions.CloseEnoughToGoal
			
			forwardspeed = Config.forward_max_speed
			# Move slower if we are close to the goal.
			if Global.distanceToGoal < Config.forward_max_dist:
				forwardspeed -= 0.5 * Config.forward_max_speed * (1 - Global.distanceToGoal / Config.forward_max_dist)
			setMotorSpeed(forwardspeed, 0)
			rospy.sleep(0.1)

class ReachedGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[])

	def execute(self, userdata):
		rospy.loginfo('Executing state ReachedGoal')

def closeEnoughToGoal():             
	return math.sqrt( (Global.currentPosition.x - Global.currentGoal.x) * (Global.currentPosition.x- Global.currentGoal.x) +
					(Global.currentPosition.y - Global.currentGoal.y) * (Global.currentPosition.y - Global.currentGoal.y) )	< Config.hysteresis_goal

def imuCallback(msg):
	Global.imuCallbackCalled = True
	quater = msg.orientation
	roll, Global.currentHeadPosition, yaw = quatToAngles(quater.x, quater.y, quater.z, quater.w)
	#rospy.loginfo('imuCallback: ' + repr(Global.currentHeadPosition))
	
def posemeterCallback(msg):	
	Global.currentPosition = msg.pose.position
	#rospy.loginfo('posemeterCallback: ' + repr(Global.currentPosition))
	#
	dx = currentGoal.posX - Global.currentPosition.x
	dy = currentGoal.posY - Global.currentPosition.y
	Global.headingToGoal = math.atan2(dx, dy) + math.pi/2;
	Global.distanceToGoal = math.sqrt(dx*dx + dy*dy);
	
def goalCallback(msg):	
	Global.currentGoal = msg.pose.position
	rospy.loginfo('goalCallback: ' + repr(Global.currentGoal))	

def configCallback(config, level):
	rospy.loginfo('Reconfiugre Request: ' + repr(config['desiredDistance']))
	Config.hysteresis_heading = config['hysteresis_heading']
	Config.hysteresis_goal = config['hysteresis_goal']
	Config.forward_max_speed = config['forward_max_speed']
	Config.forward_max_dist = config['forward_max_dist']
	Config.angular_min_speed = config['angular_min_speed']
	Config.angular_max_speedl = config['angular_max_speed']
	Config.p_heading = config['p_heading']
	return config

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

if __name__ == '__main__':
	rospy.init_node('navigation')	
	
	# Config server
#	configSrv = Server(WallFollowingConfig, configCallback)

	# Subscriber/Publisher
	rospy.Subscriber('/hanse/imu', Imu, imuCallback)
	rospy.Subscriber('/hanse/posemeter', PoseStamped, posemeterCallback)
	rospy.Subscriber('/goal', PoseStamped, goalCallback)
	
	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=[])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(States.Idle, Idle(),
								transitions={Transitions.HasGoal : 'GotoGoal'})
		smach.StateMachine.add(States.ReachedGoal, ReachedGoal(),
								transitions={})								
		# Create the sub SMACH state machine 
		sm_sub = smach.StateMachine(outcomes=[Transitions.CloseEnoughToGoal])
        
		# Open the container 
		with sm_sub:
			# Add states to the container 
			smach.StateMachine.add(States.AdjustDepth, AdjustDepth(),
                                   transitions={})
			smach.StateMachine.add(States.AdjustHeading, AdjustHeading(),
                                   transitions={Transitions.HeadingAdjusted : States.MoveForward})
			smach.StateMachine.add(States.MoveForward, MoveForward(),
                                   transitions={})

			smach.StateMachine.add('GotoGoal', sm_sub,
							transitions={Transitions.CloseEnoughToGoal : States.ReachedGoal})

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

    # Execute SMACH plan
	outcome = sm.execute()
	rospy.loginfo('state machine stopped')

	#rospy.spin()
	sis.stop()
