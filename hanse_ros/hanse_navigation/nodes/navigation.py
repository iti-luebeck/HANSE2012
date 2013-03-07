#!/usr/bin/env python
import roslib; roslib.load_manifest('hanse_navigation')
import rospy
import smach
import smach_ros
import math
import numpy
import collections
import actionlib
import tf 
from tf.transformations import euler_from_quaternion
from hanse_navigation.msg import NavigateAction, NavigateFeedback, NavigateResult
from hanse_navigation.cfg import NavigationConfig
from dynamic_reconfigure.server import Server
from hanse_msgs.msg import sollSpeed
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import PoseStamped, Point, Twist, Vector3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path

# konfigurierbare werte, werden durch dynamic_reconfigure gesetzt  
class Config:
	hysteresis_goal = 0
	hysteresis_heading = 0
	forward_max_speed = 0
	forward_max_dist = 0
	angular_min_speed = 0
	angular_max_speed = 0
	p_heading = 0
        simulator = False

# variablen, die in mehreren zustaenden verwendet werden
class Global:
	abortFlag = False
	currentHeading = 0.0
	currentPosition = Point()
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
	Exit = 'Exit'
	
class Idle(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.HasGoal, Transitions.Exit])

	def execute(self, userdata):
		rospy.loginfo('Executing state Idle')
		
		# beim idlen soll das auv sich nicht bewegen
		setMotorSpeed(0,0)
		
		# warten bis ein pfad vorgegeben wird
		while not rospy.is_shutdown():
			Global.abortFlag = False
			if len(Global.path) > 0:
				Global.currentGoal = Global.path.popleft()
				return Transitions.HasGoal
			rospy.sleep(0.1)			
		
		return Transitions.Exit

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
		smach.State.__init__(self, outcomes=[Transitions.HeadingAdjusted, Transitions.Aborted, Transitions.Exit])

	def execute(self, userdata):
		rospy.loginfo('Executing state AdjustHeading')
		
		while not rospy.is_shutdown():
			# pruefen ob aktuelle navigation abgebrochen werden soll
			if Global.abortFlag:
				return Transitions.Aborted
			
			#diffHeading = Global.headingToGoal - Global.currentHeading
			diffHeading = normalize_angle(Global.headingToGoal - Global.currentHeading)
			rospy.loginfo('diffHeading = ' + repr(diffHeading))

			# pruefen ob heading in akzeptablem bereich ist
			if math.fabs(diffHeading) < Config.hysteresis_heading: 
				return Transitions.HeadingAdjusted
			
			# angular speed berechnen (positive: rotate left (counter clockwise))
			maxAngSpeed = Config.angular_max_speed
			minAngSpeed = Config.angular_min_speed
			val = Config.p_heading * diffHeading
			if val > 0: val = numpy.clip(val, minAngSpeed, maxAngSpeed)
			if val < 0: val = numpy.clip(val, -maxAngSpeed, -minAngSpeed)
			
			if Config.simulator:
				setMotorSpeed(0, -val)
			else:
				setMotorSpeed(0, val)
			rospy.sleep(0.1)

		return Transitions.Exit
		
class MoveForward(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.CloseEnoughToGoal, Transitions.HeadingAdjustmentNeeded, Transitions.Aborted, Transitions.Exit])

	def execute(self, userdata):
		rospy.loginfo('Executing state MoveForward')
		
		while not rospy.is_shutdown():
			# pruefen ob aktuelle navigation abgebrochen werden soll
			if Global.abortFlag:
				return Transitions.Aborted
			
			# pruefen ob auv nah genug am ziel ist
			if closeEnoughToGoal():
				return Transitions.CloseEnoughToGoal
			
			# pruefen, ob heading korrigiert werden muss
			if math.fabs(normalize_angle(Global.headingToGoal - Global.currentHeading)) > Config.hysteresis_heading:
				return Transitions.HeadingAdjustmentNeeded
			
			forwardspeed = Config.forward_max_speed
			# Move slower if we are close to the goal.
			if Global.distanceToGoal < Config.forward_max_dist:
				forwardspeed -= 0.5 * Config.forward_max_speed * (1 - Global.distanceToGoal / Config.forward_max_dist)
			setMotorSpeed(forwardspeed, 0)
			rospy.sleep(0.1)

		return Transitions.Exit

class ReachedGoal(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=[Transitions.Idle])

	def execute(self, userdata):
		rospy.loginfo('Executing state ReachedGoal')

		setMotorSpeed(0,0)
		Global.currentGoal = None		
		Global.actionServer.set_succeeded() # action als erfolgreich abgeschlossen markieren
		
		return Transitions.Idle

def closeEnoughToGoal():
	if Global.currentGoal == None:
		return False
	distanceToGoal = math.sqrt( math.pow(Global.currentPosition.x - Global.currentGoal.pose.position.x, 2)  +
					math.pow(Global.currentPosition.y - Global.currentGoal.pose.position.y, 2) )
	rospy.loginfo('distance to goal: ' + repr(distanceToGoal))
	return distanceToGoal < Config.hysteresis_goal

# die orientierung wird aus der imu ermittelt
#def imuCallback(msg):
#	q = msg.orientation	
#	(yaw,pitch,roll) = euler_from_quaternion([q.w, q.x, q.y, q.z])
#	Global.currentHeading = yaw
	#rospy.loginfo('imuCallback: ' + repr(Global.currentHeading))

# die aktuelle position wird aus posemeter ausgelesen
def positionCallback(msg):	
	Global.currentPosition = msg.pose.position
	# wenn zur zeit zu einem ziel navigiert wird, relevante werte aktualisieren
	if Global.currentGoal!=None:
		dx = Global.currentGoal.pose.position.x - Global.currentPosition.x
		dy = Global.currentGoal.pose.position.y - Global.currentPosition.y
		Global.headingToGoal = normalize_angle(math.atan2(dy, dx))
		Global.distanceToGoal = math.sqrt(dx*dx + dy*dy)
		rospy.loginfo('headingToGoal='+repr(Global.headingToGoal)+' ### currentHeading='+repr(Global.currentHeading))		
#	if not Config.simulator:
		q = msg.pose.orientation	
		(roll,pitch,yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
		Global.currentHeading = yaw
	
def goalCallback(msg):
	Global.actionServer.set_aborted()
	Global.abortFlag = True
	Global.path = collections.deque()
	Global.path.append(msg)
	
#def pathCallback(msg):
#	Global.abortFlag = True
#	Global.path = collections.deque()
#	for pose in msg.poses:
#		Global.path.append(pose)	

# publisht das aktuelle ziel
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
        Config.simulator = config['simulator']
	return config

# werte im bereich [-1, 1]
def setMotorSpeed(lin, ang):
	rospy.loginfo("angularoutput: " + repr(ang))
	twist = Twist(linear=Vector3(x=lin,z=0), angular=Vector3(z=ang))
	pub_cmd_vel.publish(twist)
	#left = lin*127 + ang*127
	#right = lin*127 - ang*127
	# auf den wertebereich -127 bis 127 beschraenken
	#left = numpy.clip(left, -127, 127)
	#right = numpy.clip(right, -127, 127)
	# nachrichten an motoren publishen
	#pub_motor_left.publish(sollSpeed(data = left))
	#pub_motor_right.publish(sollSpeed(data = right))

# aus dem angles-package uebernommen
def normalize_angle_positive(angle):
	return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi)

# aus dem angles-package uebernommen
def normalize_angle(angle):
	a = normalize_angle_positive(angle)
	if a > math.pi:
		a -= 2.0 *math.pi
	return a

class NavigateActionServer(object):
	# create messages that are used to publish feedback/result
	_feedback = NavigateFeedback()
	_result   = NavigateResult()

	def __init__(self, name):
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, NavigateAction, execute_cb=self.execute_cb)
		self._as.start()
		
	def set_succeeded(self):
		rospy.loginfo('NavigateActionServer.set_succeeded')
		self._result.successful = True
		self._as.set_succeeded(self._result)

	def set_aborted(self):
		rospy.loginfo('NavigateActionServer.set_aborted')
		Global.abortFlag = True
		self._as.set_aborted()
		Global.path = collections.deque()
		Global.currentGoal = None
    
	def execute_cb(self, goal):
		rospy.loginfo('NavigateActionServer.execute_cb' + repr(goal))
		
		Global.path = collections.deque()
		Global.path.append(goal.goal)
		
		# schleife wird erst verlassen, wenn der actionserver nicht mehr aktiv ist oder das aktuelle goal gewechselt hat
		rospy.loginfo('is_active='+repr(self._as.is_active()) + ' ### ' + repr(self._as.current_goal.get_goal()==goal))
		while self._as.is_active() and self._as.current_goal.get_goal()==goal:
			rospy.loginfo('is_active='+repr(self._as.is_active()) + ' ### ' + repr(self._as.current_goal.get_goal()==goal))
			# bei preempt das aktuelle goal abbrechen
			if self._as.is_preempt_requested():
				self.set_aborted()
				return
			# publish aktuelle position als feedback
			pose = PoseStamped()
			pose.pose.position = Global.currentPosition
			self._feedback.current_position = pose
			self._as.publish_feedback(self._feedback)
			rospy.sleep(0.2)
			
		return ''

if __name__ == '__main__':
	rospy.init_node('navigation')	
	
	# actionserver starten
	Global.actionServer = NavigateActionServer(rospy.get_name())
	
	# Config server
	configSrv = Server(NavigationConfig, configCallback)

	# Subscriber/Publisher
		
	if Config.simulator:
#		rospy.Subscriber('imu', Imu, imuCallback)
		rospy.Subscriber('posemeter', PoseStamped, positionCallback)
	else:
		rospy.Subscriber('position/estimate', PoseStamped, positionCallback)

	rospy.Subscriber('/goal', PoseStamped, goalCallback)
#	rospy.Subscriber('/waypoints', Path, pathCallback)
	pub_cmd_vel = rospy.Publisher('commands/cmd_vel_behaviour', Twist)
	#pub_motor_left = rospy.Publisher('/hanse/motors/left', sollSpeed)
	#pub_motor_right = rospy.Publisher('/hanse/motors/right', sollSpeed)	
	
	pub_path = rospy.Publisher('/path', Path)
	
	# timer zum publishen des aktuellen path
	rospy.Timer(rospy.Duration(1.0), timerCallback)
	
	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=[Transitions.Exit])

	# Open the container
	with sm:
		# Add states to the container
		smach.StateMachine.add(States.Idle, Idle(),
								transitions={Transitions.HasGoal : 'GotoGoal'})
		smach.StateMachine.add(States.ReachedGoal, ReachedGoal(),
								transitions={Transitions.Idle : States.Idle})								
		# Create the sub SMACH state machine 
		sm_sub = smach.StateMachine(outcomes=[Transitions.CloseEnoughToGoal, Transitions.Aborted, Transitions.Exit])
        
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
