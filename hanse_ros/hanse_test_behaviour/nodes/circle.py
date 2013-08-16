#!/usr/bin/env python

import roslib; roslib.load_manifest('hanse_test_behaviour')
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Point, Twist, Vector3
from hanse_msgs.msg import BehaviourStatus
from std_msgs.msg import String

class Global:
    name = 'circle'
    do_start = False
    do_abort = False
    pub_cmd_vel = 0
    status_pub = 0

class MoveCircle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['time_is_up', 'aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVECIRCLE')
        setMotorSpeed(0,0.5)
        start_time = rospy.get_rostime()
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():            
            if (rospy.get_rostime() - start_time).to_sec() > 30:
                publishStatus('finish')
                return 'time_is_up'
            if Global.do_abort:
                Global.do_abort = False
                return 'aborted'
            publishStatus('circling')
            rate.sleep()
				
        return 'aborted'

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        rospy.loginfo('Executing state IDLE')
        setMotorSpeed(0,0)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if Global.do_start:
                publishStatus('started')
                Global.do_start = False
                return 'start'
            publishStatus('idle')
            rate.sleep()
				
        return 'start'

def publishStatus(behaviour_status):
    status_msg = BehaviourStatus(name=String(Global.name), status=String(behaviour_status))
    Global.status_pub.publish(status_msg)

def setMotorSpeed(lin, ang):
    twist = Twist(linear=Vector3(x=lin,z=0), angular=Vector3(z=ang))
    Global.pub_cmd_vel.publish(twist)
    
def statusCallback(msg):
    if msg.name.data == Global.name:
        if msg.status.data == 'start':
            Global.do_start = True
        if msg.status.data == 'abort':
            Global.do_abort = True

if __name__ == '__main__':
    rospy.init_node('circle_behaviour')
    
    Global.pub_cmd_vel = rospy.Publisher('/hanse/commands/cmd_vel_behaviour', Twist)
    Global.status_pub = rospy.Publisher('/hanse/behaviourstatus', BehaviourStatus)    
    rospy.Subscriber('/hanse/behaviourstatus', BehaviourStatus, statusCallback)

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes={})
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('IDLE', Idle(),
                               transitions={'start': 'WORK'})

        # Create the sub SMACH state machine
        sm_work = smach.StateMachine(outcomes={'time_is_up', 'aborted'})

        # Open the container
        with sm_work:

            # Add states to the container
            smach.StateMachine.add('MOVECIRCLE', MoveCircle())

        smach.StateMachine.add('WORK', sm_work,
                               transitions={'time_is_up':'IDLE','aborted':'IDLE'})

    # Execute SMACH plan
    outcome = sm_top.execute()
    
