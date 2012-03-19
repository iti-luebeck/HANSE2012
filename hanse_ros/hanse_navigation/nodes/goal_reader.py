#!/usr/bin/env python
import roslib
roslib.load_manifest('hanse_navigation')
import rospy

import tf
import actionlib

from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path

import yaml

class GoalReader:
	def __init__(self, filename):
		with open(filename, 'r') as filename:
			data = yaml.safe_load(filename)
		self.goal_list = data['list']
		rospy.logdebug("Goal list is %s", self.goal_list)
		self.raw_goals = data['goals']

		self.goals = {}
		for x,y in self.raw_goals.iteritems():
			self.goals[x] = GoalReader.create_pose_stamped_from_yaml(y)

	@staticmethod
	def create_pose_stamped_from_yaml(yaml_goal):
		"""Creates a PoseStamped from a goal loaded up from the yaml file"""
		pose = PoseStamped()
		pose.header.frame_id = yaml_goal['frame_id']
		pose.header.stamp = rospy.Time.now()

		pose.pose.position.x = yaml_goal['x']
		pose.pose.position.y = yaml_goal['y']
		quaternion = tf.transformations.quaternion_about_axis(yaml_goal['theta'], (0,0,1))
		pose.pose.orientation = Quaternion(*quaternion)

		return pose

def main(filename):
	goal_reader = GoalReader(filename)

	path = Path()
	for g in goal_reader.goal_list:
		path.poses.append( goal_reader.goals[g] )
	
	path_publisher = rospy.Publisher('/waypoints', Path)
	rospy.sleep(0.5) # dem publisher etwas zeit geben..
	path_publisher.publish(path)

if __name__ == '__main__':
	rospy.init_node('goal_reader')
	main(rospy.get_param('~filename'))
