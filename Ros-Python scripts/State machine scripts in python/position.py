#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from laser_line_extraction.msg import LineSegmentList
from laser_line_extraction.msg import LineSegment
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxySubscriberCached
from tf.transformations import euler_from_quaternion


'''
Created: 05/05/2022
'''

class Position(EventState):
	'''
	State for center approach to a service area in the simulation environment. Finds the center of the line segment to 
	calculate the robot's distance and publish cmd_vel messages in order to achieve the goal.
	
	<= success
	<= failed
	'''
	
	def __init__(self):
		super(Position, self).__init__(outcomes=['success','failed'])
		
		self._sub_topic = '/line_segments'
		self._pub_topic = '/gaz_controller/cmd_vel'
		self.line_sub = ProxySubscriberCached({self._sub_topic: LineSegmentList})
		self.line_sub.set_callback(self._sub_topic, self.line_callback)
		self.cmd_pub = ProxyPublisher({self._pub_topic: Twist})
		self.flag = True
		self.rate = rospy.Rate(2)	

	def line_callback(self, data):
		self.lines = data.line_segments #list of classes (lines)
		self.front_lines = []
		self.front_lines.append((10, 10))
		#Logger.loginfo(self.lines)
	
	def on_start(self):
		Logger.loginfo("find position")
		
	def on_enter(self, userdata):
		Logger.loginfo("finding position STARTED!")
		
		self.cmd_vel = Twist()
		self.cmd_vel.linear.y = 0.0
		self.cmd_vel.angular.z = 0.0
		self._start_time = rospy.Time.now()
    
	def execute(self, userdata):
		if not self.cmd_pub:
                        return 'failed'

		for i, line in enumerate(self.lines):

        # [(-0.4),(0.4)] = 0.8rasds = 45.836 degrees
				if (line.angle < 0.4 and line.angle > -0.4): 
						self.front_lines.append((i, line.radius, line.start, line.end)) 

		self.front_lines.sort(key=lambda x:x[1]) # Sort based on minimum distance
		radius = self.front_lines[0][1] # Take minimum distance
		start = self.front_lines[0][2]
		end = self.front_lines[0][3]
		#Logger.loginfo(start)
		#Logger.loginfo(end)
		y_goal=(start[1]+end[1])/2
		if(self.flag==True):
				if (not (-0.005<y_goal and y_goal<0.005)):
						rospy.loginfo(y_goal)
						if (y_goal>0):
							self.cmd_vel.linear.y = 0.05
						if (y_goal<0):
							self.cmd_vel.linear.y =-0.05
				else:
						self.cmd_vel.linear.y = 0.0
						rospy.loginfo("End")
						self.flag=False
			
				self.cmd_pub.publish(self._pub_topic, self.cmd_vel)
		if (self.flag == False):
				return 'success' 

	def on_stop(self):
		Logger.loginfo("Drive FWD STOPPED!")

	def on_exit(self, userdata):
		self.cmd_vel.linear.x = 0.0
		self.cmd_vel.linear.y = 0.0
		self.cmd_pub.publish(self._pub_topic, self.cmd_vel)
		self.line_sub.unsubscribe_topic(self._sub_topic)
		Logger.loginfo("find pos ENDED!")
