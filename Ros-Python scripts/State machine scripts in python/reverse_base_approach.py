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


class Reverse_Base_approach(EventState):
    '''
    State for center approach to a service area in the simulation environment. Finds the center of the line segment to
    calculate the robot's distance and publish cmd_vel messages in order to achieve the goal.

    <= success
    <= failed
    '''

    def __init__(self):
        super(Base_approach, self).__init__(outcomes=['success', 'failed'])

        self._sub_topic = '/line_segments'
        self._pub_topic = '/gaz_controller/cmd_vel'
        self.line_sub = ProxySubscriberCached({self._sub_topic: LineSegmentList})
        self.line_sub.set_callback(self._sub_topic, self.line_callback)
        self.cmd_pub = ProxyPublisher({self._pub_topic: Twist})
        self.flag = True
        self.rate = rospy.Rate(2)

    def line_callback(self, data):
        self.lines = data.line_segments  # list of classes (lines)
        self.front_lines = []
        self.front_lines.append((10, 10))

    # Logger.loginfo(self.lines)

    def on_start(self):
        Logger.loginfo("reverse_approach")

    def on_enter(self, userdata):
        Logger.loginfo("reverse_approach STARTED!")

        self.cmd_vel = Twist()
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.angular.x = 0.0
        self._start_time = rospy.Time.now()

    def execute(self, userdata):
        if not self.cmd_pub:
            return 'failed'

        for i, line in enumerate(self.lines):

            # [(-0.4),(0.4)] = 0.8rasds = 45.836 degrees
            if (line.angle < 0.4 and line.angle > -0.4):
                self.front_lines.append((i, line.radius, line.start, line.end))

        self.front_lines.sort(key=lambda x: x[1])  # Sort based on minimum distance
        radius = self.front_lines[0][1]  # Take minimum distance
        start = self.front_lines[0][2]
        end = self.front_lines[0][3]
        # Logger.loginfo(start)
        # Logger.loginfo(end)
        y_goal = (start[1] + end[1]) / 2
        if (self.flag == True):
            if radius < 0.4:
                self.cmd_vel.linear.x = -0.05
                rospy.loginfo(radius)
            else:
                self.cmd_vel.linear.x = 0.0
                rospy.loginfo("End")
                self.flag = False
        self.cmd_vel.publish(self._pub_topic, self.cmd_vel)

        if (self.flag == False):
            return 'success'

    def on_stop(self):
        Logger.loginfo("Reverse base approach STOPPED!")

    def on_exit(self, userdata):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_pub.publish(self._pub_topic, self.cmd_vel)
        self.line_sub.unsubscribe_topic(self._sub_topic)
        Logger.loginfo("reverse_base_approach ENDED!")
