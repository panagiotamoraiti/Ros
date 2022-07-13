#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from laser_line_extraction.msg import LineSegmentList

flag=True

def callback(msg):
    global flag
    rate = rospy.Rate(15)
    lines = msg.line_segments #list of classes (lines)
    front_lines = []
    front_lines.append((10, 10))
    
    for i, line in enumerate(lines):

        # [(-0.4),(0.4)] = 0.8rasds = 45.836 degrees
        if (line.angle < 0.4 and line.angle > -0.4): 
            front_lines.append((i, line.radius, line.start, line.end)) 

    front_lines.sort(key=lambda x:x[1]) # Sort based on minimum distance
    radius = front_lines[0][1] # Take minimum distance
    start = front_lines[0][2]
    end = front_lines[0][3]
    #rospy.loginfo(start)
    #rospy.loginfo(end)
    y_goal=(start[1]+end[1])/2
    if(flag==True):
        if (not (-0.005<y_goal and y_goal<0.005)):
            rospy.loginfo(y_goal)
            if (y_goal>0):
                move.linear.y = 0.05
            if (y_goal<0):
                move.linear.y =-0.05
        else:
            move.linear.y = 0.0
            rospy.loginfo("End")
            flag=False
        pub.publish(move)


rospy.init_node('position2')
sub = rospy.Subscriber('/line_segments', LineSegmentList, callback)
pub = rospy.Publisher('/dynamixel_workbench/cmd_vel', Twist, queue_size=10)
 
move = Twist()
rospy.spin()

