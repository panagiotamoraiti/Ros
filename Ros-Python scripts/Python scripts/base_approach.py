#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from laser_line_extraction.msg import LineSegmentList

flag=True

def callback(msg):
    rate = rospy.Rate(15)
    lines = msg.line_segments #list of classes (lines)
    front_lines = []
    front_lines.append((10, 10, 0))
   
    for i, line in enumerate(lines):
        # [(-0.4),(0.4)] = 0.8rasds = 45.836 degrees
        if (line.angle < 0.4 and line.angle > -0.4): 
            front_lines.append((i, line.radius, line.angle)) 

    front_lines.sort(key=lambda x:x[1]) # Sort based on minimum distance
    radius = front_lines[0][1] # Take minimum distance
    angle = front_lines[0][2]
    rospy.loginfo(angle)
    while (not (angle < 0.07  and angle > -0.07 and flag==False)):
        for i, line in enumerate(lines):
        # [(-0.4),(0.4)] = 0.8rasds = 45.836 degrees
            if (line.angle < 0.4 and line.angle > -0.4): 
                front_lines.append((i, line.radius, line.angle)) 

        front_lines.sort(key=lambda x:x[1]) # Sort based on minimum distance
        radius = front_lines[0][1] # Take minimum distance
        angle = front_lines[0][2]

        rospy.loginfo(angle)
        if (angle>0):
                 move.angular.z = 0.05
                 rospy.loginfo("Angle > 0")
        if (angle<0):
                move.angular.z = -0.05
                rospy.loginfo("Angle < 0")
	pub.publish(move)
   	flag=False
    move.angular.z = 0.0
    rospy.loginfo("end")
    pub.publish(move)
rospy.init_node('base_approach_4')
sub = rospy.Subscriber('/line_segments', LineSegmentList, callback)
pub = rospy.Publisher('/dynamixel_workbench/cmd_vel', Twist, queue_size=10)

move = Twist() 
rospy.spin()
