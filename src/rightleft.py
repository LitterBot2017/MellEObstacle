#!/usr/bin/env python
# license removed for brevity
import rospy
import hokuyo_node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(stuff):
	leftWeight = 0.0
	rightWeight = 0.0
	if min(stuff.ranges[200:320]) < 2:
		i = 200
		for item in stuff.ranges[200:320]:
			if item != "inf":
				if i < 260:
					leftWeight = leftWeight + (1/(1+item))
				else:
					rightWeight = rightWeight + (1/(1+item))
			i += 1
		if leftWeight>rightWeight:
			rospy.loginfo("left")
		else:
			rospy.loginfo("right")
		leftWeight = 0.0
		rightWeight = 0.0
	else:
		rospy.loginfo("go")
'''
def callback(stuff):
	ranges = stuff.ranges
	angles = stuff.angles
	#normalize the ranges
	lineardistances = ranges * cos(angles)
	#if min of ranges between someangleleft and someangleright


 ''' 
def laser_reader():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laser_reader', anonymous=True)

    rospy.Subscriber("/scan",LaserScan,callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    laser_reader()