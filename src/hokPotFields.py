#!/usr/bin/env python
# license removed for brevity
import rospy
import hokuyo_node
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from melle_obstacle_avoidance.msg import ObAvData



ob_av_data_pub = None

def callback(stuff):
	leftWeight = 0.0
	rightWeight = 0.0
	command = 0


	iter = (i for i in range(len(stuff.ranges)))

	strengthVec = np.zeros((1,len(stuff.ranges)))
	directionVec = strengthVec
	weightVec = strengthVec

	strengthVec = strengthVec + stuff.ranges
	strengthVec = 1/strengthVec

	for i in iter:
		if not math.isnan(stuff.angle_min + stuff.angle_increment*i) and not math.isnan(3/(1+stuff.ranges[i])):
			directionVec[0][i] = stuff.angle_min + stuff.angle_increment*i
			weightVec[0][i] = 3/(1+stuff.ranges[i])
			#rospy.loginfo(directionVec[0][i])


	



	xDirVec = np.cos(directionVec)
	yDirVec = np.sin(directionVec)

	totalX = np.sum(xDirVec*weightVec)
	totalY = np.sum(yDirVec*weightVec)
	totalDir = np.arctan(totalY/totalX)
	totalMag = math.sqrt(totalX*totalX + totalY*totalY)
	rospy.loginfo(totalDir)
	rospy.loginfo(totalMag)

	#rospy.loginfo(directionVec)

'''

	#stop if something is in either testicle

	if min(stuff.ranges[40:180]) < 0.5 and min(stuff.ranges[340:480]) < 0.5:
		command = 3



	elif min(stuff.ranges[160:360]) < 1 or min(stuff.ranges[40:480]) < 0.5 :

		# Penis
		i = 160
		for distance in stuff.ranges[160:360]:
			if distance < 2 and distance != float('inf') and distance != float('nan'):
				if i < 260:
					leftWeight = leftWeight + (1/distance)
				else:
					rightWeight = rightWeight + (1/distance)
			i += 1

		# Left Testicle
		i = 40
		for distance in stuff.ranges[40:160]:
			if distance < 0.5 and distance != float('inf') and distance != float('nan'):
				leftWeight = leftWeight + (1/distance)
			i += 1

		# Right Testicle
		i = 360
		for distance in stuff.ranges[360:480]:
			if distance < 0.5 and distance != float('inf') and distance != float('nan'):
				rightWeight = rightWeight + (1/distance)
			i += 1

		if leftWeight > rightWeight:
			rospy.loginfo("left")
			command = 2
		else:
			rospy.loginfo("right")
			command = 1
		leftWeight = 0.0
		rightWeight = 0.0
	else:
		rospy.loginfo("go")

	msg = ObAvData()
	msg.command = command

	ob_av_data_pub.publish(msg)
'''
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
   # global ob_av_data_pub 
   # ob_av_data_pub = rospy.Publisher("/ob_av_data", ObAvData)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    laser_reader()