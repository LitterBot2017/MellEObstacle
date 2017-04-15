#!/usr/bin/env python
# license removed for brevity
import rospy
import hokuyo_node
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from obstacle_avoidance.msg import ObstacleHeading



desired_heading_publisher = None


def callback(stuff):
	leftWeight = 0.0
	rightWeight = 0.0
	command = 0
	maxRangeMeters = 2.0
	receptiveField = 180
	angleMin = -1.57
	angleMax = 1.57

	iter = (i for i in range(len(stuff.ranges)))

	arrLength = int((angleMax - angleMin) / stuff.angle_increment)
	strengthVec =  np.zeros((1, arrLength)) #np.zeros((1,len(stuff.ranges)))
	directionVec = np.zeros((1, arrLength)) #strengthVec
	weightVec =  np.zeros((1, arrLength)) # strengthVec

	#strengthVec = strengthVec + stuff.ranges
	#strengthVec = 1/(strengthVec)


	j = 0

	for i in iter:
		currAngle = stuff.angle_min + stuff.angle_increment*i
		if currAngle > angleMin and currAngle < angleMax:
			if not math.isnan(currAngle) and not math.isnan(3/(1+stuff.ranges[i])):
				strengthVec[0][j] = 1/(stuff.ranges[i])
				directionVec[0][j] = stuff.angle_min + stuff.angle_increment*i
				weightVec[0][j] = 1/(1+stuff.ranges[i]*stuff.ranges[i])#3/(1+stuff.ranges[i])
				if stuff.ranges[i] > maxRangeMeters:
					weightVec[0][j] = 0
				j = j + 1


	xDirVec = np.cos(directionVec)
	yDirVec = np.sin(directionVec)

	totalX = np.sum(xDirVec*weightVec)
	totalY = np.sum(yDirVec*weightVec)
	totalDir = np.arctan(totalY/totalX)
	totalMag = math.sqrt(totalX*totalX + totalY*totalY)
	dirDeg = np.rad2deg(totalDir)
	negDir = totalDir - np.pi
	negDirDeg = np.rad2deg(negDir)
	if math.isnan(dirDeg):
		dirDeg = 0



	msg = ObstacleHeading()
	msg.direction = dirDeg
	msg.magnitude = totalMag
	desired_heading_publisher.publish(msg)
	rospy.loginfo(dirDeg)
	rospy.loginfo(totalMag)


def laser_reader():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('laser_reader', anonymous=True)

    rospy.Subscriber("/last",LaserScan,callback)
    global desired_heading_publisher 
    desired_heading_publisher = rospy.Publisher("/obstacle_heading", ObstacleHeading)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    laser_reader()