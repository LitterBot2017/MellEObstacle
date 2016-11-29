#!/usr/bin/env python
# license removed for brevity
import rospy
import hokuyo_node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from melle_obstacle_avoidance.msg import ObAvData

# Commands:
# 0 = go
# 1 = right
# 2 = left

ob_av_data_pub = None

def callback(stuff):
	leftWeight = 0.0
	rightWeight = 0.0
	command = 0

	if min(stuff.ranges[160:360]) < 1 or min(stuff.ranges[40:480]) < 0.5 :

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
    global ob_av_data_pub 
    ob_av_data_pub = rospy.Publisher("/ob_av_data", ObAvData)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    laser_reader()