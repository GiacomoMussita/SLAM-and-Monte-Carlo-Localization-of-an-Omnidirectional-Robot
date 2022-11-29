#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData
from std_srvs.srv import Trigger, TriggerResponse


# list of the poses (trajectory)
traj = []

# package path and map.png path
rp = rospkg.RosPack()
package_path = rp.get_path('project2')
map_image_path = package_path + '/maps/map.pgm'

# default map resolution and map origin
resolution = 0.05
origin = [0, 0]


def callbackTraj(data):

	global traj
	traj.append((data.pose.pose.position.x, data.pose.pose.position.y))
	#print(traj)


def callbackMetaData(data):

	global resolution
	global origin
	resolution = data.resolution
	origin[0] = data.origin.position.x
	origin[1] = data.origin.position.y


def callbackSaveMap(name):

	newmap = cv2.imread(map_image_path, 1)
	# pixel dimension in y direction of the image: we need to compensate this since the position of the points
	# is taken with respect to the lower-left pixel, which is pixel (0, height)
	height = newmap.shape[0]
	#print(height)
	counter = 0

	for x, y in traj:
		# we keep drawing lines until we reach the end of the trajectory
		if counter < len(traj)-1:
			xstart = int((x+abs(origin[0]))/resolution)
			ystart = height - int((y+abs(origin[1]))/resolution)
			xend = int((traj[counter+1][0]+abs(origin[0]))/resolution)
			yend = height - int((traj[counter+1][1]+abs(origin[1]))/resolution)
			if counter == 1:
				# start point: green dot
				newmap = cv2.circle(newmap, (xstart,ystart), radius=2, color=(255, 0, 0), thickness=-1)
			newmap = cv2.line(newmap, (xstart, ystart), (xend, yend), (0, 0, 255), 1)
		counter += 1
	
	# end point: blue dot
	newmap = cv2.circle(newmap, (xend,yend), radius=2, color=(0, 255, 0), thickness=-1)
	#newmap = cv2.circle(newmap, (int(abs(origin[0])/resolution), height - int(abs(origin[1])/resolution)), 
	#radius=2, color=(255, 0, 0), thickness=-1)
	filename = package_path + '/maps/pathmap.png'
	cv2.imwrite(filename, newmap)
	response = TriggerResponse()
	response.success = True
	response.message = 'saved image'
	return response


def listener():

    rospy.init_node('path_map', anonymous=False)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callbackTraj)
    rospy.Subscriber('/map_metadata', MapMetaData, callbackMetaData)
    rospy.Service('save_map', Trigger, callbackSaveMap)

    rospy.spin()


if __name__ == '__main__':
    listener()