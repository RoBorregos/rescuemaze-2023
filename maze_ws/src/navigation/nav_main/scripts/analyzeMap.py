#!/usr/bin/env python3

# Print map's properties

import cv2
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped

class InspectArray():
	def __init__(self):
		self.occupancy_grid = None
		self.footprint = None
		self.id = 0
		self.image = 0
		
		self.grid_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.occupancy_grid_callback)
		self.footprint_sub = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.footprint_callback)
	
	def occupancy_grid_callback(self, msg):
		self.occupancy_grid = msg

	def footprint_callback(self, msg):
		self.footprint = msg

	def show_image(self):
		self.read_array()
		cv2.imshow("Occupancy Grid with footprint", self.image)
		while True:
				key = cv2.waitKey(0)
				if key == ord('n'):
						# code to generate and show the new image
						cv2.imshow("Occupancy Grid with footprint", self.image)
				elif key == ord('q'):
						break
		cv2.destroyAllWindows()

	def read_array(self):
		while self.occupancy_grid is None or self.footprint is None:
			rospy.sleep(0.1)

		self.map_info()

		data = np.array(self.occupancy_grid.data, np.float32)
		image = data.reshape(self.occupancy_grid.info.height, self.occupancy_grid.info.width)

		for i in range (self.occupancy_grid.info.width):
			image[0][i] = 1
		
		s = "Dimensiones: " + str(image.ndim)
		rospy.loginfo(s)
		s = "Elementos: " + str(image.size)
		rospy.loginfo(s)
		s = "Shape: " + str(image.shape)
		rospy.loginfo(s)
		self.image = image

	def map_info(self):
		map = self.occupancy_grid
		s = "Map resolution: %d" % map.info.resolution
		print(s)
		s = "Map width: %d" % map.info.width
		print(s)
		s = "Map height: %d" % map.info.height
		print(s)
		print("Posicion:")
		print(map.info.origin.position)


if __name__ == '__main__':
	rospy.init_node('analyzeMap')
	costmap_with_rectangle = InspectArray()
	costmap_with_rectangle.show_image()