#!/usr/bin/env python3

import cv2
import numpy as np
import time
import copy
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class LocalizationGrid:
	def __init__(self, dist_to_walls):
		self.occupancy_grid = None
		self.footprint = None
		self.robot_pos = None
		self.first = False
		self.dist_to_walls = dist_to_walls
		
		self.grid_sub = rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.occupancy_grid_callback)
		self.footprint_sub = rospy.Subscriber("/move_base/local_costmap/footprint", PolygonStamped, self.footprint_callback)
		self.robot_pos = rospy.Subscriber("/slam_out_pose", PoseStamped, self.robot_pos_callback)
		self.pub = rospy.Publisher('/center_location', Point, queue_size=20)
		
		if dist_to_walls:
			self.pub_north = rospy.Publisher('/walls/north', Float32, queue_size=10)
			self.pub_south = rospy.Publisher('/walls/south', Float32, queue_size=10)
			self.pub_east = rospy.Publisher('/walls/east', Float32, queue_size=10)
			self.pub_west = rospy.Publisher('/walls/west', Float32, queue_size=10)

	def occupancy_grid_callback(self, msg):
		self.occupancy_grid = msg

	def footprint_callback(self, msg):
		self.footprint = msg

	def robot_pos_callback(self, msg):
		if self.first == False:
			self.origin = msg
			self.first = True
		self.robot_pos = msg

	def show_image(self):
		cv2.imshow("Occupancy Grid with footprint", self.generate_image())
		while True:
				key = cv2.waitKey(0)
				if key == ord('n'):
						# code to generate and show the new image
						cv2.imshow("Occupancy Grid with footprint", self.generate_image())
				elif key == ord('q'):
						break
		cv2.destroyAllWindows()

	def generate_image(self, debug=False):
		while self.occupancy_grid is None or self.footprint is None:
			rospy.sleep(0.1)
		
		data = np.array(self.occupancy_grid.data, np.float32)
		image = data.reshape(self.occupancy_grid.info.height, self.occupancy_grid.info.width)
		image = np.where(image == -1, 255, image)
		image = np.where(image <= 80, 255, image)
		image = np.where(image <= 100, 0, image)

		def dilateBlack(image):
			kernel = np.ones((5, 5), np.uint8)
			image[image == 0] = 1
			image[image == 255] = 0
			image = cv2.dilate(image, kernel, iterations=2)
			image = cv2.erode(image, kernel, iterations=2)
			image[image == 0] = 255
			image[image == 1] = 0
			return image

		image = dilateBlack(image)
		image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

		# Increase size to 500px at least.
		width = self.occupancy_grid.info.width
		height = self.occupancy_grid.info.height
		new_width = 500
		new_height = int(500 * self.occupancy_grid.info.height / self.occupancy_grid.info.width)
		image = cv2.resize(image, (new_width, new_height))
		image = cv2.flip(image, 0) # Flip image to match rviz/gazebo output

		if debug:
			self.original = image.copy()
		if dist_to_walls:
			self.dist_img = image.copy()
			self.dist_img = cv2.cvtColor(self.dist_img, cv2.COLOR_BGR2GRAY)
		
		# Transform input into grid
		def getCells(image):
			image = np.uint8(image)

			gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
			blur = cv2.GaussianBlur(gray, (3,3), 0)
			thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]

			# Obtain horizontal lines mask
			horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (50,1))
			horizontal_mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, horizontal_kernel, iterations=1)
			horizontal_mask = cv2.dilate(horizontal_mask, horizontal_kernel, iterations=9)

			# Obtain vertical lines mask
			vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,50))
			vertical_mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, vertical_kernel, iterations=1)
			vertical_mask= cv2.dilate(vertical_mask, vertical_kernel, iterations=9)

			# Bitwise-and masks together
			result = 255 - cv2.bitwise_or(vertical_mask, horizontal_mask)

			# Fill individual grid holes
			cnts = cv2.findContours(result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cnts = cnts[0] if len(cnts) == 2 else cnts[1]
			for c in cnts:
				x,y,w,h = cv2.boundingRect(c)
				cv2.rectangle(result, (x, y), (x + w, y + h), 255, -1)

			return result

		image = getCells(image)

		self.image = image.copy()
		
		# Draw robot in map

		# Use map metadata to locate the rectangle
		origin_x = self.occupancy_grid.info.origin.position.x
		origin_y = self.occupancy_grid.info.origin.position.y
		resolution = self.occupancy_grid.info.resolution
		footprint_points = [(int((point.x - origin_x) / resolution), int((point.y - origin_y) / resolution)) for point in self.footprint.polygon.points]
		footprint_points = [(int((point[0] * new_width) / width), int((point[1] * new_height) / height)) for point in footprint_points]

		# Flip individual points (because image is flipped to match rviz / simulator output)
		footprint_points = [((500-point[0]-1),(point[1])) for point in footprint_points]

		# Get robot's center
		x_loc = int((footprint_points[0][0] + footprint_points[1][0] + footprint_points[2][0] + footprint_points[3][0]) / 4)
		y_loc = int((footprint_points[0][1] + footprint_points[1][1] + footprint_points[2][1] + footprint_points[3][1]) / 4)

		self.centerX = x_loc
		self.centerY = y_loc
		self.footprint_data = footprint_points

		# Get distance from robot to walls in the north, south, east, and west.
		if self.dist_to_walls:
			self.dist_center_point(x_loc, y_loc, dist_to_walls=True)

		# Draw Rectangle
		if debug:
			for i in range(len(footprint_points)):
				if i+1 < len(footprint_points):
					cv2.line(image, footprint_points[i], footprint_points[i+1], color = (0,255,0), thickness = 2)
					cv2.line(self.original, footprint_points[i], footprint_points[i+1], color = (0,255,0), thickness = 2)
				else:
					cv2.line(image, footprint_points[i], footprint_points[0], color = (0,255,0), thickness = 2)
					cv2.line(self.original, footprint_points[i], footprint_points[0], color = (0,255,0), thickness = 2)

		return image
	
	
	def line_width(self, start, y_dir, up, image, width, height):
		"""Return half of the line's width.
		"""
		line_width = 0
		if y_dir:
			if up:
				for i in range(start, 0, -1):
					if line_width > 10:
						break
					if image[i][self.selected_p[0]] == 0:
						line_width += 1
					else:
						break
			# if y_dir and down
			else:
				for i in range(start, height, 1):
					if line_width > 10:
						break
					if image[i][self.selected_p[0]] == 0:
						line_width += 1
					else:
						break
		else:
			if up:
				for i in range(start, width, 1):
					if line_width > 10:
						break
					if image[self.selected_p[1]][i] == 0:
						line_width += 1
					else:
						break
			else:
				for i in range(start, 0, -1):
					if line_width > 10:
						break
					if image[self.selected_p[1]][i] == 0:
						line_width += 1
					else:
						break
		return int(line_width / 2)

	def dist_center_point(self, c_x, c_y, dist_to_walls=False):
		"""Find distance from point (c_x, c_y) to the center of the nearest cell.
		
		Args:
			c_x: the x position of the point to check
			c_y: the y position of the point to check
			dist_to_walls: publish distance to walls in north, south, east, and west

		Returns:
			Distance to center in cm.
		"""

		image = []
		if not dist_to_walls:
			image = self.image.copy() # Contains map with grid.
		else:
			image = self.dist_img

		width = 500
		height = int(500 * (self.occupancy_grid.info.height/self.occupancy_grid.info.width))
		mapHeight = 225 # in cm
		cell_distance = mapHeight / height

		north, south, east, west = 1000, 1000, 1000, 1000
		
		# If the point to check interects with the grid, skip point.
		if image[c_y][c_x] == 0:
			return [-10, -10]

		self.selected_p = (c_x, c_y) # Point used to check line width

		# get distance to nearest line in north/south, east/west
		# check north
		for i in range (c_y, 0, -1):
			# X is constant when checking north and south of robot
			if image[i][c_x] == 0:
				north = abs(c_y - i)
				north += self.line_width(i, True, True, image, width, height)
				break

		# check south
		for i in range (c_y , height, 1):
			if image[i][c_x] == 0:
				south = abs(i - c_y)
				south += self.line_width(i, True, False, image, width, height)
				break

		# check east
		for i in range (c_x , width, 1):
			# Y is constant when checking east and west of robot
			if image[c_y][i] == 0:
				east = abs(i - c_x)
				east += self.line_width(i, False, True, image, width, height)
				break
		
		for i in range (c_x, 0, -1):
			if image[c_y][i] == 0:
				west = abs(c_x - i)
				west += self.line_width(i, False, False, image, width, height)
				break
		
		if dist_to_walls:
			north *= cell_distance
			south *= cell_distance
			east *= cell_distance
			west *= cell_distance

			# Generate and publish messages.
			d_n = Float32()
			d_s = Float32()
			d_e = Float32()
			d_w = Float32()

			d_n.data = north
			d_s.data = south
			d_e.data = east
			d_w.data = west

			self.pub_north.publish(d_n)
			self.pub_south.publish(d_s)
			self.pub_east.publish(d_e)
			self.pub_west.publish(d_w)
			return

		def distance_to_center(units_to_line):
			"""Transforms cell count into centimeters. Finds distance
			to nearest center.
			"""
			dist_to_line = cell_distance * units_to_line
			dist_to_line = dist_to_line % 30
			dist_to_center = 15 - dist_to_line
			return dist_to_center
		
		d_x = 15
		d_y = 15

		# Use nearest walls as reference to estimate center error..

		if west < east:
			d_x = distance_to_center(west)
		else:
			d_x = distance_to_center(east)
			d_x *= -1

		if north < south:
			d_y = distance_to_center(north)
			d_y *= -1
		else:
			d_y = distance_to_center(south)
		
		# Send specific values out of range if no point was used.
		if (south == 1000 and north == 1000):
			d_y = -31
		if (east == 1000 and west == 1000):
			d_x = -31		
		
		return [d_x, d_y]

	def nearest_center(self):
		"""Iterates through the robot's points such that there is at least one valid point
		if the grid image is generated correctly, in order to find the nearest grid's center.
		"""
		d_x = -10
		d_y = -10
		points = self.footprint_data.copy()

		# Use the following points to locate the square's center. Go to next point if the past point is "over" a wall.
		# Points, in order: robot's center, middle point of robot's front, front right corner, front left corner.
		test_points = [(self.centerX, self.centerY), (int((points[1][0] + points[2][0]) / 2), int((points[1][1] + points[2][1]) / 2)),
					points[1], points[2]]

		height = int(500 * (self.occupancy_grid.info.height/self.occupancy_grid.info.width))
		mapHeight = 225 # in cm
		cell_distance = mapHeight / height

		def difference(l_x, l_y):
			"""Transforms [distance to cell's center from one point] -> [distance to cell's center from robot's center]
			Args:
				l_x: X coordinate of point used to find cell's center
				l_y: Y coordinate of point used to find cell's center
			"""
			r_x = l_x - self.centerX
			r_y = l_y - self.centerY
			r_x *= cell_distance
			r_y *= cell_distance * -1 # Negate value because greater Y values correspond to points closer to the end of the map
			return [r_x, r_y]

		for i in range(len(test_points)):
			d_x, d_y = self.dist_center_point(test_points[i][0], test_points[i][1])

			# If the distances are valid, transform distances to make output relative to robot's center
			if d_x != -10 and d_y != -10:
				diff_x, diff_y = difference(test_points[i][0], test_points[i][1])
				d_x += diff_x
				d_y += diff_y
				break
		
		return [d_x, d_y]
	
	# TODO: debug method to calculate centers using robot's position.
	def robot_pos_data(self):
		while self.robot_pos is None:
			rospy.sleep(0.1)

		pos = self.robot_pos.pose.position
		origin = self.origin.pose.position

		# set origin to bottom left corner
		pos_x = pos.x - origin.x
		pos_y = pos.y - origin.y

		origin_casilla = (1, 1)

		# Set position to cm
		pos_x = round(pos_x * 100,1) 
		pos_y = round(pos_y * 100,1) 
		pos_z = round(pos.z * 100,1) 

		pos_x += 30 * origin_casilla[0]
		pos_y += 30 * origin_casilla[1]

		c_x = (pos_x % 30)
		c_y = (pos_y % 30)

		if c_x > 15:
			c_x = c_x - 15
		else:
			c_x = 15 - c_x
		if c_y > 15:
			c_y = c_y - 15
		else:
			c_y = 15 - c_y

		print("Real dist to center -x: " + str(c_x) + ", -y: " + str(c_y))
		casillas_moved_x = int(pos_x / 30)
		casillas_moved_y = int(pos_y / 30)
		print("Robot position - x: " + str(pos_x) + ", y: " + str(pos_y) + ", z: " + str(pos_z))
		print("Tile position - x: " + str(casillas_moved_x) + ", y: " + str(casillas_moved_y))

	def loopData(self):
		counter = 0
		average = 0

		while not rospy.is_shutdown():
			start_time = time.time()
			self.generate_image()
			d_x, d_y = self.nearest_center()
			
			# Generate and publish message.
			message = Point()
			message.x = d_x
			message.y = d_y
			self.pub.publish(message)

			end_time = time.time()
			exec_time = (end_time - start_time) * 1000
			print("--- %s ms ---" % (exec_time))
			average += exec_time
			counter += 1
			print("--- %s Average ms ---" % ((average) / counter))

	def debugData(self, sleep_t):
		counter = 0
		average = 0

		height = 500
		mapHeight = 225 # in cm
		cd = mapHeight / height

		while not rospy.is_shutdown():
			start_time = time.time()
			out = self.generate_image(True)
			d_x, d_y = self.nearest_center()
			
			# Generate and publish message.
			message = Point()
			message.x = d_x
			message.y = d_y
			self.pub.publish(message)

			ad = 0
			# transform Y cell count because lower values are closer to top.
			if d_y > 0:
				ad = abs(d_y/cd) * -1
			else:
				ad = abs(d_y/cd)
			
			# Log detected center
			cv2.circle(out, (int(self.centerX + d_x/cd), int(self.centerY + ad)), 3, color = (0,0,255), thickness = 2)
			cv2.imshow("Real Time", out)

			cv2.circle(self.original, (int(self.centerX + d_x/cd), int(self.centerY + ad)), 3, color = (0,0,255), thickness = 2)
			cv2.imshow('Original', self.original)
			
			print("Distance to center: x: " + str(d_x) + ", y: " + str(d_y))
			key = cv2.waitKey(1)
			
			end_time = time.time()
			exec_time = (end_time - start_time) * 1000
			print("--- %s ms ---" % (exec_time))
			average += exec_time
			counter += 1
			print("--- %s Average ms ---" % ((average) / counter))
			rospy.sleep(sleep_t)


if __name__ == '__main__':
	try:
		# Initialize the node
		rospy.init_node('localization_grid')
		rospy.loginfo('Localization grid initialized.')

		# Specify node behaviour using parameters
		dist_to_walls = False
		debug = False

		name = rospy.get_name() + "/"

		if rospy.has_param(name + "send_dist_to_walls"):
			dist_to_walls = rospy.get_param(name + "send_dist_to_walls")
		
		if rospy.has_param(name + "debug"):
			debug = rospy.get_param(name + "debug")

		costmap_with_rectangle = LocalizationGrid(dist_to_walls)

		if debug:
			costmap_with_rectangle.debugData(0.2)
		else:
			costmap_with_rectangle.loopData()
		
	except rospy.ROSInterruptException:
		pass
