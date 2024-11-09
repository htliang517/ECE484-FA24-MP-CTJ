import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time
import csv

class vehicleController():

	def __init__(self):
		# Publisher to publish the control input to the vehicle model
		self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
		self.prev_vel = 0
		self.L = 1.75 # Wheelbase, can be get from gem_control.py
		self.log_acceleration = True  # Set to True to log acceleration
		# self.acceleration_data = []  # List to store acceleration values
		# self.time_data = []  # List to store time values
		# self.current_time = 0  # Initialize time 
		self.acceleration_list = []

	def getModelState(self):
		# Get the current state of the vehicle
		# Input: None
		# Output: ModelState, the state of the vehicle, contain the
		#   position, orientation, linear velocity, angular velocity
		#   of the vehicle
		rospy.wait_for_service('/gazebo/get_model_state')
		try:
			serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
			resp = serviceResponse(model_name='gem')
		except rospy.ServiceException as exc:
			rospy.loginfo("Service did not process request: "+str(exc))
			resp = GetModelStateResponse()
			resp.success = False
		return resp


	# Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
	#       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
	# Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
	def extract_vehicle_info(self, currentPose):

		####################### TODO: Your TASK 1 code starts Here #######################
		pos_x, pos_y, vel, yaw = 0, 0, 0, 0
		# Use ModelState.msg
		# extract x and y positions from currentPose
		pos_x = currentPose.pose.position.x
		pos_y = currentPose.pose.position.y
		
		# extract velocity 
		vel_x = currentPose.twist.linear.x
		vel_y = currentPose.twist.linear.y
		vel = np.sqrt(vel_x**2 + vel_y**2)  
		
		# extractEuler angles (roll, pitch, yaw)
		orientation = currentPose.pose.orientation
		euler = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
		yaw = euler[2]  # only need the yaw (heading) angle
		####################### TODO: Your Task 1 code ends Here #######################

		return pos_x, pos_y, vel, yaw # note that yaw is in radian

	# Task 2: Longtitudal Controller
	# Based on all unreached waypoints, and your current vehicle state, decide your velocity
	def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

		####################### TODO: Your TASK 2 code starts Here #######################
		target_velocity = 10
		if len(future_unreached_waypoints) < 2:  # only one point left
			return curr_vel  # no more waypoints, just keep same vel
		
		wp1_x, wp1_y = future_unreached_waypoints[0]  # Next waypoint x
		wp2_x, wp2_y = future_unreached_waypoints[1]  # waypoint y after x

		# calculate curvature
		# vectors of distances
		vec1_x, vec1_y = wp1_x - curr_x, wp1_y - curr_y
		vec2_x, vec2_y = wp2_x - wp1_x, wp2_y - wp1_y

		dot_product = vec1_x * vec2_x + vec1_y * vec2_y
		
		# magnitudes of the vectors
		mag1 = np.sqrt(vec1_x**2 + vec1_y**2)
		mag2 = np.sqrt(vec2_x**2 + vec2_y**2)
		
		# cosine of the angle between the two vectors
		cos_theta = dot_product / (mag1 * mag2)

		if cos_theta > 0.98:  # Near straight, 1 for "straightness"
			target_velocity = 12  
		else:
			target_velocity = 8 

		####################### TODO: Your TASK 2 code ends Here #######################
		return target_velocity


	# Task 3: Lateral Controller (Pure Pursuit)
	def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

		####################### TODO: Your TASK 3 code starts Here #######################
		
		lookahead_distance = 6
		min_ld = 2.0  # min look-ahead distance for sharp turns

		# using the last waypoint as look-ahead if no interpolation is needed
		lookahead_x, lookahead_y = target_point

		# Interpolate 
		for i in range(len(future_unreached_waypoints) - 1):
			wp1 = future_unreached_waypoints[i]
			wp2 = future_unreached_waypoints[i + 1]

			# distances from the current position to each waypoint
			dist_wp1 = np.sqrt((wp1[0] - curr_x) ** 2 + (wp1[1] - curr_y) ** 2)
			dist_wp2 = np.sqrt((wp2[0] - curr_x) ** 2 + (wp2[1] - curr_y) ** 2)

			# If look-ahead distance between wp1 and wp2, interpolate
			if dist_wp1 < lookahead_distance < dist_wp2:
				# interpolation ratio
				ratio = (lookahead_distance - dist_wp1) / (dist_wp2 - dist_wp1)
				# Interpolate to find the look-ahead point
				lookahead_x = wp1[0] + ratio * (wp2[0] - wp1[0])
				lookahead_y = wp1[1] + ratio * (wp2[1] - wp1[1])
				break

		# ld  (not in range, use closest target point)
		dx = lookahead_x - curr_x
		dy = lookahead_y - curr_y
		ld = np.sqrt(dx**2 + dy**2)  
		
		ld = max(ld, min_ld)  # Make sure ld doesn't drop below min_ld

		# adjust look-ahead distance dynamically
		straight_threshold = 0.08  
		angle_to_lookahead = np.arctan2(dy, dx)  
		
		# if the angle between current heading and the look-ahead point is small 
		if abs(angle_to_lookahead - curr_yaw) < straight_threshold:
			# If path is straight, increase the look-ahead distance for smoother control
			ld = max(ld, 4.0)
		
		alpha = angle_to_lookahead - curr_yaw  
		alpha = np.arctan2(np.sin(alpha), np.cos(alpha))   # Normalize alpha to the range [-pi, pi]
		L = self.L 
		target_steering = np.arctan(2 * L * np.sin(alpha) / ld)


		####################### TODO: Your TASK 3 code starts Here #######################
		return target_steering


	def execute(self, currentPose, target_point, future_unreached_waypoints):
		# Compute the control input to the vehicle according to the
		# current and reference pose of the vehicle
		# Input:
		#   currentPose: ModelState, the current state of the vehicle
		#   target_point: [target_x, target_y]
		#   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
		# Output: None

		curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

		# Acceleration Profile
		if self.log_acceleration:
			acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
			self.prev_vel = curr_vel
			t = rospy.Time.now()
			self.acceleration_list.append([str(t), str(acceleration)])
			if len(future_unreached_waypoints) == 1:
				with open("t-a.csv", "w") as f:
					writer = csv.writer(f)
					writer.writerows(self.acceleration_list)
					print("Save t-a.csv!")

		target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
		target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)

		# --------- [Write Trace to File] ---------
		with open("Trace_yudai.csv", mode='a', newline="") as file:
			writer = csv.writer(file)
			writer.writerow([curr_x, curr_y])
		# ------------------------------------------

		
		#Pack computed velocity and steering angle into Ackermann command
		newAckermannCmd = AckermannDrive()
		newAckermannCmd.speed = target_velocity
		newAckermannCmd.steering_angle = target_steering

		# Publish the computed control input to vehicle model
		self.controlPub.publish(newAckermannCmd)

	def stop(self):
		newAckermannCmd = AckermannDrive()
		newAckermannCmd.speed = 0
		self.controlPub.publish(newAckermannCmd)
