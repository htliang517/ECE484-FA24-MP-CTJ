import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = False

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
        """
        currentPose => TYPE = ModelState.msg

        (DataType   ParamName)
        >>>[ModelState Message (currentPose)]<<<
        string   model_name
        geometry_msgs/Pose   pose
        geometry_msgs/Twist   twist
        string   reference_frame

        >>>[geometry_msgs/Pose (pose)]<<<
        Point   position
            float64   x
            float64   y
            float64   z
        Quaternion   orientation
            float64   x
            float64   y
            float64   z
            float64   w

        >>>[geometry_msgs/Twist (twist)]<<<
        Vector3   linear
            float64   x
            float64   y
            float64   z
        Vector3   angular
            float64   x
            float64   y
            float64   z
        """
        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0

        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y

        vel = (currentPose.twist.linear.x**2 + currentPose.twist.linear.y**2)**0.5

        roll, pitch, yaw = quaternion_to_euler(
            currentPose.pose.orientation.x,
            currentPose.pose.orientation.y,
            currentPose.pose.orientation.z,
            currentPose.pose.orientation.w
        )
        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################
        # We suggest a baseline target speed of 12 m/s for straight sections and 8 m/s for turns.
        target_velocity = 10
        temp_targetX = future_unreached_waypoints[0][0]
        temp_targetY = future_unreached_waypoints[0][1]

        target_theta = math.atan2((temp_targetY-curr_y),(temp_targetX-curr_x))  # radius

        d_theta = target_theta - curr_yaw   # radius

        threshold = 10      # degree
        if d_theta <= threshold*180/np.pi:
            # print("Forward")
            target_velocity = 12
        else:
            # print("Turn")
            target_velocity = 8
        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity

    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints):

        ####################### TODO: Your TASK 3 code starts Here #######################
        target_steering = 0
        # ----- 1.Choose the lookahead point
        lookahead_point = future_unreached_waypoints[0] # closest point
        # lookahead_point = #TODO: Constant lookahead distance and interpolate between future waypoints
        # lookahead_point = #TODO: Dynamic lookahead distance and interpolate between future waypoints

        # ----- 2.Calculate lookachead distance
        ld = ((lookahead_point[0]-curr_x)**2 + (lookahead_point[1]-curr_y)**2)**0.5

        # ----- 3.Calculate Alpha
        alpha = math.atan2(lookahead_point[1] - curr_y, lookahead_point[0] - curr_x) - curr_yaw

        target_steering = math.atan2((2*self.L*np.sin(alpha)),(ld))
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

        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints)

        # Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
