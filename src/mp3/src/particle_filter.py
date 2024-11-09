import numpy as np
from maze import Maze, Particle, Robot
import bisect
import rospy
from gazebo_msgs.msg import  ModelState
from gazebo_msgs.srv import GetModelState
import shutil
from std_msgs.msg import Float32MultiArray
from scipy.integrate import ode

import random
import matplotlib.pyplot as plt
import time
import datetime
import math

def vehicle_dynamics(t, vars, vr, delta):
    curr_x = vars[0]
    curr_y = vars[1] 
    curr_theta = vars[2]
    
    dx = vr * np.cos(curr_theta) * t
    dy = vr * np.sin(curr_theta) * t
    dtheta = delta * t
    return [dx,dy,dtheta]

class particleFilter:
    def __init__(self, bob, world, num_particles, sensor_limit, x_start, y_start):
        self.num_particles = num_particles  # The number of particles for the particle filter
        self.sensor_limit = sensor_limit    # The sensor limit of the sensor
        particles = list()

        ##### TODO:  #####
        # Modify the initial particle distribution to be within the top-right quadrant of the world, and compare the performance with the whole map distribution.
        for i in range(num_particles):

            # (Default) The whole map
            x = np.random.uniform(0, world.width)
            y = np.random.uniform(0, world.height)

            ## top-left quadrant
            # x = np.random.uniform(int(world.width / 2), world.width)
            # y = np.random.uniform(int(world.height / 2), world.height)

            ## top-right quadrant
            # x = np.random.uniform(0, int(world.width / 2))
            # y = np.random.uniform(int(world.height / 2), world.height)

            particles.append(Particle(x = x, y = y, maze = world, sensor_limit = sensor_limit))

        ###############

        self.particles = particles          # Randomly assign particles at the begining
        self.bob = bob                      # The estimated robot state
        self.world = world                  # The map of the maze
        self.x_start = x_start              # The starting position of the map in the gazebo simulator
        self.y_start = y_start              # The starting position of the map in the gazebo simulator
        self.modelStatePub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=1)
        self.controlSub = rospy.Subscriber("/gem/control", Float32MultiArray, self.__controlHandler, queue_size = 1)
        self.control = []                   # A list of control signal from the vehicle

        self.control_new_len = 0
        self.control_old_len = 0
        
        return

    def __controlHandler(self,data):
        """
        Description:
            Subscriber callback for /gem/control. Store control input from gem controller to be used in particleMotionModel.
        """
        tmp = list(data.data)
        self.control.append(tmp)

    def getModelState(self):
        """
        Description:
            Requests the current state of the polaris model when called
        Returns:
            modelState: contains the current model state of the polaris vehicle in gazebo
        """

        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name='polaris')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
        return modelState

    def weight_gaussian_kernel(self,x1, x2, std = 5000):
        if x1 is None: # If the robot recieved no sensor measurement, the weights are in uniform distribution.
            return 1./len(self.particles)
        else:
            tmp1 = np.array(x1)
            tmp2 = np.array(x2)
            return np.sum(np.exp(-((tmp2-tmp1) ** 2) / (2 * std)))


    def updateWeight(self, readings_robot):
        """
        Description:
            Update the weight of each particles according to the sensor reading from the robot 
        Input:
            readings_robot: List, contains the distance between robot and wall in [front, right, rear, left] direction.
        """

        ## TODO #####

        for particle in self.particles:
            readings_particle = particle.read_sensor()
            weight = self.weight_gaussian_kernel(readings_robot, readings_particle)
            particle.weight = weight

        total_weight = sum(p.weight for p in self.particles)
        for particle in self.particles:
            particle.weight /= total_weight


        ###############
        # pass

    def resampleParticle(self):
        """
        Description:
            Perform resample to get a new list of particles 
        """
        particles_new = list()

        ## TODO #####

        # Multinominal 
        # cumulative_weights = np.cumsum([p.weight for p in self.particles])
        # for i in range(self.num_particles):
        #     random_weight = np.random.uniform(0, cumulative_weights[-1])
        #     index = bisect.bisect_left(cumulative_weights, random_weight)
        #     x = self.particles[index].x
        #     y = self.particles[index].y
        #     h = self.particles[index].heading
        #     particles_new.append(Particle(x, y, heading=h, maze=self.world, sensor_limit=self.sensor_limit, noisy=True))
        

        # Systematic resampling
        cumulative_weights = np.cumsum([p.weight for p in self.particles])
        cumulative_weights = cumulative_weights / cumulative_weights[-1]

        positions = (np.arange(self.num_particles) + np.random.uniform()) / self.num_particles
        indexes = np.zeros(self.num_particles, "i")

        i, j = 0, 0
        while i < self.num_particles:
            if positions[i] < cumulative_weights[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1
        for k in indexes:
            new_particle = Particle(self.particles[k].x, self.particles[k].y, heading=self.particles[k].heading, maze=self.world, sensor_limit=self.sensor_limit, noisy=True)

            # std = 0.05
            # new_particle.x = new_particle.add_noise(x = new_particle.x, std = std)
            # new_particle.y = new_particle.add_noise(x = new_particle.y, std = std)
            # new_particle.heading = new_particle.add_noise(x = new_particle.heading, std = np.pi * 2 * std)
            # new_particle.fix_invalid_particles()

            particles_new.append(new_particle)


        ###############

        self.particles = particles_new

    def particleMotionModel(self):
        """
        Description:
            Estimate the next state for each particle according to the control input from actual robot 
            You can either use ode function or vehicle_dynamics function provided above
        """
        ## TODO #####
        
        ts = 0.01
        
        for control in self.control[self.control_old_len : self.control_new_len]:
            v = control[0]
            delta = control[1]

            for particle in self.particles:
                vars = [particle.x, particle.y, particle.heading]
                new_state = vehicle_dynamics(ts, vars, v, delta)
                
                particle.x += new_state[0]
                particle.y += new_state[1]
                particle.heading += new_state[2]

        ###############
        # pass


    def runFilter(self):
        """
        Description:
            Run PF localization
        """
        count = 0 
        errors_pos = []
        errors_ori = []
        iterations = []
        times = []

        fig, (ax1, ax2) = plt.subplots(2, 1)
        fig.suptitle('Position and Orientation Error over Time')
        line1, = ax1.plot([], [], 'r-', label="Position Error")
        line2, = ax2.plot([], [], 'b-', label="Orientation Error")

        ax1.set_xlim(0, 100)
        ax1.set_ylim(0, 50)  # Assume position error won't exceed 10 meters
        ax1.set_ylabel('Position Error [m]')
        ax1.legend()

        ax2.set_xlim(0, 100)
        ax2.set_ylim(0, np.pi)  # Assume orientation error won't exceed pi radians
        ax2.set_ylabel('Orientation Error [rad]')
        ax2.legend()
        # ax2.set_xlabel('Iteration')
        ax2.set_xlabel('Time [s]')

        start_time = time.time()
        end_time = start_time

        while True:
            # ## TODO:

            try:
                # (i) Implement Section 3.2.2.
                self.control_new_len = len(self.control)

                if self.control_new_len != self.control_old_len:
                    self.particleMotionModel()
                    self.control_old_len = self.control_new_len
                
                reading = self.bob.read_sensor()
                if reading is None:
                    continue

                self.updateWeight(reading)
                self.resampleParticle()

                # (ii) Display robot and particles on map.
                self.world.clear_objects()
                self.world.show_particles(self.particles)
                self.world.show_robot(self.bob)
                estimated_x, estimated_y, estimated_h_deg = self.world.show_estimated_location(self.particles)
                estimated_h = estimated_h_deg * np.pi / 180

                # (iii) Compute and save position/heading error to plot.
                actual_x, actual_y, actual_h = self.bob.x, self.bob.y, self.bob.heading
                position_error = np.sqrt((actual_x - estimated_x) ** 2 + (actual_y - estimated_y) ** 2)
                orientation_error = np.abs(actual_h - estimated_h)
                if orientation_error > 2 * np.pi:
                    orientation_error = orientation_error - 2 * np.pi
                if orientation_error > np.pi:
                    orientation_error = 2 * np.pi - orientation_error


                timediff = end_time - start_time

                errors_pos.append(position_error)
                errors_ori.append(orientation_error)
                iterations.append(count)
                times.append(timediff)


                # line1.set_data(iterations, errors_pos)
                # line2.set_data(iterations, errors_ori)
                line1.set_data(times, errors_pos)
                line2.set_data(times, errors_ori)

                # ax1.set_xlim(0, count + 1)
                # ax2.set_xlim(0, count + 1)
                ax1.set_xlim(0, math.ceil(timediff))
                ax2.set_xlim(0, math.ceil(timediff))

                plt.pause(0.01)
                count += 1
                end_time = time.time()

                # if len(iterations) == 500:
                if timediff > 300:
                    # print("\nReach 500 iterations")
                    print("\n300sec passed")
                    break

            # Press Ctrl+C to stop
            except KeyboardInterrupt:
                print("\nStop by Ctrl + C")
                break

        # end_time = time.time()
        print("\n====================")
        print("Time:", timediff)
        print("Iter:", len(iterations))
        print("Ave pos err:", np.mean(errors_pos))
        print("Ave ori err:", np.mean(errors_ori))
        print("Figure saved! Press Ctrl+\\ to quit")
        print("====================\n")
        plt.savefig(str(self.num_particles) + "-" + str(self.sensor_limit) + "-" + datetime.datetime.now().strftime("%H:%M:%S") +".png")
        plt.show()

            ###############



