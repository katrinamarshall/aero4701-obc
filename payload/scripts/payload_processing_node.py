""" Payload Processing Node
Processes LiDAR data to output
"""

import time

from std_msgs.msg import String
from payload.msg import lidar_raw_data
from payload.msg import lidar_raw_data_single
from payload.msg import sat_info
from payload.msg import payload_data
from payload.msg import found_debris

import rospy

# from debra.msg import command_msg, satellite_pose, payload_data

import numpy as np
# import matplotlib.pyplot as plt
import datetime as dt

from constants import *

# ------------------------------ CONSTANTS ----------------------------------
SENSOR_RANGE = 200 # mm
SENSOR_MIN = 20 #mm
FOV = 45 # degrees
PIXELS_1D = 8
LIDAR_FREQ = 10 # Hz
LIDAR_PERIOD = 1e6/LIDAR_FREQ # microseconds
VEL_UNKNOWN = np.array([-1,-1,-1])
PLOT = False
CHECK_RESIDUALS = False
TEST_VEL=False
DEBUGGING_MODE = True
NUM_LIDARS_ACTIVE = 4
OBJECT_TOLERANCE = 60
# ----------------------------------------------------------------------------


# debra has a server for satellite state 
# payload has a client 

class ProcessingMaths():
    def __init__(self):
        return


    def cosine_rule(self, a, b, theta_degrees):
        """
        finds c, given opposite angle to side in degrees
        """
        theta_radians = np.radians(theta_degrees)
        cos_theta = np.cos(theta_radians)
        c_squared = a**2 + b**2 - 2 * a * b * cos_theta
        return np.sqrt(c_squared)

       
    def polar_to_cartesian(self, polar):
        """Converts a vector in the polar coordinate system to the cartesian coordinate system.
        
        The unit of the calculated cartesian vector is the same as the unit of the range.

        Args:
            polar (np.ndarray): A vector containing elevation (rad), 
            azimuth (rad), and range.

        Returns:
            np.ndarray: A vector in the cartesian coordinate system.
        """
        elev, azim, rng = polar

        x = rng * np.cos(azim) * np.cos(elev)
        y = rng * np.sin(azim) * np.cos(elev)
        z = rng * np.sin(elev)
        
        cart = np.array([x, y, z])
        return cart

    
    def rot3_y(self, theta: float, degrees: bool = False):
        """Returns a 3x3 rotation matrix about the y-axis.

        Args:
            theta (float): The angle of rotation.
            degrees (bool, optional): Whether the angle is in
            degrees or radians. Defaults to False.

        Returns:
            np.ndarray: The Y-rotation matrix.
        """
        theta = np.radians(theta) if degrees else theta
        
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

    def direction_cosine_matrix(self,
        new: tuple,
        original: tuple = (
            np.array([1, 0, 0]),
            np.array([0, 1, 0]),
            np.array([0, 0, 1])
        )
    ):
        """Calculates the direction cosine matrix from original to new coordinate basis.

        Args:
            new (tuple): The (i, j, k) unit vectors of the new basis as numpy arrays.
            original (tuple, optional): The (i, j, k) unit vector of the old basis. 
                Defaults to ([1, 0, 0], [0, 1, 0], [0, 0, 1]) as numpy arrays.

        Returns:
            np.ndarray: The direction cosine matrix from original to new.
        """
        M11 = np.dot(new[0], original[0])
        M12 = np.dot(new[0], original[1])
        M13 = np.dot(new[0], original[2])
        
        M21 = np.dot(new[1], original[0])
        M22 = np.dot(new[1], original[1])
        M23 = np.dot(new[1], original[2])
        
        M31 = np.dot(new[2], original[0])
        M32 = np.dot(new[2], original[1])
        M33 = np.dot(new[2], original[2])
        
        return np.array([
            [M11, M12, M13],
            [M21, M22, M23],
            [M31, M32, M33]
        ])
     

    def quat_product(self, q: np.ndarray, p: np.ndarray) :
        """Multiplies two quaternions using the Hamilton product.

        Args:
            q (np.ndarray):
            p (np.ndarray):

        Returns:
            np.ndarray: The product of q and p.
        """
        q_scalar, q_vec  = q[0], q[1:]
        p_scalar, p_vec = p[0], p[1:]
        
        vec = q_scalar * p_vec + p_scalar * q_vec + np.cross(q_vec, p_vec)
        scalar = q_scalar * p_scalar - np.dot(q_vec, p_vec)
        
        return np.array([scalar, *vec])

        
    def LVLH_to_ECI(self,x_lvlh: np.ndarray,r_sat: np.ndarray,v_sat: np.ndarray,*,rotation_only: bool = False)  :
        """Converts a vector in Local Vertical Local Horizontal
        coordinates to ECI coordinates.

        Args:
            x_lvlh (np.ndarray): Vector in LVLH coordinates.
            r_sat (np.ndarray): Position vector of satellite in ECI coordinates.
            v_sat (np.ndarray): Velocity vector of satellite in ECI coordinates.
            rotation_only (bool, optional): If True, only the rotation to the

        Returns:
            np.ndarray: Vector in ECI coordinates.
        """
        
        if not rotation_only:
            x_rel = x_lvlh + r_sat
        else:
            x_rel = x_lvlh
        
        z_hat = - r_sat / np.linalg.norm(r_sat)     # Radial unit vector
        h = np.cross(r_sat, v_sat)                  # Specific angular momentum
        y_hat = - h / np.linalg.norm(h)             # negative angular momentum unit vector
        x_hat = np.cross(y_hat, z_hat)
        
        # DCM from LVLH to ECI
        eci_dcm = self.direction_cosine_matrix(
            new=(np.array([1,0,0]), np.array([0,1,0]), np.array([0,0,1])),
            original=(x_hat, y_hat, z_hat)
        )
            
        return eci_dcm @ x_rel    

    
    def quat_rotate(self, x: np.ndarray, q: np.ndarray):
        """Rotates a vector by a quaternion.
        
        Quaternion assumed to be in the form [w, x, y, z]
        where w scalar and (x, y, z) vector.

        Args:
            x (np.ndarray): The vector to rotate.
            q (np.ndarray): The quaternion to rotate by.

        Returns:
            np.ndarray: The rotated vector.
        """
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])      # Calculate conjugate of q
        x_quat = np.array([0, *x])                          # Convert vector to quaternion
        
        x_prime = self.quat_product(self.quat_product(q, x_quat), q_conj)
        
        return x_prime[1:]
    
    def bodyfixed_to_LVLH(self,
        x_bf: np.ndarray,
        attitude: np.ndarray
    ):
        """Converts a vector in the bodyfixed frame into
        the LVLH frame

        Args:
            x_bf (np.ndarray): A vector in the body fixed frame
            attitude (np.ndarray): The attitude quaternion

        Returns:
            np.ndarray: A vector in the LVLH frame
        """
        att_conj = np.array([
            attitude[0],
            *(-attitude[1:])
        ])
        
        return self.quat_rotate(x_bf, att_conj)

    def bodyfixed_to_ECI(self,
        x_bf: np.ndarray,
        r_sat: np.ndarray,
        v_sat: np.ndarray,
        attitude: np.ndarray,
        *,
        rotation_only: bool = False
    ) :
        """Converts a vector in the body fixed frame to the ECI frame

        Args:
            x_bf (np.ndarray): A vector in the body fixed frame
            r_sat (np.ndarray): Position vector of satellite in ECI coordinates.
            v_sat (np.ndarray): Velocity vector of satellite in ECI coordinates.
            attitude (np.ndarray): The attitude quaternion
            rotation_only (bool, optional): If True, only the rotation to the
            ECI frame is applied. Defaults to False.

        Returns:
            np.ndarray: A vector in the ECI frame
        """
        x_lvlh = self.bodyfixed_to_LVLH(x_bf, attitude)
        return self.LVLH_to_ECI(x_lvlh, r_sat, v_sat, rotation_only=rotation_only)

        


    
            
class PayloadProcessing():
    """ gets lidar data and does all the processing"""
    def __init__(self):

        # -------------------- Publishers --------------------------------------------
        # processed debris data for downlink
        self.pub_payload_data = rospy.Publisher('/payload_data', payload_data, queue_size=1)
        # polar coordinates of found debris objects
        self.pub_blobs_detected = rospy.Publisher('/found_debris', found_debris, queue_size=100)

        # -------------------- Subscribers --------------------------------------------

        # rospy.Subscriber('/raw_lidar_data', lidar_raw_data, self.callback_raw_lidar) # for all lidars
        rospy.Subscriber('/raw_lidar_data', lidar_raw_data, self.callback_raw_lidar) # for just one lidar

        # make fake sat info publisher
        rospy.Subscriber('/sat_info', sat_info, self.callback_sat_info) # satellite pose data (pos, vel, att, time)

        # subscribe to a reset messages which would clear all arrays when not in debris detection mode
        rospy.Subscriber('/operation_state', String, self.callback_reset)

     
        self.Process = ProcessingMaths()
        
 

        # Logging fields
        self._raw_lidar = np.zeros((4,8,8)) # lidar 1 is raw_lidar[0]
        self._lidar_status = np.zeros((4,8,8))
        # self._raw_lidar_1 = [] # for lidar 1
        # self._raw_lidar_2 = [] # for lidar 2
        # self._raw_lidar_3 = [] # for lidar 3
        # self._raw_lidar_4 = [] # for lidar 4
        self._lidar_labels = []

        # Callback variables
        # self._lidar_message_received = False

        # Storage fields for processed debris
        self._debris_count = 0

        self._prev_detection_times = np.zeros((4,1))
        self._prev_detection_eci = np.zeros((4,3))
        self._prev_detection_relative_pos = np.zeros((4,3))



        # updated so just store current detections
        self._debris_eci_pos = []
        self._debris_sizes = [] 
        self._debris_velocities = []
        self._detection_times = []
        self._debris_rel_velocities = []

        # logging satellite state fields - should only contain most recent state
        self._sat_pos = VEL_UNKNOWN
        self._sat_vel = VEL_UNKNOWN
        self._sat_att = np.array([1,0,0,0])
        self._sat_time = 0
        print("Processing initialised!")
        return
    
    def callback_sat_info(self, data):

        self._sat_pos = np.array(data.position)
        self._sat_vel = np.array(data.velocity)
        self._sat_att = np.array(data.attitude)
        self._sat_time = float(data.timestamp)
        # print("Position received", self.sat_pos)
        return


    def callback_reset(self, state):
        if state.data != "Debris Detection":
            self._debris_count = 0
            self._prev_detection_times = np.zeros((4,1))
            self._prev_detection_eci = np.zeros((4,3))
            self._prev_detection_relative_pos = np.zeros((4,3))
            self._debris_eci_pos = []
            self._debris_sizes = [] 
            self._debris_velocities = []
            self._detection_times = []
            self._debris_rel_velocities = []
        return



    def callback_raw_lidar(self, raw_data):
        # self._lidar_message_received = True 

        # try:
        #     satellite_state_data = rospy.ServiceProxy('satellite_pose', SatellitePose)
        
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)

        # print(np.array(raw_data.distances_1).reshape(8,8))
     

        self._raw_lidar[0] = np.array(raw_data.distances_1).reshape(8,8)
        self._raw_lidar[1] = (np.array(raw_data.distances_2).reshape(8,8))
        self._raw_lidar[2] = (np.array(raw_data.distances_3).reshape(8,8))
        self._raw_lidar[3] = (np.array(raw_data.distances_4).reshape(8,8))

        self._lidar_status[0] = np.array(raw_data.status_1).reshape(8,8)
        self._lidar_status[1] = np.array(raw_data.status_2).reshape(8,8)
        self._lidar_status[2] = np.array(raw_data.status_3).reshape(8,8)
        self._lidar_status[3] = np.array(raw_data.status_4).reshape(8,8)



        # self._lidar_labels.append(raw_data.label)

        self.process_debris(self._sat_pos, self._sat_vel, self._sat_att, self._sat_time)
        # self._lidar_labels.remove
        return

    def payload_data_downlink(self, timestamp, debris_count, x_eci, debris_size, abs_vel, rel_vel):
        self.pub_payload_data.publish(x_eci[0], x_eci[1], x_eci[2], rel_vel[0], rel_vel[1], rel_vel[2], debris_size, timestamp, debris_count)
        # print("sent to ground!")
        return

        
    def is_valid_position(self, data, visited, i, j, sensor_range, current_value):
        """ Checks if a position is valid for exploration
        Inputs:
            data: 2D array of LiDAR data in mm (ints)
            visited: 2D array shaped same as data with boolean already visited 
            i: row index for current position
            j: column index for current position
            sensor_range: the maximum valid sensor range
            current_value: the LiDAR measurement for the adjacent pixel just investigated
        Output:
            result: boolean - valid for exploration
        """
        rows, cols = data.shape
        # Check if the position is within the grid bounds
        within_grid = (i >= 0 and i < rows) and (j >= 0 and j < cols)
        # Check has a value within the range of lidar sensor
        if (within_grid):
            # TODO need to fine tune tolerance based on lidar
            tolerance = OBJECT_TOLERANCE # 5cm change per pixel max grad for same object
            within_range = (data[i][j] < sensor_range) and (abs(data[i][j] - current_value) < tolerance) and (data[i][j] > SENSOR_MIN)
            # Check has not been visited before
            not_visited = not visited[i][j]
            result = (not_visited and within_grid and within_range)
        else:
            result = within_grid
        return result

    def explore_blob(self, data, visited, i, j, sensor_range, start_i, start_j):
        """
        Use DFS to explore a detected object
        Inputs: 
            data: 2D array of LiDAR data in mm (ints)
            visited: 2D array shaped same as data with boolean already visited 
            i: row index for current position
            j: column index for current position
            sensor_range: the maximum valid sensor range
            start_i: row index for first pixel where object found
            start_j: column index for first pixel where object found

        Output:
            maximum diameter of the detected object in pixels in x coordinate
            maximum diameter of the detected object in pixels in y coordinate
            average value of found object (from LiDAR readings) (mm)
            total size of object in pixels
        """
        # Initialize a stack to keep track of positions to explore
        stack = [(i, j)]
        visited[i, j] = True
        
        # Initialize variables to track properties of the blob
        max_diameter_x = 0
        max_diameter_y = 0
        total_value = data[i, j]
        count = 1
        # Define the directions (up, down, left, right)
        directions = [[1, 0], [-1, 0], [0, 1], [0, -1]]
        
        # Explore positions in the stack until it's empty
        while stack:
            # Pop the current position from the stack
            current_i, current_j = stack.pop()
            current_value = data[current_i, current_j]
            
            # Calculate the distance from the starting position to the current position
            diameter_x = abs(current_j - start_j)
            diameter_y = abs(current_i - start_i)
            # diameter = max(abs(current_i - start_i), abs(current_j - start_j))
            max_diameter_x = max(max_diameter_x, diameter_x)
            max_diameter_y = max(max_diameter_y, diameter_y)
            
            # Explore neighbors in each direction
            for di, dj in directions:
                neighbor_i, neighbor_j = current_i + di, current_j + dj
                
                # Check if the neighbor is valid for exploration and has not been visited
                if self.is_valid_position(data, visited, neighbor_i, neighbor_j, sensor_range, current_value):
                    visited[neighbor_i, neighbor_j] = True
                    stack.append((neighbor_i, neighbor_j))
                    total_value += data[neighbor_i, neighbor_j]
                    count += 1
        
        # include current pixel in diameter  
        max_diameter_x = max_diameter_x + 1
        max_diameter_y = max_diameter_y + 1
        # Return the maximum distance and average value found from the starting position
        return max_diameter_x, max_diameter_y, total_value / count, count

    def find_blobs(self, data, status, sensor_range):
        """
        Searches LiDAR grid image to detect objects
        Inputs:
            data: 2D array of LiDAR data in mm (ints)
            sensor_range: the maximum valid sensor range in mm
        Output:
            blob_diameters: array of maximum diameter of detected objects (blobs)
            blob_positions: array of x-y positions on grid of detected objects
            blob_avg_values: array of average lidar sensor readings of detected objects
        """
        rows, cols = data.shape
        # Initialize a boolean array to keep track of visited positions
        visited = np.zeros_like(data, dtype=bool)
        blob_diameters = []
        blob_positions = []
        blob_avg_values = []
        
        # Iterate through each position in the grid
        for i in range(rows):
            for j in range(cols):
                # If the position has a value above or equal to the threshold and has not been visited
                if (data[i][j]) < sensor_range and data[i][j] > SENSOR_MIN and not visited[i][j] and status[i][j] != 255:
                    #HELP MESSAGE DO THIS ASAP NOISE YAY
                    # Explore the blob starting from this position
                    max_diameter_x, max_diameter_y, avg_value, size = self.explore_blob(data, visited, i, j, sensor_range, i, j)
                    max_diameter = max(max_diameter_x, max_diameter_y)

                    row = i
                    col = j
                    # If x-y dimension more than pixels in size use centroid as position coordinate
                    if max_diameter_y > 2:
                        # Calculate the centroid position assuming regular object
                        row = np.floor(i + max_diameter_y/2)
                        if row > 7:
                            row = i
                        # row = min(row,7)
                        
                    if max_diameter_x > 2:
                        col = np.floor(j + max_diameter_x/2)
                        if col > 7:
                            col = j
                        # col = min(col,7)

                    # Add the maximum distance, centroid position, and average value to their respective lists
                    blob_diameters.append(max_diameter)
                    blob_positions.append([row, col])
                    blob_avg_values.append(avg_value)
        
        # Return the maximum distances (diameters), centroid positions, and average values of all blobs found
        return np.array(blob_diameters), np.array(blob_positions), np.array(blob_avg_values)

    
    def image2polar(self, lidar_label, blob_positions, blob_avg_values, blob_diameters, fov, num_pixels_1D):
        """
        Converts given position of object in pixel image to polar coordinates from sensor
        Prints detected object characteristics
        Inputs:
            blob_positions: The centroid position of detected objects as an array of [x,y] pixels
            blob_avg_values: Array of average lidar readings of detected objects (mm)
            blob_diameters: Maximum diameter of object detected in pixels
            fov: field of view of sensor in degrees
            num_pixels_1D: number of pixels in one dimension of lidar image
        
        Outputs:
            debris_pos: Array of polar coordinate positions of detected objects from sensor
            sizes: Array of size of maximum diameter of objects in mm
        """
        debris_pos = np.zeros((len(blob_avg_values), 3))
        sizes = np.zeros_like(blob_diameters)
        for i in range(len(blob_positions)):
            # i is row = vertical; j is column = horizontal
            y, x = blob_positions[i]
            dist = blob_avg_values[i] # mm
            r = dist/1000 # m

            # resolution is 1 pixel at this range using cos rule
            resolution =self.Process.cosine_rule(dist,dist,fov)/num_pixels_1D

            theta = np.radians((fov/num_pixels_1D) * x - fov/2)
            phi = np.radians((fov/num_pixels_1D) * y - fov/2)
            debris_pos[i] = [phi,theta,r]

            debris_size = blob_diameters[i]*resolution #[mm]
            sizes[i] = debris_size
            if DEBUGGING_MODE:
                # print("------------------------------------------------------------------------------------------------")
                print(f"Debris object detected max diameter {debris_size:.1f}mm ({blob_diameters[i]} pixels), at r:{dist:.3f}mm, theta:{np.degrees(theta):.2f}°, phi:{np.degrees(phi):.2f}°\n")
                # print("------------------------------------------------------------------------------------------------")
            # print(lidar_label, dist, np.degrees(theta), np.degrees(phi), debris_size, blob_diameters[i], blob_positions[i])
            self.pub_blobs_detected.publish(lidar_label, dist, np.degrees(theta), np.degrees(phi), debris_size, blob_diameters[i], [int(blob_positions[i][0]), int(blob_positions[i][1] )])

        return np.array(debris_pos), sizes
    

    def process_debris(self, sat_pos, sat_vel, attitude, sat_time):
        print("Running...")
        # assume all are synchronised
        # make version for 4 and version for 1
        # if self._lidar_message_received == True:
        if True:
            # print("going through data")
            # rospy.spin()

            # self._lidar_message_received = False
            
            # find how many new lidar readings are to be processed
            # num_new_readings = len(self._lidar_labels) - self._all_readings_count
            # print("Num new readings", num_new_readings)
            # self._all_readings_count = len(self._lidar_labels)
            # FOR IF WANT 4 DIFFERENT LIDARS PUBLISHING SEPARATELY
            num_new_readings = NUM_LIDARS_ACTIVE
            # print(self._lidar_labels_prev_detections)

            for n in range(num_new_readings):
                # print("found new reading")
                # rospy.spin()


                # data = np.array(self._raw_lidar[-num_new_readings+n])
                # data = np.array(self._raw_lidar[n])
                data = np.array(self._raw_lidar[n])
                status = np.array(self._lidar_status[n])


                # run blob detection algorithm on new data
                timestamp = sat_time

                blob_diameters, blob_positions, blob_avg_values = self.find_blobs(data, status, SENSOR_RANGE)
                # print("found some blobs")

                # check if any debris has been found 
                if len(blob_diameters) > 0:
                    # print(self._raw_lidar[n])


                    lidar_label = n + 1


                    # Found new debris - add whichever lidar this has come from
                    # self._lidar_labels_prev_detections.append(self._lidar_labels[-num_new_readings+n])
                    # self._lidar_labels_prev_detections.append(lidar_label)

                    # if so, find its coordinates
                    debris_pos_polar, debris_sizes = self.image2polar(lidar_label, blob_positions, blob_avg_values,blob_diameters, FOV, PIXELS_1D)
                    # if CHECK_RESIDUALS == True:
                    #     debris_pos_polar_noisy, debris_sizes_noisy = self.image2polar(blob_positions_noisy, blob_avg_values_noisy,blob_diameters_noisy, FOV, PIXELS_1D)

                    # plotted = False
                    # if PLOT == True and plotted == False:
                    #     self.plot_lidar_data(data,blob_positions)
                    #     plotted = True
           
                    # if DEBUGGING_MODE:
                    #     print(f"----- SAT POS: {sat_pos} ECI coords -------------")
                    #     print(f"VEL: {sat_vel}, ATT: {attitude}")

                    s = 0 # counter
                    # For each debris object set of coordinates

                    # if sum(self._prev_detection_eci[n]) != -3 and sum(self._prev_detection_eci[n]) != 0:
                    #     print("Same object found")
                    # else:
                    for x_polar in debris_pos_polar:
                    
                        # update debris count

                        # identify which lidar this came from
                        # lidar_label = self._lidar_labels_prev_detections[n]
                        # convert to xyz coordinates
                        debris_pos_cart = self.Process.polar_to_cartesian(x_polar)
                
                        # convert these to body fixed based on lidar positioning
                        # in body-fixed frame y points straight down
                        debris_pos_body_fixed = self.Process.rot3_y((lidar_label-1)*np.pi/2)@debris_pos_cart

                        # convert body fixed coordinates to ECI
                        debris_pos_eci_rel = self.Process.bodyfixed_to_ECI(debris_pos_body_fixed,sat_pos,sat_vel,attitude,rotation_only=True)
                        debris_pos_eci = debris_pos_eci_rel + sat_pos

                    

                    # at new timestep, check previous timestep to find the velocity of debris object

                        if len(debris_pos_polar) > 1:
                            print(f"Cannot find speed, {len(debris_pos_polar)} objects in frame")
                            self._debris_count += len(debris_pos_polar)

                            abs_vel = VEL_UNKNOWN
                            rel_vel = VEL_UNKNOWN
                        else:
                            # add size info here
                            self._debris_count += 1
                            abs_vel, rel_vel = self.find_vels_hw(lidar_label, debris_pos_eci, debris_pos_cart, timestamp)
                        
                        # if only one object in frame store to check if its moving in next round
                        if len(debris_pos_polar) == 1:
                            # print("storing eci pos for next time!")
                            self._prev_detection_eci[n] = debris_pos_eci
                            self._prev_detection_times[n] = timestamp
                            self._prev_detection_relative_pos[n] = debris_pos_cart
                        # else reset because we don't care about past debris anymore
                        else:
                            # print("resetting because this is object no longer needed")
                            self._prev_detection_eci[n] = VEL_UNKNOWN
                            self._prev_detection_times[n] = 0
                            self._prev_detection_relative_pos[n] = VEL_UNKNOWN

                            
                
                    
                        # Log detected debris
                        print("-----------------------------------------------------------------------------------------------")
                        print("------------------------------ TRANSMIT DATA --------------------------------------------------")
                        print(f"From LiDAR {lidar_label}: {timestamp}: ")
                        print(f"DEBRIS FOUND with max diam {debris_sizes[s]}mm at {debris_pos_eci} ECI ")
                        if sum(abs_vel - VEL_UNKNOWN) != 0:
                            print(f"Travelling at absolute velocity in ECI frame {abs_vel} m/s")
                            print(f"Relative to DEBRA speed {np.linalg.norm(rel_vel)} m/s, ")
                            print(f" {rel_vel} m/s")
                            print(f"v satellite {sat_vel}")
                    

                        print("-----------------------------------------------------------------------------------------------")
                        print("\n")
                        self.payload_data_downlink(timestamp, self._debris_count, debris_pos_eci, debris_sizes[s], abs_vel, rel_vel)
                        s += 1


        return
    

    def find_vels_hw(self, lidar_label, debris_pos_eci, debris_pos_cart, timestamp):
        vel_abs = VEL_UNKNOWN
        vel_rel = VEL_UNKNOWN

        # If object detected at previous timestep
        # if timestamp - self._prev_detection_times[lidar_label-1] > LIDAR_PERIOD:
        if self._prev_detection_times[lidar_label - 1] >= 0:

        # check if debris size is same - check if same object as before
        # if abs(size - self._debris_sizes[overall_index]) > 100:
        #     print("Debris object detected a different size, not the same object in frame - cannot find speed")
            # else:
            time_diff_micro = (timestamp - self._prev_detection_times[lidar_label - 1]) 
            
            if time_diff_micro < 0:
                time_diff_micro += 1000000
            
            if time_diff_micro == 0:
                print("Speed cannot be determined, larger timestep required")
            else:
                time_diff_s = time_diff_micro/1e6
                # print("TIME DIFF SEC", time_diff_s)

                pos_diff_abs = (debris_pos_eci - self._prev_detection_eci[lidar_label - 1])
                pos_diff_rel = (debris_pos_cart - self._prev_detection_relative_pos[lidar_label - 1])

                vel_abs = pos_diff_abs/time_diff_s # m/s
                vel_rel = pos_diff_rel/time_diff_s # m/s
                self._debris_count -= 1
                # self._debris_count 



                    # speed = np.linalg.norm(vel)
        else:
            print(f"Most recent detection from LiDAR {lidar_label} more than one sensor period ago, cannot find speed") 
        return vel_abs, vel_rel




    def find_abs_vel_sim(self, lidar_label,debris_pos_eci, size, timestamp):
        vel = VEL_UNKNOWN 
        # If object was detected in previous timestep
        if lidar_label in self._prev_detections:                            
            # make sure its only doing for a certain lidar - find lidar label in previous detections
            prev_found_index = self._prev_detections.index(lidar_label)
            overall_index = -len(self._prev_detections)+prev_found_index
            # if previous detection greater than a lidar timestep before, no longer consider for next velocity calculations
            if (timestamp - self._detection_times[overall_index]).microseconds > LIDAR_PERIOD:
                self._prev_detections.remove(self._prev_detections[prev_found_index])

                if lidar_label not in self._prev_detections: 
                    print(f"Most recent detection from LiDAR {lidar_label} more than one sensor period ago, cannot find speed") 
                    return vel

                # update indices
                prev_found_index = self._prev_detections.index(lidar_label)
                overall_index = -len(self._prev_detections)+prev_found_index

   
            # check if debris size is same - check if same object as before
            if abs(size - self._debris_sizes[overall_index]) > 100:
                print("Debris object detected a different size, not the same object in frame - cannot find speed")
            else:
                time_diff = 1e-6*(timestamp - self._detection_times[overall_index]).microseconds 
                # print(time_diff)
                if time_diff == 0:
                    print("Speed cannot be determined, larger timestep required")
                else:
                    pos_diff = (debris_pos_eci - self._debris_eci_pos[overall_index])
                    vel = pos_diff/time_diff # m/s
                    # speed = np.linalg.norm(vel)
        return vel
    
    


if __name__ == "__main__":
    rospy.init_node("payload_processing")
    myProcessing = PayloadProcessing()
    # rospy.Timer(rospy.Duration(1.0/2.0), myLidar.)
    rospy.spin()
