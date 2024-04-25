import numpy as np
# import matplotlib.pyplot as plt
import datetime as dt

from rotation import *
from _conversion import *

from lidar_contants import *

import cProfile
import time

def gen_test_data(_num_pixels, _debris_freq, current_time, test_vel=False, empty = False):
    eps = 0.01
    if test_vel == True:
        # if testing velocity we just want one cluster of debris
        test_data = np.full((_num_pixels, _num_pixels), SENSOR_RANGE)
        if current_time < 1:
            cluster_size1 = np.random.randint(5)
            clump1_center = np.random.randint(0, _num_pixels - cluster_size1, size=2)
            test_data[clump1_center[0]:clump1_center[0] + cluster_size1, clump1_center[1]:clump1_center[1] + cluster_size1] = 100
            # plt.imshow(test_data, cmap='viridis', interpolation= 'nearest')
            # plt.colorbar(label='Value')
            # # plt.scatter(blob_positions[:,1], blob_positions[:,0], marker='x', color = 'r')
            # plt.xlabel('Column')
            # plt.ylabel('Row')
            # plt.title('2D Plot of test_data')
            # plt.show()
    else:
        if current_time % _debris_freq > eps or empty == True:
            test_data = np.full((_num_pixels, _num_pixels), SENSOR_RANGE)
        elif _num_pixels == 8: 
            # 8 x 8 input data
            test_data = np.array([[1, 2, 4000, 4000, 1, 2, 4000, 4000], 
                            [1, 1, 8, 4000, 4000, 4000, 4000, 4000],
                            [1, 8, 8, 4000, 4000, 4000, 4000, 4000],
                            [1, 8, 8, 12, 4000, 4000, 4000, 4000],
                            [4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000],
                            [4000, 4000, 4000, 4000, 4000, 80, 80, 80], 
                            [4000, 50, 4000, 4000, 4000, 4000, 4000, 4000], 
                            [4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000]])  
                                                                                                                            
        else:
            # test_data = np.random.uniform(0.1, 4000, size=(64, 64))
            # test_data_8 = np.array([[0.1, 0.2, 4000, 4000], [0.1, 0.1, 4000, 4000], [4000, 4000, 4000, 4000], [4000, 4000, 4000, 4000]])
            # Create a 1024x1024 array filled with 4000 
            test_data = np.full((_num_pixels, _num_pixels), SENSOR_RANGE)
            # Define cluster sizes
            cluster_size1 = 100
            cluster_size2 = 50
            # Generate random positions for the clumps
            clump_0_1_center = np.random.randint(0, 1024 - cluster_size1, size=2)
            clump_10_center = np.random.randint(0, 1024 - cluster_size2, size=2)
            # Set values for clump 0.1
            test_data[clump_0_1_center[0]:clump_0_1_center[0] + cluster_size1, clump_0_1_center[1]:clump_0_1_center[1] + cluster_size1] = 100
            # Set values for clump 10
            test_data[clump_10_center[0]:clump_10_center[0] + cluster_size2, clump_10_center[1]:clump_10_center[1] + cluster_size2] = 10

    return test_data



# Logging fields
timestamp = dt.datetime(2024,4,25,17,27,2,881934)
current_time = 0
_num_pixels = 8
_debris_freq = 1/15
test_data = gen_test_data(_num_pixels, _debris_freq, current_time, test_vel= True)
# print(test_data)
_raw_lidar = np.array([gen_test_data(_num_pixels, _debris_freq, current_time, test_vel= True)])
                    #    gen_test_data(_num_pixels, _debris_freq, current_time, test_vel= True),
                    #    gen_test_data(_num_pixels, _debris_freq, current_time, test_vel= True),
                    #    gen_test_data(_num_pixels, _debris_freq, current_time, test_vel= True),
                    #    gen_test_data(_num_pixels, _debris_freq, current_time, test_vel= True),
                    #    gen_test_data(_num_pixels, _debris_freq, current_time, test_vel= True),
                    #    gen_test_data(_num_pixels, _debris_freq, current_time, test_vel= True),
                    #    gen_test_data(_num_pixels, _debris_freq, current_time, test_vel= True)])
_lidar_labels = [0]#,1,2,3,0,1,2,3]

# Callback variables
_lidar_message_received = True
_all_readings_count = 0

# Storage fields for processed debris
_debris_count = 0
_lidar_labels_prev_detections = []
_debris_eci_pos = []
_debris_sizes = []
_debris_velocities = []
_detection_times = []
_prev_detections = []

def is_valid_position(data, visited, i, j, sensor_range, current_value):
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
        tolerance = 50 # 5cm change per pixel max grad for same object
        within_range = (data[i][j] < sensor_range) and (abs(data[i][j] - current_value) < tolerance)
        # Check has not been visited before
        not_visited = not visited[i][j]
        result = (not_visited and within_grid and within_range)
    else:
        result = within_grid
    return result

def explore_blob( data, visited, i, j, sensor_range, start_i, start_j):
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
            if is_valid_position(data, visited, neighbor_i, neighbor_j, sensor_range, current_value):
                visited[neighbor_i, neighbor_j] = True
                stack.append((neighbor_i, neighbor_j))
                total_value += data[neighbor_i, neighbor_j]
                count += 1
    
    # include current pixel in diameter  
    max_diameter_x = max_diameter_x + 1
    max_diameter_y = max_diameter_y + 1
    # Return the maximum distance and average value found from the starting position
    return max_diameter_x, max_diameter_y, total_value / count, count

def find_blobs( data, sensor_range):
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
            if (data[i][j]) < sensor_range and not visited[i][j]:
                # Explore the blob starting from this position
                max_diameter_x, max_diameter_y, avg_value, size = explore_blob(data, visited, i, j, sensor_range, i, j)
                max_diameter = max(max_diameter_x, max_diameter_y)

                row = i
                col = j
                # If x-y dimension more than pixels in size use centroid as position coordinate
                if max_diameter_y > 2:
                    # Calculate the centroid position assuming regular object
                    row = np.floor(i + max_diameter_y/2)
                if max_diameter_x > 2:
                    col = np.floor(j + max_diameter_x/2)

                # Add the maximum distance, centroid position, and average value to their respective lists
                blob_diameters.append(max_diameter)
                blob_positions.append([row, col])
                blob_avg_values.append(avg_value)
    
    # Return the maximum distances (diameters), centroid positions, and average values of all blobs found
    return np.array(blob_diameters), np.array(blob_positions), np.array(blob_avg_values)

def cosine_rule( a, b, theta_degrees):
    """
    finds c, given opposite angle to side in degrees
    """
    theta_radians = np.radians(theta_degrees)
    cos_theta = np.cos(theta_radians)
    c_squared = a**2 + b**2 - 2 * a * b * cos_theta
    return np.sqrt(c_squared)

def image2polar( blob_positions, blob_avg_values, blob_diameters, fov, num_pixels_1D):
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
        dist = blob_avg_values[i]
        r = dist/1000 # m

        # resolution is 1 pixel at this range using cos rule
        resolution =cosine_rule(dist,dist,fov)/num_pixels_1D

        theta = np.radians((fov/num_pixels_1D) * x - fov/2)
        phi = np.radians((fov/num_pixels_1D) * y - fov/2)
        debris_pos[i] = [phi,theta,r]

        debris_size = blob_diameters[i]*resolution
        sizes[i] = debris_size
        print("------------------------------------------------------------------------------------------------")
        print(f"Debris object detected diameter {debris_size:.1f}mm ({blob_diameters[i]} pixels), at r:{dist:.3f}mm, and theta:{theta:.2f}°, phi:{phi:.2f}°")
        # print("------------------------------------------------------------------------------------------------")
    return np.array(debris_pos), sizes

def on_tick( timestamp,_all_readings_count, _lidar_message_received, _lidar_labels, _raw_lidar, _lidar_labels_prev_detections, _debris_eci_pos, _debris_sizes,_detection_times,_debris_velocities) -> None:
    """This `on_tick()` method does TODO"""
    if _lidar_message_received == True:
        _lidar_message_received = False
        
        # find how many new lidar readings are to be processed
        num_new_readings = len(_lidar_labels) - _all_readings_count
        _all_readings_count = len(_lidar_labels)
        # print(self._lidar_labels_prev_detections)

        # for each new lidar packet
        for n in range(num_new_readings):
            
            data = np.array(_raw_lidar[-num_new_readings+n])

            # run blob detection algorithm on new data
            blob_diameters, blob_positions, blob_avg_values = find_blobs(data, SENSOR_RANGE)

            # check if any debris has been found - if so print updating debris count - add as var TODO
            if len(blob_diameters) > 0:

                # Found new debris
                _lidar_labels_prev_detections.append(_lidar_labels[-num_new_readings+n])

                # if so, find its coordinates
                debris_pos_polar, debris_sizes = image2polar(blob_positions, blob_avg_values,blob_diameters, FOV, PIXELS_1D)

                plotted = False
                if PLOT == True and plotted == False:
                    # plot_lidar_data(data,blob_positions)
                    plotted = True
                # update debris count
                # _debris_count += len(blob_avg_values)
                #TODO edit this print statement
            
                # get gnss data
                r_sat = np.array([   -9006.10175046, -6926679.95121315 ,    9565.3464146 ])
                print(f"----- SAT POS: {r_sat} ECI coords -------------")
                v_sat = np.array([4559.07205974,   12.98671741, 6062.13485532] )

                # get attitude data
                attitude = np.array([1,0,0,0])
                s = 0 # counter
                # For each debris object set of coordinates
                for x_polar in debris_pos_polar:
                    # identify which lidar this came from
                    lidar_label = _lidar_labels_prev_detections[n]
                    # convert to xyz coordinates
                    debris_pos_cart = polar_to_cartesian(x_polar)
            
                    # convert these to body fixed based on lidar positioning
                    # in body-fixed frame y points straight down
                    debris_pos_body_fixed = rot3_y(lidar_label*np.pi/2)@debris_pos_cart

                    # convert body fixed coordinates to ECI
                    debris_pos_eci_rel = bodyfixed_to_ECI(debris_pos_body_fixed,r_sat,v_sat,attitude,rotation_only=True)
                    debris_pos_eci = debris_pos_eci_rel + r_sat

                    # print timestep and eci data
                    


                # TODO write exception handling for if there are multiple debris objects in one frame 
                # at new timestep, check previous timestep to find the velocity of debris object
                #TODO make very basic for just one object in frame, if there in next time step
                    if len(blob_avg_values) > 1:
                        print("Cannot find speed, more than one object in frame")
                        vel = VEL_UNKNOWN
                    else:
                        vel = find_abs_vel(lidar_label, debris_sizes[s], timestamp)
                    
                    # store which lidar most recent detection from
                    _prev_detections.append(lidar_label)
                    # print(self._prev_detections)

                    timestamp = timestamp
                    x_eci = debris_pos_eci
                    debris_size = debris_sizes[s]
                    _debris_eci_pos.append(x_eci)
                    _debris_sizes.append(debris_size)
                    _detection_times.append(timestamp)
                    _debris_velocities.append(vel)

                    # Log detected debris
                    # print("-----------------------------------------------------------------------------------------------")
                    print("------------------------------ TRANSMIT DATA --------------------------------------------------")
                    print(f"From LiDAR {lidar_label}: {timestamp}: ")
                    print(f"DEBRIS FOUND with max diam {debris_sizes[s]}mm at {debris_pos_eci} ECI ")
                    if sum(_debris_velocities[-1] - VEL_UNKNOWN) != 0:
                        print(f"Travelling at absolute velocity in ECI frame {_debris_velocities[-1]} m/s")
                        print(f"Relative to DEBRA speed {np.linalg.norm(_debris_velocities[-1] - v_sat)} m/s, ")
                    print("-----------------------------------------------------------------------------------------------")
                    print("\n")
                    s += 1

    return 

def find_abs_vel( lidar_label, size, timestamp):
    vel = VEL_UNKNOWN 
    # If object was detected in previous timestep
    if lidar_label in _prev_detections:                            
        # make sure its only doing for a certain lidar - find lidar label in previous detections
        prev_found_index = _prev_detections.index(lidar_label)
        # check if debris size is same - check if same object as before
        if size - _debris_sizes[-len(_prev_detections)+prev_found_index] > 1:
            print("Debris object detected a different size, not the same object in frame - cannot find speed")
        else:
            time_diff = 1e-6*(_detection_times[-1] - _detection_times[prev_found_index]).microseconds 
            if time_diff == 0:
                print("Speed cannot be determined, larger timestep required")
            else:
                pos_diff = (_debris_eci_pos[-1] - _debris_eci_pos[prev_found_index])
                vel = pos_diff/time_diff # m/s
                # speed = np.linalg.norm(vel)
        
            # if previous detection greater than a lidar timestep before, no longer consider for next velocity calculations
            if (timestamp - _detection_times[prev_found_index]).microseconds >= LIDAR_PERIOD:
                _prev_detections.remove(_prev_detections[prev_found_index])
    return vel

# def plot_lidar_data(test_data, blob_positions):
#     plt.figure()
#     plt.imshow(test_data, cmap='viridis', interpolation= 'nearest')
#     plt.colorbar(label='Value')
#     plt.scatter(blob_positions[:,1], blob_positions[:,0], marker='x', color = 'r')
#     plt.xlabel('Column')
#     plt.ylabel('Row')
#     plt.title('2D Plot of test_data')
#     plt.savefig("lidar")


st = time.process_time()

on_tick( timestamp,_all_readings_count, _lidar_message_received, _lidar_labels, _raw_lidar, _lidar_labels_prev_detections, _debris_eci_pos, _debris_sizes,_detection_times,_debris_velocities)
et = time.process_time()
res = et - st
print('CPU Execution time:', res, 'seconds')

# cProfile.run('on_tick( timestamp,_all_readings_count, _lidar_message_received, _lidar_labels, _raw_lidar, _lidar_labels_prev_detections, _debris_eci_pos, _debris_sizes,_detection_times,_debris_velocities)'
#              , filename='profile_results.txt')

# class LidarProcessing():
#     # gets test data and does all the processing
#     """A custom subsystem that listens for LiDAR measurements"""
#     def __init__(self) -> None:
#         super().__init__()
        
#         # Subscribe to the "lidar" topic
#         _lidar_sub = msg.Subscriber("raw_lidar", _lidar_subscriber)

#         # would subscribe to a reset messages which would clear all arrays

        
        
#         return

# import pstats

# # Load the profiling results from the file
# profile = pstats.Stats('profile_results.txt')

# # Print statistics to the console
# profile.print_stats()   
    

#     def _log_for_transmit(self, timestamp, x_eci, debris_size, debris_vel) -> None:
#         self._debris_eci_pos.append(x_eci)
#         self._debris_sizes.append(debris_size)
#         self._detection_times.append(timestamp)
#         self._debris_velocities.append(debris_vel)
        
#     def _lidar_subscriber(self, measurement: LidarMeasurement) -> None:
#         # Log the lidar data
#         self._log_lidar(measurement.distances, measurement.label)
#         self._lidar_message_received = True
#         return
    
#     def _log_lidar(self, distances: np.ndarray, label: int) -> None:
#         self._raw_lidar.append(distances)
#         self._lidar_labels.append(label)
#         return
    