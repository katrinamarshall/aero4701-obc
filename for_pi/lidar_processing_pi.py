import numpy as np
import matplotlib.pyplot as plt
import datetime as dt

from sat_data_simulated import *
from lidar_constants import *
from rotation import *
from _conversion import *
            
class LidarProcessing():
    """ gets test data and does all the processing"""
    def __init__(self) -> None:

        # Subscribe to the "lidar" topic
        self._lidar_sub = msg.Subscriber("raw_lidar", self._lidar_subscriber)

        # would subscribe to a reset messages which would clear all arrays

        # subscribe to "GNSS" topic
        self._gnss_sub = msg.Subscriber("rakon_gnss", self._gnss_subscriber)
        
        # subscribe to attitude topic
        self._att_sub = msg.Subscriber("attitude_w_noise", self._att_subscriber)
        
        # logging for gnss 
        self._positions = []
        self._velocities = []

        # logging for attitude
        self._attitudes = []

        # Logging fields
        self._raw_lidar = []
        self._lidar_labels = []

        # Callback variables
        self._lidar_message_received = False
        self._all_readings_count = 0

        # Storage fields for processed debris
        self._debris_count = 0
        self._lidar_labels_prev_detections = []

        self._debris_eci_pos = []
        self._debris_sizes = []
        self._debris_velocities = []
        self._detection_times = []
        self._debris_rel_velocities = []

        self._prev_detections = [] #
     
        return
    
    def _gnss_subscriber(self, measurement: msg.GNSSMeasurement) -> None:
        # Log the position
        self._log_position(measurement.position)
        self._log_velocity(measurement.velocity)
        return
    
    def _log_position(self, position: np.ndarray) -> None:
        self._positions.append(position)
        return
    def _log_velocity(self, velocity: np.ndarray) -> None:
        self._velocities.append(velocity)
        return
    
    
        
    def _att_subscriber(self, measurement: AttitudeMeasurement) -> None:
        # Log the position
        self._log_attitude(measurement.attitude)
        return
    
    def _log_attitude(self, attitude: np.ndarray) -> None:
        self._attitudes.append(attitude)
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
            tolerance = 50 # 5cm change per pixel max grad for same object
            within_range = (data[i][j] < sensor_range) and (abs(data[i][j] - current_value) < tolerance)
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

    def find_blobs(self, data, sensor_range):
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
                    max_diameter_x, max_diameter_y, avg_value, size = self.explore_blob(data, visited, i, j, sensor_range, i, j)
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

    def cosine_rule(self, a, b, theta_degrees):
        """
        finds c, given opposite angle to side in degrees
        """
        theta_radians = np.radians(theta_degrees)
        cos_theta = np.cos(theta_radians)
        c_squared = a**2 + b**2 - 2 * a * b * cos_theta
        return np.sqrt(c_squared)

    def image2polar(self, blob_positions, blob_avg_values, blob_diameters, fov, num_pixels_1D):
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
            resolution =self.cosine_rule(dist,dist,fov)/num_pixels_1D

            theta = np.radians((fov/num_pixels_1D) * x - fov/2)
            phi = np.radians((fov/num_pixels_1D) * y - fov/2)
            debris_pos[i] = [phi,theta,r]

            debris_size = blob_diameters[i]*resolution #[mm]
            sizes[i] = debris_size
            if DEBUGGING_MODE:
                # print("------------------------------------------------------------------------------------------------")
                print(f"Debris object detected max diameter {debris_size:.1f}mm ({blob_diameters[i]} pixels), at r:{dist:.3f}mm, theta:{np.degrees(theta):.2f}°, phi:{np.degrees(phi):.2f}°")
                # print("------------------------------------------------------------------------------------------------")
        return np.array(debris_pos), sizes
    
    def on_tick(self, satellite_state: msg.SatelliteState) -> None:
        """This `on_tick()` method does TODO"""
        if self._lidar_message_received == True:
            self._lidar_message_received = False
            
            # find how many new lidar readings are to be processed
            num_new_readings = len(self._lidar_labels) - self._all_readings_count
            self._all_readings_count = len(self._lidar_labels)
            # print(self._lidar_labels_prev_detections)

            # for each new lidar packet
            for n in range(num_new_readings):
                
                data = np.array(self._raw_lidar[-num_new_readings+n])

                sigma = 0.05 * data
                noisy_data = data + np.random.normal(0, sigma,size=data.shape)

                # run blob detection algorithm on new data
                blob_diameters, blob_positions, blob_avg_values = self.find_blobs(data, SENSOR_RANGE)
                blob_diameters_noisy, blob_positions_noisy, blob_avg_values_noisy = self.find_blobs(noisy_data, SENSOR_RANGE)

                # check if any debris has been found - if so print updating debris count - add as var TODO
                if len(blob_diameters) > 0:

                    # Found new debris
                    self._lidar_labels_prev_detections.append(self._lidar_labels[-num_new_readings+n])

                    # if so, find its coordinates
                    debris_pos_polar, debris_sizes = self.image2polar(blob_positions, blob_avg_values,blob_diameters, FOV, PIXELS_1D)
                    if CHECK_RESIDUALS == True:
                        debris_pos_polar_noisy, debris_sizes_noisy = self.image2polar(blob_positions_noisy, blob_avg_values_noisy,blob_diameters_noisy, FOV, PIXELS_1D)

                    plotted = False
                    if PLOT == True and plotted == False:
                        self.plot_lidar_data(data,blob_positions)
                        plotted = True
                    # update debris count
                    self._debris_count += len(blob_avg_values)
                    #TODO edit this print statement
                
                    # get gnss data
                    r_sat = satellite_state.position
                    v_sat = satellite_state.velocity
                    
                    # get attitude data
                    attitude = satellite_state.attitude

                    r_sat_gnss = self._positions[-1]
                    v_sat_gnss = self._velocities[-1]
                    att_noise = self._attitudes[-1]
                    # r_sat_gnss = r_sat
                    # v_sat_gnss = v_sat
                    # att_noise = attitude

                    if DEBUGGING_MODE:
                        print(f"----- SAT POS: {r_sat} ECI coords -------------")
                        print(f"VEL: {v_sat}, ATT: {attitude}")
                        

                    s = 0 # counter
                    # For each debris object set of coordinates
                    for x_polar in debris_pos_polar:
                        # identify which lidar this came from
                        lidar_label = self._lidar_labels_prev_detections[n]
                        # convert to xyz coordinates
                        debris_pos_cart = _conversion.polar_to_cartesian(x_polar)
                
                        # convert these to body fixed based on lidar positioning
                        # in body-fixed frame y points straight down
                        debris_pos_body_fixed = rotation.rot3_y((lidar_label-1)*np.pi/2)@debris_pos_cart

                        # convert body fixed coordinates to ECI
                        debris_pos_eci_rel = _conversion.bodyfixed_to_ECI(debris_pos_body_fixed,r_sat,v_sat,attitude,rotation_only=True)
                        debris_pos_eci = debris_pos_eci_rel + r_sat
       

                    # TODO write exception handling for if there are multiple debris objects in one frame 
                    # at new timestep, check previous timestep to find the velocity of debris object
                    #TODO make very basic for just one object in frame, if there in next time step
                        timestamp = satellite_state.timestamp

                        if len(blob_avg_values) > 1:
                            print("Cannot find speed, more than one object in frame")
                            vel = VEL_UNKNOWN
                        else:
                            vel = self.find_abs_vel(lidar_label, debris_pos_eci, debris_sizes[s], timestamp)
                            
                        # store which lidar most recent detection from
                        self._prev_detections.append(lidar_label)
                        self._log_for_transmit(timestamp, debris_pos_eci, debris_sizes[s], vel,v_sat)

                       
                        # Log detected debris
                        print("-----------------------------------------------------------------------------------------------")
                        print("------------------------------ TRANSMIT DATA --------------------------------------------------")
                        print(f"From LiDAR {lidar_label}: {satellite_state.timestamp}: ")
                        print(f"DEBRIS FOUND with max diam {debris_sizes[s]}mm at {debris_pos_eci} ECI ")
                        if sum(self._debris_velocities[-1] - VEL_UNKNOWN) != 0:
                            print(f"Travelling at absolute velocity in ECI frame {self._debris_velocities[-1]} m/s")
                            print(f"Relative to DEBRA speed {np.linalg.norm(self._debris_rel_velocities[-1])} m/s, ")
                            print(f" {self._debris_rel_velocities[-1]} m/s")
                            print(f"v satellite {v_sat}")
                    
                        s += 1

                        print("-----------------------------------------------------------------------------------------------")
                        print("\n")

        return
    
    def find_abs_vel(self, lidar_label,debris_pos_eci, size, timestamp):
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
    
   
    def plot_lidar_data(self, test_data, blob_positions):
        plt.figure()
        plt.imshow(test_data, cmap='viridis', interpolation= 'nearest')
        plt.colorbar(label='Value')
        plt.scatter(blob_positions[:,1], blob_positions[:,0], marker='x', color = 'r')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('2D Plot of test_data')
        plt.savefig("lidar")


    def _log_for_transmit(self, timestamp, x_eci, debris_size, debris_vel, v_sat) -> None:
        self._debris_eci_pos.append(x_eci)
        self._debris_sizes.append(debris_size)
        self._detection_times.append(timestamp)
        self._debris_velocities.append(debris_vel)
        self._debris_rel_velocities.append(debris_vel - v_sat)

    
    def _lidar_subscriber(self, measurement: LidarMeasurement) -> None:
        # Log the lidar data
        self._log_lidar(measurement.distances, measurement.label)
        self._lidar_message_received = True
        return
    
    def _log_lidar(self, distances: np.ndarray, label: int) -> None:
        self._raw_lidar.append(distances)
        self._lidar_labels.append(label)
        return
    
    

def main() -> None:
    # DEBRA orbit elements
    a = 7212100
    e = 0
    i = np.radians(53.055)
    RAAN = np.radians(38)
    omega = np.radians(0)
    theta = np.radians(0)
    
    elements = (a, e, i, RAAN, omega, theta)
    
    # ---------- Initialise sensors and subsystems
    gnss = RakonGNSSReceiver(
        15,
        position_noise=np.array([1,1,1]),         # 1m position noise on each axis
        velocity_noise=np.array([0.1, 0.1,0.1])    # 0.1m/s velocity noise on each axis
    )

    
    gnss_listener = GNSSListener()

    attitude_sensor = DeterminedAttitude(
        rate=15,
        attitude_noise=0 # degrees on roll,pitch,yaw
    )

    lidar1 = Lidar(
        LIDAR_FREQ, 
        num_pixels_side=PIXELS_1D,
        debris_period_s= 1/LIDAR_FREQ,
        label=1
    )
    lidar2 = Lidar(
        LIDAR_FREQ,
        num_pixels_side=PIXELS_1D,
        debris_period_s= 3/LIDAR_FREQ,
        label=2
    )
    lidar3 = Lidar(
        LIDAR_FREQ,
        num_pixels_side=PIXELS_1D,
        debris_period_s= 2/LIDAR_FREQ,
        label=3
    )
    lidar4 = Lidar(
        LIDAR_FREQ,
        num_pixels_side=PIXELS_1D,
        debris_period_s=2/LIDAR_FREQ,
        label=4
    )

    lidar_listener = LidarProcessing()
    # ---------- Build the simulation
    sim_builder = pss.SimulatorBuilder()
    
    # Elements or initial velocity and position can be used
    sim_builder.add_satellite(elements)
    sim_builder.add_sensor(gnss)
    sim_builder.add_subsystem(gnss_listener)
    sim_builder.add_sensor(attitude_sensor)

    sim_builder.add_sensor(lidar1)
    sim_builder.add_sensor(lidar2)
    sim_builder.add_sensor(lidar3)
    sim_builder.add_sensor(lidar4)

    sim_builder.add_subsystem(lidar_listener)
    
    sim_builder.set_start_time(dt.datetime.now())
    sim_builder.set_end_time(600)       # Can be a datetime object
    
    # ---------- Run the simulation
    simulator = sim_builder.get_instance()
    simulator.run()
    
    # ---------- Plot the GNSS receiver positions that were logged
    # positions = gnss_listener.positions
    # x = positions[:, 0]
    # y = positions[:, 1]
    # z = positions[:, 2]
    # velocities = gnss_listener.velocities
    # vx = velocities[:, 0]
    # vy = velocities[:, 1]
    # vz = velocities[:, 2]


    # print("Positions:", x,y,z)
    # print("Norm pos", np.linalg.norm([x[0],y[0],z[0]]))
    # print("Velocities:", vx,vy,vz)
    # print("Norm vel", np.linalg.norm([vx[0],vy[0],vz[0]]))
    
    # Plot the positions
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    
    # ax.plot(x, y, z)
    # ax.set_xlabel("X (m)")
    # ax.set_ylabel("Y (m)")
    # ax.set_zlabel("Z (m)")
    
    # plt.show()

    true_eci = np.array(lidar_listener._debris_eci_pos)
    eci_error = np.array(lidar_listener._debris_eci_pos_error)

    true_vel = np.array(lidar_listener._debris_velocities)
    vel_error = np.array(lidar_listener._debris_velocities_error)

    true_rel_vel = np.array(lidar_listener._debris_rel_velocities)
    rel_vel_error = np.array(lidar_listener._debris_rel_velocities_error)


    # plt.figure()
    # plt.plot(range(len(true_eci)), true_eci[0] - eci_error[1])
    # # plt.legend('x','y','z')
    # plt.show()

    percent_error_eci = 100*(eci_error - true_eci)/true_eci
    percent_error_vel = 100*(vel_error - true_vel)/true_vel
    percent_error_rel_vel = 100*(rel_vel_error - true_rel_vel)/true_rel_vel

    fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # Plot data on each subplot
    axs[0].plot(range(len(true_eci)), eci_error[:,0]- true_eci[:,0], color='blue')
    # axs[0].plot(range(len(true_eci)), true_eci[:,0], color= 'red')
    axs[0].set_title('Debris Position X ')
    axs[0].set_xlabel('Timestep')
    axs[0].set_ylabel('Residual (m)')

    axs[1].plot(range(len(true_eci)), eci_error[:,1]- true_eci[:,1],  color='blue')
    # axs[1].plot(range(len(true_eci)), true_eci[:,1], color= 'red')
    axs[1].set_title('Debris Position Y')
    axs[1].set_xlabel('Timestep')
    axs[1].set_ylabel('Residual (m)')

    axs[2].plot(range(len(true_eci)), eci_error[:,2]- true_eci[:,2],  color='blue')
    # axs[2].plot(range(len(true_eci)), true_eci[:,2], color= 'red')
    axs[2].set_title('Debris Position Z')
    axs[2].set_xlabel('Timestep')
    axs[2].set_ylabel('Residual (m)')

    plt.tight_layout()
    plt.savefig('residual_pos1')


    fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # Plot data on each subplot
    axs[0].plot(range(len(true_eci)), eci_error[:,0], color='blue')
    axs[0].plot(range(len(true_eci)), true_eci[:,0], color= 'red')
    axs[0].set_title('Debris Position X ')
    axs[0].set_xlabel('Timestep')
    axs[0].set_ylabel('ECI Position(m)')

    axs[1].plot(range(len(true_eci)), eci_error[:,1],  color='blue')
    axs[1].plot(range(len(true_eci)), true_eci[:,1], color= 'red')
    axs[1].set_title('Debris Position Y')
    axs[1].set_xlabel('Timestep')
    axs[1].set_ylabel('ECI Position(m)')

    axs[2].plot(range(len(true_eci)), eci_error[:,2],  color='blue')
    axs[2].plot(range(len(true_eci)), true_eci[:,2], color= 'red')
    axs[2].set_title('Debris Position Z')
    axs[2].set_xlabel('Timestep')
    axs[2].set_ylabel('ECI Position(m)')

    plt.tight_layout()
    plt.savefig("tracking_eci")


    fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # Plot data on each subplot
    axs[0].plot(range(len(true_vel)), vel_error[:,0], color='blue')
    axs[0].plot(range(len(true_eci)), true_vel[:,0], color= 'red')
    axs[0].set_title('Debris Velocity X ')
    axs[0].set_xlabel('Timestep')
    axs[0].set_ylabel('ECI Velocity (m/s)')

    axs[1].plot(range(len(true_eci)), vel_error[:,1],  color='blue')
    axs[1].plot(range(len(true_eci)), true_vel[:,1], color= 'red')
    axs[1].set_title('Debris Velocity Y')
    axs[1].set_xlabel('Timestep')
    axs[1].set_ylabel('ECI Velocity (m/s)')

    axs[2].plot(range(len(true_eci)), vel_error[:,2],  color='blue')
    axs[2].plot(range(len(true_eci)), true_vel[:,2], color= 'red')
    axs[2].set_title('Debris Velocity Z')
    axs[2].set_xlabel('Timestep')
    axs[2].set_ylabel('ECI Velocity (m/s)')

    plt.tight_layout()
    plt.savefig("tracking_vel")

    fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # Plot data on each subplot
    axs[0].plot(range(len(true_vel)), rel_vel_error[:,0], color='blue')
    axs[0].plot(range(len(true_eci)), true_rel_vel[:,0], color= 'red')
    axs[0].set_title('Debris Velocity X ')
    axs[0].set_xlabel('Timestep')
    axs[0].set_ylabel('ECI Velocity (m/s)')

    axs[1].plot(range(len(true_eci)), rel_vel_error[:,1],  color='blue')
    axs[1].plot(range(len(true_eci)), true_rel_vel[:,1], color= 'red')
    axs[1].set_title('Debris Velocity Y')
    axs[1].set_xlabel('Timestep')
    axs[1].set_ylabel('ECI Velocity (m/s)')

    axs[2].plot(range(len(true_eci)), rel_vel_error[:,2],  color='blue')
    axs[2].plot(range(len(true_eci)), true_rel_vel[:,2], color= 'red')
    axs[2].set_title('Debris Velocity Z')
    axs[2].set_xlabel('Timestep')
    axs[2].set_ylabel('ECI Velocity (m/s)')

    plt.tight_layout()
    plt.savefig("tracking_vel_rel")

    fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # Plot data on each subplot
    axs[0].plot(range(len(true_vel)), vel_error[:,0] - true_vel[:,0], color='blue')
    # axs[0].plot(range(len(true_vel)), vel_error[:,0], color='red')
    axs[0].set_title('Debris Velocity X ')
    axs[0].set_xlabel('Timestep')
    axs[0].set_ylabel('Residual (m/s)')

    axs[1].plot(range(len(true_vel)), vel_error[:,1] - true_vel[:,1],color='blue')
    # axs[1].plot(range(len(true_vel)), vel_error[:,1], color='red')
    axs[1].set_title('Debris Velocity Y')
    axs[1].set_xlabel('Timestep')
    axs[1].set_ylabel('Residual (m/s)')

    axs[2].plot(range(len(true_vel)), vel_error[:,2] - true_vel[:,2], color='blue')
    # axs[2].plot(range(len(true_vel)), vel_error[:,2], color='red')
    axs[2].set_title('Debris Velocity Z')
    axs[2].set_xlabel('Timestep')
    axs[2].set_ylabel('Residual (m/s)')

    plt.tight_layout()
    plt.savefig("vel_residual1")

    fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # Plot data on each subplot
    axs[0].plot(range(len(true_vel)), percent_error_vel[:,0], color='blue')
    # axs[0].plot(range(len(true_vel)), vel_error[:,0], color='red')
    axs[0].set_title('Debris Velocity X ')
    axs[0].set_xlabel('Timestep')
    axs[0].set_ylabel('Residual (%)')

    axs[1].plot(range(len(true_vel)), percent_error_vel[:,1],color='blue')
    # axs[1].plot(range(len(true_vel)), vel_error[:,1], color='red')
    axs[1].set_title('Debris Velocity Y')
    axs[1].set_xlabel('Timestep')
    axs[1].set_ylabel('Residual (%)')

    axs[2].plot(range(len(true_vel)), percent_error_vel[:,2], color='blue')
    # axs[2].plot(range(len(true_vel)), vel_error[:,2], color='red')
    axs[2].set_title('Debris Velocity Z')
    axs[2].set_xlabel('Timestep')
    axs[2].set_ylabel('Residual (%)')

    plt.tight_layout()
    plt.savefig("percent_vel1")
    
    fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # Plot data on each subplot
    axs[0].plot(range(len(true_vel)), rel_vel_error[:,0] - true_rel_vel[:,0], color='blue')
    # axs[0].plot(range(len(true_vel)), vel_error[:,0], color='red')
    axs[0].set_title('Debris Relative Velocity X ')
    axs[0].set_xlabel('Timestep')
    axs[0].set_ylabel('Residual (m/s)')

    axs[1].plot(range(len(true_vel)), rel_vel_error[:,1] - true_rel_vel[:,1],color='blue')
    # axs[1].plot(range(len(true_vel)), vel_error[:,1], color='red')
    axs[1].set_title('Debris Relative Velocity Y')
    axs[1].set_xlabel('Timestep')
    axs[1].set_ylabel('Residual (m/s)')

    axs[2].plot(range(len(true_vel)), rel_vel_error[:,2] - true_rel_vel[:,2], color='blue')
    # axs[2].plot(range(len(true_vel)), vel_error[:,2], color='red')
    axs[2].set_title('Debris Relative Velocity Z')
    axs[2].set_xlabel('Timestep')
    axs[2].set_ylabel('Residual (m/s)')

    plt.tight_layout()
    plt.savefig("rel_vel1")

    fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    # Plot data on each subplot
    axs[0].plot(range(len(true_vel)), percent_error_rel_vel[:,0], color='blue')
    # axs[0].plot(range(len(true_vel)), vel_error[:,0], color='red')
    axs[0].set_title('Debris Velocity X ')
    axs[0].set_xlabel('Timestep')
    axs[0].set_ylabel('Residual (%)')

    axs[1].plot(range(len(true_vel)), percent_error_rel_vel[:,1],color='blue')
    # axs[1].plot(range(len(true_vel)), vel_error[:,1], color='red')
    axs[1].set_title('Debris Velocity Y')
    axs[1].set_xlabel('Timestep')
    axs[1].set_ylabel('Residual (%)')

    axs[2].plot(range(len(true_vel)), percent_error_rel_vel[:,2], color='blue')
    # axs[2].plot(range(len(true_vel)), vel_error[:,2], color='red')
    axs[2].set_title('Debris Velocity Z')
    axs[2].set_xlabel('Timestep')
    axs[2].set_ylabel('Residual (%)')

    plt.tight_layout()
    plt.savefig("rel_vel_percent1")

    debris_sizes = np.array(lidar_listener._debris_sizes)
    sizes_error = np.array(lidar_listener._debris_sizes_error)

    residual_sizes = sizes_error - debris_sizes

    fig, axs = plt.subplots(1, 1, figsize=(8, 6))

    # Plot data on each subplot
    axs.plot(range(len(true_vel)), 100*(sizes_error - debris_sizes)/debris_sizes, color='blue')
    # axs[0].plot(range(len(true_vel)), vel_error[:,0], color='red')
    axs.set_title('Debris Sizes Residual')
    axs.set_xlabel('Timestep')
    axs.set_ylabel('Residual (%)')

    plt.tight_layout()
    plt.savefig("sizes")

    




    return








if __name__ == "__main__":
    main()