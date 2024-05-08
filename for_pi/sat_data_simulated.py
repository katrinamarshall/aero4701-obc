import pysatsim as pss
import pysatsim.messages as msg
from pysatsim.subsystem import Subsystem, Sensor
from pysatsim.coordinates import rotation, _conversion





# Message types use the `@message` decorator.
# Some common message types are already defined in `pysatsim.messages`
@msg.message
class LidarMeasurement():
    """LiDAR Measurement message.
    
    Attributes:

    """
    distances: np.ndarray
    label: int
    range_sigma_mm: int # a percentage
    # reflectance: np.ndarray 
    # target_status: np.ndarray

@msg.message
class AttitudeMeasurement():
    """
    
    """
    attitude: np.ndarray
    timestamp: dt.datetime

# @msg.message
# class ProcessedPayloadData():
#     """Data processed from payload message. Ready for transmit to ground
#     """
#     timestep: dt.datetime
#     debris_count: int
#     debris_ecis: np.ndarray
#     debris_sizes: np.ndarray
#     # debris_speeds: np.ndarray


class RakonGNSSReceiver(Sensor):
    """A custom sensor class for a Rakon GNSS receiver"""
    
    # Sensors must have a sampling rate
    def __init__(
        self,
        rate: float,
        position_noise: np.ndarray = np.zeros(3),
        velocity_noise: np.ndarray = np.zeros(3)
    ) -> None:
        """Initialises a Rakon GNSS receiver sensor

        Args:
            rate (float): The sampling rate of the sensor in hertz
            position_noise (np.ndarray, optional): Stddev of position noise in metres.
            velocity_noise (np.ndarray, optional): Stddev of velocity noise in m/s
        """
        super().__init__(rate)
        
        self._r_noise = position_noise
        self._v_noise = velocity_noise
        
        # Typically, you want to publish a sensors measurement
        # "rakon_gnss" is the topic name
        self._publisher = msg.Publisher("rakon_gnss")
        
        return
    
    # All sensors must implement the `on_tick` method. For sensors,
    # this method is called by the simulation manager when a measurement
    # is due (specified by the rate) e.g. if the rate is 1 Hz, `on_tick` 
    # is called once every second
    def on_tick(self, satellite_state: msg.SatelliteState) -> None:
        # GNSS measurement is noise added to the true position and velocity
        true_position = satellite_state.position
        true_velocity = satellite_state.velocity
        
        gnss_position = true_position + np.random.normal(0, self._r_noise)
        gnss_velocity = true_velocity + np.random.normal(0, self._v_noise)
        
        # Create a message object
        measurement = msg.GNSSMeasurement(
            timestamp=satellite_state.timestamp,
            position=gnss_position,
            velocity=gnss_velocity
        )
        
        # Publish the message
        self._publisher.publish(measurement)
        return 


class DeterminedAttitude(Sensor):
    """A custom sensor class to mimic noise from attitude determination"""
    
    # Sensors must have a sampling rate
    def __init__(
        self,
        rate: float,
        attitude_noise: np.ndarray = np.zeros(3)
        ) -> None:
        """Initialises a Rakon GNSS receiver sensor

        Args:
            rate (float): The sampling rate of the sensor in hertz
            attitude_noise (np.ndarray, optional): Stddev of attitude noise in degrees.
        """
        super().__init__(rate)
        
        self._att_noise = attitude_noise
        
        # Typically, you want to publish a sensors measurement
        # "rakon_gnss" is the topic name
        self._publisher = msg.Publisher("attitude_w_noise")
        
        return
    
    # All sensors must implement the `on_tick` method. For sensors,
    # this method is called by the simulation manager when a measurement
    # is due (specified by the rate) e.g. if the rate is 1 Hz, `on_tick` 
    # is called once every second
    def on_tick(self, satellite_state: msg.SatelliteState) -> None:
        # GNSS measurement is noise added to the true position and velocity
        true_attitude = satellite_state.attitude

        # convert from quaternion to euler to add noise
        att_euler = np.array(rotation.ryp_from_quart(true_attitude))
        att_w_noise_euler = att_euler + np.random.normal(0, self._att_noise, size=3)

        # Ensure that yaw, pitch, and roll angles are within valid range
        # att_w_noise_euler[0] = np.mod(att_w_noise_euler[0], 360)  # Yaw (mod 360 to keep within 0-360 range)
        # att_w_noise_euler[1] = np.clip(att_w_noise_euler[1], -90, 90)  # Pitch (clipped to -90 to 90 degrees range)
        # att_w_noise_euler[2] = np.mod(att_w_noise_euler[2], 360)  # Roll (mod 360 to keep within 0-360 range)

        att_w_noise = rotation.quart_from_ryp(att_w_noise_euler[0],att_w_noise_euler[1],att_w_noise_euler[2])
        
        # Create a message object
        measurement = AttitudeMeasurement(
            attitude=att_w_noise,
            timestamp=satellite_state.timestamp,
        )
        
        # Publish the message
        self._publisher.publish(measurement)
        return 


class Lidar(Sensor):
    """A custom sensor class for a LiDAR sensor"""
    # Sensors must have a sampling rate
    def __init__(
        self,
        rate: float,
        num_pixels_side: int,
        debris_period_s: float,
        label: int # number 0 1 2 3
        # lidar_noise: np.ndarray = np.zeros(64)
    ) -> None:
        """Initialises a TODO

        Args:
            rate (float): The sampling rate of the sensor in hertz
            position_noise (np.ndarray, optional): Stddev of position noise in metres.
            velocity_noise (np.ndarray, optional): Stddev of velocity noise in m/s
        """
        super().__init__(rate)
        
        self._num_pixels = num_pixels_side
        self._label = label
        self._debris_period = debris_period_s

        # self._v_noise = velocity_noise
        
        # Typically, you want to publish a sensors measurement
        # "rakon_gnss" is the topic name
        self._publisher = msg.Publisher("raw_lidar")
        
        return
    

    # TODO make this data change with time - get it to show up every max debris frequency
    def gen_test_data(self, current_time, test_vel=False, empty = False):
        eps = 0.01
        if test_vel == True:
            # if testing velocity we just want one cluster of debris
            test_data = np.full((self._num_pixels, self._num_pixels), 10000)
            # if current time is multiple of debris frequency period, then show piece of debris
            if current_time % self._debris_period < eps:
                cluster_size1 = np.random.randint(100)
                # clump1_center = np.random.randint(0, 1024 - cluster_size1, size=2)
                clump1_center = np.random.randint(0, (self._num_pixels - cluster_size1), size=2)
                test_data[clump1_center[0]:clump1_center[0] + cluster_size1, clump1_center[1]:clump1_center[1] + cluster_size1] = 100
                # plt.imshow(test_data, cmap='viridis', interpolation= 'nearest')
                # plt.colorbar(label='Value')
                # # plt.scatter(blob_positions[:,1], blob_positions[:,0], marker='x', color = 'r')
                # plt.xlabel('Column')
                # plt.ylabel('Row')
                # plt.title('2D Plot of test_data')
                # plt.show()
        else:
            if empty == True: #current_time % self._debris_period < eps or
                test_data = np.full((self._num_pixels, self._num_pixels), SENSOR_RANGE)
            elif self._num_pixels == 8: 
                # 8 x 8 input data
                    test_data = np.array([[40, 30, 50, 4000, 100, 140, 4000, 4000], 
                                    [10, 10, 1000, 4000, 4000, 4000, 4000, 4000],
                                    [10, 4000, 1000, 1000, 1000, 4000, 4000, 4000],
                                    [1000, 1040, 1000, 4000, 4000, 4000, 4000, 4000],
                                    [4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000],
                                    [4000, 4000, 4000, 4000, 4000, 1500, 1540, 1580], 
                                    [4000, 2000, 4000, 4000, 4000, 4000, 4000, 4000], 
                                    [4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000]])  
                                                                                                                                
            else:
                # test_data = np.random.uniform(0.1, 4000, size=(64, 64))
                # test_data_8 = np.array([[0.1, 0.2, 4000, 4000], [0.1, 0.1, 4000, 4000], [4000, 4000, 4000, 4000], [4000, 4000, 4000, 4000]])
                # Create a 1024x1024 array filled with 4000 
                test_data = np.full((self._num_pixels, self._num_pixels), SENSOR_RANGE)
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

    def on_tick(self, satellite_state: msg.SatelliteState) -> None:
    
        # Generate lidar data
        
        raw_lidar_data = self.gen_test_data(satellite_state.simulation_time, test_vel=TEST_VEL)

        # Create a message object
        measurement = LidarMeasurement(
            distances=raw_lidar_data,
            label = self._label,
            range_sigma_mm=0.05
        )
        
        # Publish the message
        self._publisher.publish(measurement)
        return 



    

class GNSSListener(Subsystem):
    """A custom subsystem that listens for GNSS measurements"""
    
    def __init__(self) -> None:
        super().__init__()
        
        # Subscribe to the "rakon_gnss" topic
        self._gnss_sub = msg.Subscriber("rakon_gnss", self._gnss_subscriber)
        
        # Logging fields
        self._positions = []
        self._velocities = []
        return
    
    # For Subsystems, the `on_tick()` method is called at every time step
    # in the simulation. You may not want to do anything at every time step
    # You might only want to do something after a message is recieved or
    # after a certain amount of time has passed.
    def on_tick(self, satellite_state: msg.SatelliteState) -> None:
        # """This `on_tick()` method does do anything"""
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
    
    @property
    def positions(self) -> np.ndarray:
        """The logged positions of the GNSS receiver"""
        return np.array(self._positions)
    
    @property
    def velocities(self) -> np.ndarray:
        """The logged velocities of the GNSS receiver"""
        return np.array(self._velocities)
    
    