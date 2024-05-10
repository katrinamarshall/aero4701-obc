"""Conversions between different coordinate systems.

Contains:
---------
    - elements_to_perifocal
    - perifocal_to_ECI_matrix
    - perifocal_to_ECI
    - elements_to_ECI
    - ECI_to_LVLH
    - LVLH_to_ECI
    - ECI_to_bodyfixed
    - bodyfixed_to_ECI
    - ECI_to_elements
    - ECEF_to_ECI
    - ECI_to_ECEF
    - ECEF_to_LLH
    - LLH_to_ECEF
    - LLH_to_ECI
    - ECI_to_LLH
    - geodetic_radius
    - geo_ecliptic_to_ECI
    - NED_to_ECEF
    - NED_to_ECI
    - NED_to_LVLH
    - polar_to_cartesian
    - cartesian_to_polar
"""

import numpy as np
import math
import datetime as dt
import constants as const
import time
import rotation as rot


def elements_to_perifocal(
    a: float,
    e: float,
    mean_anomaly: float
) -> tuple[np.ndarray, np.ndarray]:
    """Converts the classical orbital elements of an orbit
    into position and velocity vectors in the perifocal frame.

    Args:
        a (float): The semimajor axis of the orbit.
        e (float): The eccentricity of the orbit.
        mean (float): The true anomaly in radians.

    Returns:
        tuple[np.ndarray, np.ndarray]: The position and velocity
        vectors in the perifocal frame as numpy arrays.
    """
    h = np.sqrt(const.MU_EARTH * a * (1 - e**2))                        # Specific angular momentum
    r = (h**2 / const.MU_EARTH) * (1 / (1 + e * np.cos(mean_anomaly)))  # Radius
    v_p = const.MU_EARTH / h                                            # Perifocal velocity

    r_perifocal = r * np.array([
        np.cos(mean_anomaly),
        np.sin(mean_anomaly),
        0
    ], dtype=np.float64)
    
    v_perifocal = v_p * np.array([
        -np.sin(mean_anomaly),
        e + np.cos(mean_anomaly),
        0
    ], dtype=np.float64)
    
    return r_perifocal, v_perifocal


def perifocal_to_ECI_matrix(
    i: float,
    rt_asc: float,
    arg_p: float
) -> np.ndarray:
    """Creates a matrix to convert a vector in the perifocal 
    frame to the ECI frame.

    Args:
        i (float): Inclination of the orbit in radians.
        rt_asc (float): Right ascension of the ascending node radians.
        arg_p (float): Argument of perigee in radians.

    Returns:
        np.ndarray: A (3 x 3) matrix to convert a vector in the 
        perifocal frame to the ECI frame.
    """
    # *_r denotes radians    
    M11 = -np.sin(rt_asc) * np.cos(i) * np.sin(arg_p) + np.cos(rt_asc) * np.cos(arg_p)
    M12 = -np.sin(rt_asc) * np.cos(i) * np.cos(arg_p) - np.cos(rt_asc) * np.sin(arg_p)
    M13 = np.sin(rt_asc) * np.sin(i)
    
    M21 = np.cos(rt_asc) * np.cos(i) * np.sin(arg_p) + np.sin(rt_asc) * np.cos(arg_p)
    M22 = np.cos(rt_asc) * np.cos(i) * np.cos(arg_p) - np.sin(rt_asc) * np.sin(arg_p)
    M23 = -np.cos(rt_asc) * np.sin(i)
    
    M31 = np.sin(i) * np.sin(arg_p)
    M32 = np.sin(i) * np.cos(arg_p)
    M33 = np.cos(i)
    
    return np.array([
        [M11, M12, M13],
        [M21, M22, M23],
        [M31, M32, M33]
    ], dtype=np.float64)


def perifocal_to_ECI(
    x_perifocal: np.ndarray,
    i: float,
    rt_asc: float,
    arg_p: float
) -> np.ndarray:
    """Converts a vector in the perifocal frame to the ECI frame.

    Args:
        x_perifocal (np.ndarray): A vector in the perifocal frame.
        i (float): Inclination of the orbit in radians.
        rt_asc (float): Right ascension of the ascending node in radians.
        arg_p (float): Argument of perigee in radians.

    Returns:
        np.ndarray: A vector in the ECI frame.
    """    
    return perifocal_to_ECI_matrix(i, rt_asc, arg_p) @ x_perifocal


def elements_to_ECI(
    a: float,
    e: float,
    i: float,
    rt_asc: float,
    arg_p: float,
    theta: float
) -> tuple[np.ndarray, np.ndarray]:
    """Converts the classical orbital elements of an orbit
    into position and velocity vectors in the ECI frame.

    Args:
        a (float): The semimajor axis of the orbit.
        e (float): The eccentricity of the orbit.
        i (float): Inclination of the orbit in radians.
        rt_asc (float): Right ascension of the ascending
        node in radians.
        arg_p (float): Argument of perigee in radians.
        theta (float): The true anomaly in radians.

    Returns:
        tuple[np.ndarray, np.ndarray]: The position and velocity
        vectors in the ECI frame.
    """
    r_p, v_p = elements_to_perifocal(a, e, theta)
    r_ECI = perifocal_to_ECI(r_p, i, rt_asc, arg_p)
    v_ECI = perifocal_to_ECI(v_p, i, rt_asc, arg_p)
    
    return r_ECI, v_ECI

def ECI_to_elements(r: np.ndarray, v: np.ndarray, mu: float) -> np.ndarray:
    """Finds the orbital elements of an orbit given the position and velocity
    vectors in the ECI frame.
    
    All units are SI or radians.

    Args:
        r (np.ndarray): The position vector in the ECI frame.
        v (np.ndarray): The velocity vector in the ECI frame.
        mu (float): The gravitational parameter of the central body.

    Returns:
        np.ndarray: The orbital elements as a (6 x 1) vector.
        In the following order:
            semimajor axis,\n
            eccentricity,\n
            inclination,\n
            right ascension of the ascending node,\n
            argument of perigee,\n
            true anomaly.
    """
    
    r_mag = np.linalg.norm(r)
    r_hat = r / r_mag
    
    h = np.cross(r, v)                          # Specific angular momentum
    h_mag = math.sqrt(h.dot(h))
    
    e_vec = np.cross(v, h) / mu - r_hat
    e = np.linalg.norm(e_vec)                   # eccentricity
    
    k_hat = np.array([0,0,1], dtype=np.float64)
    N = np.cross(k_hat, h)                      # Node line
    N_mag = np.linalg.norm(N)
    
    a = h_mag**2 / (mu * (1 - e**2))            # semimajor axis
    
    RAAN = math.acos(N[0] / N_mag)              # Right accension
    RAAN = RAAN if N[1] >= 0 else 2 * np.pi - RAAN
    
    omega = math.acos(N.dot(e_vec) / (N_mag * e))         # Argument of perigee
    omega = omega if e_vec[2] >= 0 else 2 * np.pi - omega
    
    inclination = math.acos(h[2] / h_mag)                 # Inclination
    
    e_hat = e_vec / e
    theta = math.acos(e_hat.dot(r_hat))                   # True anomaly
    theta = theta if r.dot(v) >= 0 else 2 * np.pi - theta
    
    return np.array([
        a, e, inclination, RAAN, omega, theta
    ])
    
    
def ECI_to_ECEF(x_eci: np.ndarray, utc: dt.datetime) -> np.ndarray:
    """Converts a vector in the ECI frame to the ECEF frame.

    Args:
        x_eci (np.ndarray): A vector in the ECI frame.
        utc (dt.datetime): The current UTC time.

    Returns:
        np.ndarray: A vector in the ECEF frame.
    """
    ERA = np.deg2rad(time.sidereal_time(utc))
    rot_mat = rot.rot3_z(ERA).T
    return rot_mat @ x_eci


def ECI_to_LLH(eci: np.ndarray, utc: dt.datetime) -> np.ndarray:
    """Converts a vector in the ECI frame to the geodetic LLH frame.

    Args:
        eci (np.ndarray): A vector in the ECI frame.
        utc (dt.datetime): The current UTC time.

    Returns:
        np.ndarray: A vector containing latitude (rad), longitude (rad) and height.
    """
    ecef = ECI_to_ECEF(eci, utc)
    return ECEF_to_LLH(ecef)


def LLH_to_ECI(llh: np.ndarray, utc: dt.datetime) -> np.ndarray:
    """Converts a vector in the geodetic LLH frame to the ECI frame.

    Args:
        llh (np.ndarray): A vector containing latitude (rad), longitude (rad) and height.
        utc (dt.datetime): The current UTC time.

    Returns:
        np.ndarray: A vector in the ECI frame.
    """
    ecef = LLH_to_ECEF(llh)
    return ECEF_to_ECI(ecef, utc)


def ECEF_to_ECI(ecef: np.ndarray, utc: dt.datetime) -> np.ndarray:
    """Converts a vector in the ECEF frame to the ECI frame.

    Args:
        ecef (np.ndarray): A vector in the ECEF frame.
        utc (dt.datetime): The current UTC time.

    Returns:
        np.ndarray: A vector in the ECI frame.
    """
    ERA = np.deg2rad(time.sidereal_time(utc))
    rot_mat = rot.rot3_z(ERA)
    return rot_mat @ ecef


def ECEF_to_LLH(ecef: np.ndarray) -> np.ndarray:
    """Applies Ferrari's solution to convert a coordinate in the ECEF frame to the
    geodetic LLH frame.
    
    Source: https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#The_application_of_Ferrari's_solution

    Args:
        eci (np.ndarray): A  vector in the ECI frame.

    Returns:
        np.ndarray: A vector containing latitude (rad), longitude (rad) and height.
    """   
    x,y,z = ecef
    
    a = const.R_EQAT_EARTH
    b = const.R_POLE_EARTH
    e_2 = (a**2 - b**2) / a**2
    e_2_prime = (a**2 - b**2) / b**2
    p = np.sqrt(x**2 + y**2)
    F = 54 * b**2 * z**2
    G = p**2 + (1 - e_2) * z**2 - e_2 * (a**2 - b**2)
    c = (e_2**2 * F * p**2) / (G**3)
    s = (1 + c + np.sqrt(c**2 + 2 * c))**(1/3)
    k = s + 1 / s + 1
    P = F / (3 * k**2 * G**2)
    Q = np.sqrt(1 + 2 * e_2**2 * P)
    r_0 = -(P * e_2 * p) / (1 + Q) + np.sqrt(0.5 * a**2 * (1 + 1 / Q) - P * (1 - e_2) * z**2 / (Q * (1 + Q)) - 0.5 * P * p**2)
    U = np.sqrt((p - e_2 * r_0)**2 + z**2)
    V = np.sqrt((p - e_2 * r_0)**2 + (1 - e_2) * z**2)
    z_0 = (b**2 * z) / (a * V)
    
    # Final values
    h = U * (1 - b**2 / (a * V))
    latitude = np.arctan((z + e_2_prime * z_0) / p)
    longitude = np.arctan2(y, x)
    
    return np.array([latitude, longitude, h], dtype=np.float64)


def LLH_to_ECEF(llh: np.ndarray) -> np.ndarray:
    """Converts a vector in the geodedic LLH frame to the ECEF frame.
    
    Args:
        llh (np.ndarray): A vector containing latitude (rad), longitude (rad) 
        and height (m).

    Returns:
        np.ndarray: A vector in the ECEF frame.
    """
    lat, lng, height = llh
    
    R_N = geodetic_radius(lat)        
    e_2 = const.E2_EARTH
    
    x = (R_N + height) * np.cos(lat) * np.cos(lng)
    y = (R_N + height) * np.cos(lat) * np.sin(lng)
    z = (R_N * (1 - e_2) + height) * np.sin(lat)
    
    return np.array([x, y, z], dtype=np.float64)


def geodetic_radius(lat_geodetic: float) -> float:
    """Finds the radius of earth at a given latitude, accounting
    for Earth's oblateness.

    Args:
        lat_geodetic (float): The geodetic latitude in radians.

    Returns:
        float: The radius of earth in metres.
    """
    a = const.R_EQAT_EARTH
    e_2 = const.E2_EARTH
    return a / np.sqrt(1 - e_2 * (np.sin(lat_geodetic) ** 2))


def geo_ecliptic_to_ECI(
    x_geo_ecliptic: np.ndarray,
    utc: dt.datetime
) -> np.ndarray:
    """Transforms a vector in Geocentric Ecliptic coordinates
    to ECI coordinates.

    Args:
        x_geo_ecliptic (np.ndarray): Vector in Geocentric Ecliptic
        coordinates. utc (dt.datetime): The current UTC time

    Returns:
        np.ndarray: Vector in ECI coordinates.
    """
    jd = time.utc_to_jd(utc)            # Current Julian Date
    n = jd - 2451545.0                  # Days since J2000
    
    obliquity = np.deg2rad(23.439 - 3.56e-7 * n)    # Obliquity of the ecliptic (rad)
    return rot.rot3_x(obliquity) @ x_geo_ecliptic


def ECI_to_LVLH(
    x_eci: np.ndarray,
    r_sat: np.ndarray,
    v_sat: np.ndarray,
    *,
    rotation_only: bool = False
) -> np.ndarray:
    """Converts a vector in ECI coordinates to Local Vertical
    Local Horizontal coordinates.
    
    LVLH has the basis vectors:
    - z: Negative position vector
    - y: Normal to the orbital plane (i.e. specific angular momentum)
    - x: Completes the right-handed coordinate system (x y z)

    Args:
        x_eci (np.ndarray): Vector in ECI coordinates.
        r_sat (np.ndarray): Position vector of satellite in ECI coordinates.
        v_sat (np.ndarray): Velocity vector of satellite in ECI coordinates.
        rotation_only (bool, optional): If True, only the rotation to the
        LVLH frame is applied. Defaults to False.

    Returns:
        np.ndarray: Vector in LVLH coordinates.
    """
    if not rotation_only:
        x_rel = x_eci - r_sat
    else:
        x_rel = x_eci    
    
    z_hat = - r_sat / np.linalg.norm(r_sat)     # Radial unit vector
    h = np.cross(r_sat, v_sat)                  # Specific angular momentum

    y_hat = - h / np.linalg.norm(h)             # negative angular momentum unit vector
    x_hat = np.cross(y_hat, z_hat)
    
    # DCM from ECI to LVLH
    lvlh_dcm = rot.direction_cosine_matrix((x_hat, y_hat, z_hat))
    return lvlh_dcm @ x_rel


def LVLH_to_ECI(
    x_lvlh: np.ndarray,
    r_sat: np.ndarray,
    v_sat: np.ndarray,
    *,
    rotation_only: bool = False
) -> np.ndarray:
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
    eci_dcm = rot.direction_cosine_matrix(
        new=(np.array([1,0,0]), np.array([0,1,0]), np.array([0,0,1])),
        original=(x_hat, y_hat, z_hat)
    )
        
    return eci_dcm @ x_rel

def LVLH_to_bodyfixed(
    x_lvlh: np.ndarray,
    attitude: np.ndarray
) -> np.ndarray:
    """Converts a vector in the LVLH frame into the body fixed frame

    Args:
        x_lvlh (np.ndarray) : The vector in the LVLH frame.
        attitude (np.ndarray): The attitude quaternion
    """
    return rot.quat_rotate(x_lvlh, attitude)

def bodyfixed_to_LVLH(
    x_bf: np.ndarray,
    attitude: np.ndarray
) -> np.ndarray:
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
    
    return rot.quat_rotate(x_bf, att_conj)

def ECI_to_bodyfixed(
    x_eci: np.ndarray,
    r_sat: np.ndarray,
    v_sat: np.ndarray,
    attitude: np.ndarray,
    *,
    rotation_only: bool = False
) -> np.ndarray:
    """Converts a vector in the ECI frame to the body fixed frame

    Args:
        x_eci (np.ndarray): A vector in the ECI frame
        r_sat (np.ndarray): Position vector of satellite in ECI coordinates.
        v_sat (np.ndarray): Velocity vector of satellite in ECI coordinates.
        attitude (np.ndarray): The attitude quaternion

    Returns:
        np.ndarray: A vector in the body fixed frame
    """
    x_lvlh = ECI_to_LVLH(x_eci, r_sat, v_sat, rotation_only=rotation_only)
    return LVLH_to_bodyfixed(x_lvlh, attitude)

def bodyfixed_to_ECI(
    x_bf: np.ndarray,
    r_sat: np.ndarray,
    v_sat: np.ndarray,
    attitude: np.ndarray,
    *,
    rotation_only: bool = False
) -> np.ndarray:
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
    x_lvlh = bodyfixed_to_LVLH(x_bf, attitude)
    return LVLH_to_ECI(x_lvlh, r_sat, v_sat, rotation_only=rotation_only)


def NED_to_ECEF(
    x_ned: np.ndarray,
    r_ecef: np.ndarray,
    *,
    rotation_only: bool = False
) -> np.ndarray:
    """Converts a vector in the North East Down frame to the ECEF frame

    Args:
        x_ned (np.ndarray): A vector in the NED frame
        r_ecef (np.ndarray): Position vector of the NED origin in
        ECEF coordinates.

    Returns:
        np.ndarray: A vector in the ECEF frame
    """
    lat, lng, _ = ECEF_to_LLH(r_ecef)
    
    # rotation matrix from ecef to ned
    R_ned = np.array([
        [-np.sin(lat) * np.cos(lng), -np.sin(lat) * np.sin(lng), np.cos(lat)],
        [-np.sin(lng), np.cos(lng), 0],
        [-np.cos(lat) * np.cos(lng), -np.cos(lat) * np.sin(lng), -np.sin(lat)]
    ])
    
    x_ecef = R_ned.T @ x_ned
    
    if not rotation_only:
        return x_ecef + r_ecef
    
    return x_ecef

def ECEF_to_NED(
    x_ecef: np.ndarray,
    r_ecef: np.ndarray,
    *,
    rotation_only: bool = False
) -> np.ndarray:
    """Converts a vector in the ECEF frame to the NED frame

    Args:
        x_ecef (np.ndarray): A vector in the ECEF frame
        r_ecef (np.ndarray): Position vector of the NED origin in
        ECEF coordinates.

    Returns:
        np.ndarray: A vector in the NED frame
    """
    lat, lng, _ = ECEF_to_LLH(r_ecef)
    
    # rotation matrix from ecef to ned
    R_ned = np.array([
        [-np.sin(lat) * np.cos(lng), -np.sin(lat) * np.sin(lng), np.cos(lat)],
        [-np.sin(lng), np.cos(lng), 0],
        [-np.cos(lat) * np.cos(lng), -np.cos(lat) * np.sin(lng), -np.sin(lat)]
    ])
    
    x_ned = R_ned @ x_ecef
    
    if not rotation_only:
        return x_ned - R_ned @ r_ecef
    
    return x_ned


def NED_to_ECI(
    x_ned: np.ndarray,
    r_eci: np.ndarray,
    utc: dt.datetime,
    *,
    rotation_only: bool = False
) -> np.ndarray:
    """Converts a vector in the North East Down frame to the ECI frame

    Args:
        x_ned (np.ndarray): A vector in the NED frame
        r_eci (np.ndarray): Position vector of the NED origin in
        ECI coordinates.
        utc (dt.datetime): The current UTC time.

    Returns:
        np.ndarray: A vector in the ECI frame
    """
    r_ecef = ECI_to_ECEF(r_eci, utc)
    x_ecef = NED_to_ECEF(x_ned, r_ecef, rotation_only=rotation_only)
    return ECEF_to_ECI(x_ecef, utc)


def NED_to_LVLH(
    x_ned: np.ndarray,
    r_sat: np.ndarray,
    v_sat: np.ndarray,
    utc: dt.datetime,
) -> np.ndarray:
    """Converts a vector in the North East Down frame to the LVLH frame

    Args:
        x_ned (np.ndarray): A vector in the NED frame
        r_sat (np.ndarray): Position vector of satellite in ECI coordinates.
        v_sat (np.ndarray): Velocity vector of satellite in ECI coordinates.
        utc (dt.datetime): The current UTC time.
        

    Returns:
        np.ndarray: A vector in the LVLH frame
    """    
    x_eci = NED_to_ECI(x_ned, r_sat, utc, rotation_only=True)
    x_lvlh = ECI_to_LVLH(x_eci, r_sat, v_sat, rotation_only=True)
    return x_lvlh
    
def polar_to_cartesian(polar: np.ndarray) -> np.ndarray:
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


def cartesian_to_polar(cart: np.ndarray) -> np.ndarray:
    """Converts a vector in the cartesian coordinate system to the polar coordinate system.
    
    The unit of range is the same as the unit of the cartesian vector.

    Args:
        cart (np.ndarray): A vector in the cartesian coordinate system.

    Returns:
        np.ndarray: A vector containing elevation (rad), azimuth (rad), and range.
    """
    x,y,z = cart
    rng = np.sqrt(x**2 + y**2 + z**2)
    azimuth = np.arctan2(y, x)
    elevation = np.arcsin(z / rng)
    
    return np.array([
        elevation,
        azimuth,
        rng
    ], dtype=np.float64).T