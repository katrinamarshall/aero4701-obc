"""This module handles everything rotation related, including
rotation matrices, conversion to and from unit quaternions and
rotation of vectors by unit quaternions.

Unit quaternions follow the Hamiltonian convention where the
scalar part is the first element of the quaternion i.e.

    q = [q0, q1, q2, q3]    where q0 is the scalar part.

Contains:
---------
    - rot3_x: Get a 3x3 rotation matrix about the x-axis.
    - rot3_y: Get a 3x3 rotation matrix about the y-axis.
    - rot3_z: Get a 3x3 rotation matrix about the z-axis.
    - rot2: Get a 2x2 rotation matrix.
    - dcm_from_quaternion: Get a Direction Cosine Matrix from a quaternion.
    - quaternion_from_dcm: Get a quaternion from a Direction Cosine Matrix.
    - quat_product: Multiply two quaternions.
    - quat_rotate: Rotate a vector by a quaternion.
    - ryp_from_quart: Get pitch, yaw and roll from a quaternion.
    - quart_from_ryp: Get a quaternion from pitch, yaw and roll.
    - direction_cosine_matrix: Get the direction cosine matrix from one basis to another.
"""
import numpy as np


def rot3_x(theta: float, degrees: bool = False) -> np.ndarray:
    """Returns a 3x3 rotation matrix about the x-axis.
    
    Args:
        theta (float): The angle of rotation.
        degrees (bool, optional): Whether the angle is in
        degrees or radians. Defaults to False.
    
    Returns:
        np.ndarray: The X-rotation matrix.
    """
    theta = np.radians(theta) if degrees else theta
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])


def rot3_y(theta: float, degrees: bool = False):
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


def rot3_z(theta: float, degrees: bool = False) -> np.ndarray:
    """Returns a 3x3 rotation matrix about the z-axis.

    Args:
        theta (float): The angle of rotation.
        degrees (bool, optional): Whether the angle is in
        degrees or radians. Defaults to False.

    Returns:
        np.ndarray: The Z-rotation matrix.
    """
    theta = np.radians(theta) if degrees else theta
    
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])


def rot2(theta: float, degrees: bool = False) -> np.ndarray:
    """Returns a 2x2 rotation matrix.

    Args:
        theta (float): The angle of rotation.
        degrees (bool, optional): Whether the angle is in
        degrees or radians. Defaults to False.

    Returns:
        np.ndarray: The 2D rotation matrix.
    """
    theta = np.radians(theta) if degrees else theta
    
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])


def dcm_from_quaternion(q: np.ndarray) -> np.ndarray:
    """Gets a Direction Cosine Matrix from a quaternion.
    
    The quaternion is assumed to be in the form [q0, q1, q2, q3]
    where q0 is the scalar part.

    Args:
        q (np.ndarray): quaternion

    Returns:
        np.ndarray: The (3 x 3) Direction Cosine Matrix
    """
    # This code works for a quaternion where q3 is the scalar part
    # Hence, we rotate the quaternion to make q0 the scalar part
    q = np.roll(q, -1)
    q0, q1, q2, q3 = q
    
    # Row 1
    M11 = q0**2 - q1**2 - q2**2 + q3**2
    M12 = 2*(q0*q1 + q2*q3)
    M13 = 2*(q0*q2 - q1*q3)
    
    # Row 2
    M21 = 2*(q0*q1 - q2*q3)
    M22 = -q0**2 + q1**2 - q2**2 + q3**2
    M23 = 2*(q0*q3 + q1*q2)
    
    # Row 3
    M31 = 2*(q0*q2 + q1*q3)
    M32 = 2*(q1*q2 - q0*q3)
    M33 = -q0**2 - q1**2 + q2**2 + q3**2
    
    return np.array([
        [M11, M12, M13],
        [M21, M22, M23],
        [M31, M32, M33]
    ])


def quaternion_from_dcm(dcm: np.ndarray) -> np.ndarray:
    """Gets a quaternion from a Direction Cosine Matrix.

    The quaternion is in the form [q0, q1, q2, q3]
    where q0 is the scalar part.
    
    Args:
        dcm (np.ndarray): The (3 x 3) Direction Cosine Matrix

    Returns:
        np.ndarray: quaternion in the form [q0, q1, q2, q3]
    """
    # unpack the DCM
    M11, M12, M13 = dcm[0]
    M21, M22, M23 = dcm[1]
    M31, M32, M33 = dcm[2]
    
    # Compute the K matrix
    K11 = M11 - M22 - M33
    K12 = M12 + M21
    K13 = M13 + M31
    K14 = M23 - M32
    
    K21 = M12 + M21
    K22 = -M11 + M22 - M33
    K23 = M23 + M32
    K24 = M31 - M13
    
    K31 = M13 + M31
    K32 = M23 + M32
    K33 = -M11 - M22 + M33
    K34 = M12 - M21
    
    K41 = M23 - M32
    K42 = M31 - M13
    K43 = M12 - M21
    K44 = M11 + M22 + M33
    
    K_mtx = (1 / 3) * np.array([
        [K11, K12, K13, K14],
        [K21, K22, K23, K24],
        [K31, K32, K33, K34],
        [K41, K42, K43, K44]
    ])
    
    # The quaternion is the eigenvector corresponding to the largest eigenvalue
    eigenvalues, eigenvectors = np.linalg.eig(K_mtx)
    quaternion = eigenvectors[:, np.argmax(eigenvalues)]
    
    quaternion = np.roll(quaternion, 1)
    return quaternion


def quat_product(q: np.ndarray, p: np.ndarray) -> np.ndarray:
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


def quat_rotate(x: np.ndarray, q: np.ndarray) -> np.ndarray:
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
    
    x_prime = quat_product(quat_product(q, x_quat), q_conj)
    
    return x_prime[1:]


def ryp_from_quart(q: np.ndarray) -> tuple:
    """Get pitch, yaw and roll from a quaternion.
    
    - Pitch is rotation about the x-axis
    - Yaw is rotation about the y-axis
    - Roll is rotation about the z-axis

    Args:
        q (np.ndarray): A quaternion in the form [q0, q1, q2, q3]
        where q0 is the scalar part.    

    Returns:
        tuple: The pitch, yaw and roll in radians.
    """
    q0, q1, q2, q3 = q

    roll = np.arctan2(
        2 * (q0 * q1 + q2 * q3),
        1 - 2 * (q1**2 + q2**2)
    )
    
    pitch = np.arcsin(
        2 * (q0 * q2 - q3 * q1)
    )
    
    yaw = np.arctan2(
        2 * (q0 * q3 + q1 * q2),
        1 - 2 * (q2**2 + q3**2)
    )
    
    return roll, yaw, pitch


def quart_from_ryp(
    roll: float,
    yaw: float,
    pitch: float
) -> float:
    """Get a quaternion from pitch, yaw and roll.
    
    - Pitch is rotation about the x-axis
    - Yaw is rotation about the y-axis
    - Roll is rotation about the z-axis

    Args:
        pitch (float): The pitch in radians.
        yaw (float): The yaw in radians.
        roll (float): The roll in radians.

    Returns:
        float: The quaternion in the form [q0, q1, q2, q3]
        where q0 is the scalar part.
    """
    q0 = (np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
        + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    )
    
    q1 = (np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
        - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    )
    
    q2 = (np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    )
    
    q3 = (np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    )
    
    return np.array([q0, q1, q2, q3])


def direction_cosine_matrix(
    new: tuple,
    original: tuple = (
        np.array([1, 0, 0]),
        np.array([0, 1, 0]),
        np.array([0, 0, 1])
    )
) -> np.ndarray:
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
    
def ryp_from_dcm(dcm: np.ndarray):
    """Get pitch, yaw and roll from a Direction Cosine Matrix.
    
    - Pitch is rotation about the x-axis
    - Yaw is rotation about the y-axis
    - Roll is rotation about the z-axis

    Args:
        dcm (np.ndarray): The (3 x 3) Direction Cosine Matrix

    Returns:
        tuple: The pitch, yaw and roll in radians.
    """
    roll = np.arctan2(dcm[2, 1], dcm[2, 2])
    pitch = np.arcsin(-dcm[2, 0])
    yaw = np.arctan2(dcm[1, 0], dcm[0, 0])
    
    return roll, yaw, pitch