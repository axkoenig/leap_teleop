# -*- coding: utf-8 -*-

""" Library of functions used for Leap Kuka ROS node.
"""

from math import radians

import numpy as np
import rospy
import tf
import tf2_ros

from geometry_msgs.msg import TransformStamped

__author__ = "Alexander Koenig"
__copyright__ = "Copyright 2019, Imperial College London"
__credits__ = ["Dr. Fabio Tatti", "Dr. Riccardo Secoli"]
__license__ = "MIT"
__version__ = "0.1"
__maintainer__ = "Alexander Koenig"
__email__ = "awckoenig@gmail.com"
__status__ = "Development"

class NotEnoughSamples(Exception):
    pass

def load_params(namespace, verbose=False):
    """ Loads all parameters in given namespace from parameter server.
    
    Important note: before calling this function the parameters have to be uploaded to the parameter
    server (e.g. by loading .yaml file in launch file)
    
    Parameters
    ----------
    namespace : string
        namespace that should be loaded
    verbose : bool, optional
        print loaded parameters, by default False
    
    Returns
    -------
    params: dict
        all parameters and their values in given namespace
    
    Raises
    ------
    TypeError
        invalid namespace format
    ValueError
        namespace not found
    """
    if type(namespace) is not str:
        raise TypeError("Namespace needs to be of type string.")
    
    try:
        params = rospy.get_param(namespace)
    except KeyError:
        raise ValueError("No namespace called '{}' on parameter server.".format(namespace))
    
    if verbose:
        rospy.loginfo("Loaded parameters from '{}' namespace.".format(namespace))
        rospy.loginfo("----------------------------------------")
        for item in params:
            rospy.loginfo(str(item) + ": " + str(params[item]))
    
    return params

def is_param_available(namespace, param):

    if type(namespace) is not str:
        raise TypeError("Namespace needs to be of type string.")
    if type(param) is not str:
        raise TypeError("Parameter needs to be of type string.")
    
    try:
        params = rospy.get_param(namespace)
    except KeyError:
        raise ValueError("No namespace called '{}' on parameter server.".format(namespace))
    
    if param in params:
        param_available = True 
    else: 
        param_available = False

    return param_available

def is_valid_start(data, leap_params, kuka_params):
    # leap params is dict 
 
    # get parameters from dicts
    box_dimensions = leap_params.get("box_dimensions")
    tol_pitch = radians(leap_params.get("start_tol_pitch"))
    tol_roll = radians(leap_params.get("start_tol_roll"))
    z_clearance = leap_params.get("z_clearance")
    z_lower_limit = kuka_params.get("z_lower_limit")
    ws_center = kuka_params.get("ws_center")
    palm_center_x = data.get("px")
    palm_center_y = data.get("py")
    palm_center_z = data.get("pz")
    pitch = data.get("p")
    roll = data.get("r")

    # initialize constraint flags
    box_constraint = False
    orient_constraint = False
    valid_start = False

    # box center is located along leap y axis 
    box_center_x = 0
    box_center_z = 0

    # calculate height such that box center coincides with kuka workspace center
    box_center_y = ws_center[2] - z_lower_limit + z_clearance

    box_center = [box_center_x, box_center_y, box_center_z]
    palm_pos = [palm_center_x, palm_center_y, palm_center_z]

    # calculate min and max vertex of box
    box_max = [a + b/2 for a, b in zip(box_center, box_dimensions)]
    box_min = [a - b/2 for a, b in zip(box_center, box_dimensions)]

    # box constraint
    if (palm_pos[0] >= box_min[0] and palm_pos[0] <= box_max[0] and
        palm_pos[1] >= box_min[1] and palm_pos[1] <= box_max[1] and
        palm_pos[2] >= box_min[2] and palm_pos[2] <= box_max[2]):

        box_constraint = True
    
    # pitch and roll constraints (yaw is unconstrained as leap frame is rotated accordingly)
    if abs(pitch) < tol_pitch and abs(roll) < tol_roll:

        orient_constraint = True
    
    # both constraints need to be fulfilled
    if box_constraint and orient_constraint:

        valid_start = True
    else:
        rospy.loginfo("To start streaming put hand in interaction box and hold flat.")
    
    return valid_start

def average_data(hand, buffer, counter, weight_array=None):

    buffer_size = buffer.shape[1]

    px = hand.palm_center.x
    py = hand.palm_center.y
    pz = hand.palm_center.z

    dx = hand.direction.x
    dy = hand.direction.y
    dz = hand.direction.z

    nx = hand.normal.x
    ny = hand.normal.y
    nz = hand.normal.z

    y = hand.yaw
    p = hand.pitch
    r = hand.roll

    c = hand.confidence

    # data as column vector
    data_vec = np.array([px, py, pz, dx, dy, dz, nx, ny, nz, y, p, r, c]).reshape(-1,1)

    # roll buffer and overwrite oldest data vector (FIFO)
    buffer = np.roll(buffer, -1, axis=1)
    buffer = np.delete(buffer, -1, axis=1)
    buffer = np.append(buffer, data_vec, axis=1)
    
    # check if enough new samples collected
    if counter > buffer_size:
        
        avg_data_vec = np.average(buffer, axis=1, weights=weight_array)
        
        # create dict for better readability
        info = ["px", "py", "pz", "dx", "dy", "dz", "nx", "ny", "nz", "y", "p", "r", "c"]
        data_dict = dict(zip(info, avg_data_vec))

    else:
        raise NotEnoughSamples("Not enough samples collected yet.")

    return data_dict, buffer, counter

def rotate_to_basis(new_basis):
    """ Calculates rotation between standard basis and given orthogonal basis.
    
    Standard basis is defined by [1,0,0], [0,1,0] and [0,0,1]. New basis vectors are given in
    the standard basis. The new basis vectors need to be linearly independent and the basis 
    must be invertible.
    
    Parameters
    ----------
    new_basis : multidimensional array
        new basis vectors as seen from standard basis
    
    Returns
    -------
    quats : float array
        unit quaternion specifying rotation
    """

    new_x = new_basis[0]
    new_y = new_basis[1]
    new_z = new_basis[2]

    # in case we are given non-orthonormal basis
    new_x = new_x / np.linalg.norm(new_x)
    new_y = new_y / np.linalg.norm(new_y)
    new_z = new_z / np.linalg.norm(new_z)

    # rotation from standard basis to new basis is inverse of new basis as shown in standard basis
    R = np.linalg.inv(np.array([new_x, new_y, new_z]))

    # quaternion_from_matrix() wants full 4*4 transformation matrix 
    R = np.append(R, np.zeros((3,1)), axis=1)
    R = np.append(R, np.array([[0,0,0,1]]), axis=0)

    quats = tf.transformations.quaternion_from_matrix(R)
    quats = quats / np.linalg.norm(quats)
    
    return quats