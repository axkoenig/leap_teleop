# -*- coding: utf-8 -*-

""" Library of functions used for Leap Kuka ROS node.
"""

from math import radians, pi

import numpy as np
import rospy
import tf
import tf2_ros

from geometry_msgs.msg import TransformStamped
from helpers import *

__author__ = "Alexander Koenig"
__copyright__ = "Copyright 2019, Imperial College London"
__credits__ = ["Dr. Fabio Tatti", "Dr. Riccardo Secoli"]
__license__ = "MIT"
__version__ = "0.1"
__maintainer__ = "Alexander Koenig"
__email__ = "awckoenig@gmail.com"
__status__ = "Development"

def get_leap_transform(data, leap_params, kuka_params):
    """ Calculates pose of virtual Leap Motion in KUKA base frame.
    
    The Leap Motion is rotated around its y axis such that direction vector of the first
    
   
    """
    """Idea: direction vector of hand must line up with workspace center line"""

    # gets transformation from base to virtual leap

    # get parameters from dicts
    center_line = kuka_params.get("center_line")
    z_lower_limit = kuka_params.get("z_lower_limit")
    ws_center = kuka_params.get("ws_center")
    z_clearance = leap_params.get("z_clearance")
    direction_x = data.get("dx")
    direction_z = data.get("dz")

    if center_line[2] != 0:
        raise ValueError("Z component of center line not zero. Center line needs to lie in xy-plane.")

    # static transform message
    leap_tf = TransformStamped()
    leap_tf.header.stamp = rospy.Time.now()
    leap_tf.header.frame_id = 'kuka/base_frame'
    leap_tf.child_frame_id = 'leap_motion/virtual_frame'

    # leap origin lies under workspace center
    leap_tf.transform.translation.x = ws_center[0]
    leap_tf.transform.translation.y = ws_center[1]
    leap_tf.transform.translation.z = z_lower_limit - z_clearance

    # desired hand basis vectors (in kuka frame)
    hand_x_kuka = center_line
    hand_z_kuka = [0,0,-1]
    hand_y_kuka = np.cross(hand_z_kuka ,hand_x_kuka)

    # standard basis is kuka frame
    des_quats = rotate_to_basis([hand_x_kuka, hand_y_kuka, hand_z_kuka])

    # measured hand basis vectors (in leap frame), but only care about rotation around leap y
    hand_x_leap = [direction_x, 0, direction_z]
    hand_z_leap = [0,-1,0]
    hand_y_leap = np.cross(hand_z_leap, hand_x_leap)

    # standard basis is leap frame
    mes_quats = rotate_to_basis([hand_x_leap, hand_y_leap, hand_z_leap])
    
    # get orientation of desired leap basis in kuka frame
    mes_quats_inv = tf.transformations.quaternion_inverse(mes_quats)
    quats = tf.transformations.quaternion_multiply(des_quats, mes_quats_inv)

    leap_tf.transform.rotation.x = quats[0]
    leap_tf.transform.rotation.y = quats[1]
    leap_tf.transform.rotation.z = quats[2]
    leap_tf.transform.rotation.w = quats[3]
    
    return leap_tf

def get_hand_transform(data):
    ## view docs for graphical overview of transformations

    # get parameters from dicts
    px = data.get("px")
    py = data.get("py")
    pz = data.get("pz")
    dx = data.get("dx")
    dy = data.get("dy")
    dz = data.get("dz")
    nx = data.get("nx")
    ny = data.get("ny")
    nz = data.get("nz")

    # transform message
    hand_tf = TransformStamped()
    hand_tf.header.stamp = rospy.Time.now()
    hand_tf.header.frame_id = 'leap_motion/virtual_frame'
    hand_tf.child_frame_id = 'hand/frame'

    # hand frame origin at palm position
    hand_tf.transform.translation.x = px
    hand_tf.transform.translation.y = py
    hand_tf.transform.translation.z = pz
    
    # hand basis vectors (in leap frame)
    hand_x = [dx, dy, dz]
    hand_z = [nx, ny, nz]
    hand_y = np.cross(hand_z, hand_x)
    
    # standard basis is leap frame
    quats = rotate_to_basis([hand_x, hand_y, hand_z])
    
    hand_tf.transform.rotation.x = quats[0]
    hand_tf.transform.rotation.y = quats[1]
    hand_tf.transform.rotation.z = quats[2]
    hand_tf.transform.rotation.w = quats[3]
    
    return hand_tf

def get_flange_transform(tool_tf, kuka_params):
    # fixed relation between flange and tool

    tool_length = kuka_params.get("tool_length")
    
    # transform message
    flange_tf = TransformStamped()
    flange_tf.header.stamp = rospy.Time.now()
    flange_tf.header.frame_id = 'kuka/tool_frame/desired'
    flange_tf.child_frame_id = 'kuka/flange_frame/desired'

    # take tool length into account
    flange_tf.transform.translation.x = 0
    flange_tf.transform.translation.y = 0
    flange_tf.transform.translation.z = -tool_length
    
    # no rotation between tool and flange
    flange_tf.transform.rotation.x = 0
    flange_tf.transform.rotation.y = 0
    flange_tf.transform.rotation.z = 0
    flange_tf.transform.rotation.w = 1

    return flange_tf

def get_tool_transform(hand_tf, leap_params, kuka_params):
       
    tol_yaw = radians(leap_params.get("run_tol_yaw"))
    tol_pitch = radians(leap_params.get("run_tol_pitch"))
    tol_roll = radians(leap_params.get("run_tol_roll"))
    
    center_line = kuka_params.get("center_line")

    # transform message
    tool_tf = TransformStamped()
    tool_tf.header.stamp = rospy.Time.now()
    tool_tf.header.frame_id = 'leap_motion/virtual_frame'
    tool_tf.child_frame_id = 'kuka/tool_frame/desired'

    # reference hand basis vectors (in leap frame)
    hand_x_ref = [0,0,-1]
    hand_z_ref = [0,-1,0]
    hand_y_ref = np.cross(hand_z_ref, hand_x_ref)

    # standard basis is leap frame
    quats_hand_ref = rotate_to_basis([hand_x_ref, hand_y_ref, hand_z_ref])

    # get measured hand orientation in leap frame
    quats_hand_mes = [0,0,0,0]
    quats_hand_mes[0] = hand_tf.transform.rotation.x
    quats_hand_mes[1] = hand_tf.transform.rotation.y
    quats_hand_mes[2] = hand_tf.transform.rotation.z
    quats_hand_mes[3] = hand_tf.transform.rotation.w

    # get rotation from reference hand frame to actual hand
    quats_hand_ref_inv = tf.transformations.quaternion_inverse(quats_hand_ref) 
    quats_ypr = tf.transformations.quaternion_multiply(quats_hand_ref_inv, quats_hand_mes)

    # find yaw (hand z) pitch (hand y) roll (hand x) 
    [yaw, pitch, roll] = tf.transformations.euler_from_quaternion(quats_ypr, axes='rzyx')

    # compare to tolerances, clip to max/min values if necessary
    if yaw > tol_yaw: yaw = tol_yaw
    elif yaw < - tol_yaw: yaw = - tol_yaw

    if pitch > tol_pitch: pitch = tol_pitch
    elif pitch < - tol_pitch: pitch = - tol_pitch

    if roll > tol_roll: roll = tol_roll
    elif roll < - tol_roll: roll = - tol_roll

    # get quaternions from fixed hand reference frame to adjusted hand frame
    quats_ypr_adjusted = tf.transformations.quaternion_from_euler(yaw, pitch, roll, axes='rzyx')

    # get rotation from virtual leap frame to adjusted hand frame
    quats_hand_adjusted = tf.transformations.quaternion_multiply(quats_hand_ref, quats_ypr_adjusted)
    
    # check if we started off in positive or negative half
    if center_line[0] >= 0:
        # tool frame is rotated 180 degrees around z axis of adjusted hand frame
        quats_hand_tool = tf.transformations.quaternion_from_euler(pi, 0, 0, axes='rzyx')
        # get rotation from virtual leap frame to tool frame
        quats = tf.transformations.quaternion_multiply(quats_hand_adjusted, quats_hand_tool)
    else:
        quats = quats_hand_adjusted

    tool_tf.transform.rotation.x = quats[0]
    tool_tf.transform.rotation.y = quats[1]
    tool_tf.transform.rotation.z = quats[2]
    tool_tf.transform.rotation.w = quats[3]

    # tool position coincides with hand center
    tool_tf.transform.translation.x = hand_tf.transform.translation.x
    tool_tf.transform.translation.y = hand_tf.transform.translation.y
    tool_tf.transform.translation.z = hand_tf.transform.translation.z

    return tool_tf