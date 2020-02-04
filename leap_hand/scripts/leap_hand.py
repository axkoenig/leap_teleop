#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" ROS Node to communicate between Leap Motion and RightHand Robotics ReFlex TakkTile.
"""

from math import degrees, radians

import rospy
import numpy as np

from reflex_msgs.msg import RadianServoCommands
from leap_motion.msg import Human
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from common.lib import *

__author__ = "Alexander Koenig"
__copyright__ = "Copyright 2019, Imperial College London"
__credits__ = ["Dr. Fabio Tatti", "Dr. Riccardo Secoli"]
__license__ = "MIT"
__version__ = "0.1"
__maintainer__ = "Alexander Koenig"
__email__ = "awckoenig@gmail.com"
__status__ = "Development"

# human_angle limits of human and robotic hand [index, middle, thumb, spread]
min_human_angles = [40.0, 40.0, 40.0, 25.0]		# [deg] closed human hand
max_human_angles = [85.0, 85.0, 85.0, 40.0]		# [deg]	opened human hand
min_robot_angles = [0.0, 0.0, 0.0, 0.0]			# [deg] opened robot hand
max_robot_angles = [115.0, 115.0, 100, 40.0]    # [deg] closed robot hand

# initialize other variables
robot_angles = min_robot_angles
hand_active = False

# can be changed in launch file, default: True (so node can be used independently)
hand_only = rospy.get_param('leap_hand_node/hand_only', True)

status_topic = '/kuka/gripper_control'
leap_topic = '/leap_motion/leap_device'
control_topic = '/reflex_takktile/radian_hand_command'
calibr_fingers_srv = '/reflex_takktile/calibrate_fingers'
calibr_tactile_srv = '/reflex_takktile/calibrate_tactile'
node_name = 'leap_hand_node'

def get_angle(a, b, deg=False):
    """ Computes angle between two vectors.
    
    Parameters
    ----------
    a : float array
        first vector
    b : float array
        second vector
    deg : bool, optional
        convert output to degrees, by default False
    
    Returns
    -------
    angle : float array
        computed angle
    """
    
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)

    # calculate angles with inner product
    angle = np.arccos(np.dot(a, b) / (norm_a * norm_b))
    
    if deg:
        angle = degrees(angle)

    return angle 

def is_topic_published(topic_name):
    """ Checks if topic exist.
    
    Searches list of topics for topic name.
    
    Parameters
    ----------
    topic_name : string
        name of topic
    
    Returns
    -------
    found: bool
        result of check
    """

    found = False
    topic_list = rospy.get_published_topics()

    # iterate through topic list
    for i in range(len(topic_list)):
        if topic_name in topic_list[i][0]:
            found = True

    return found

def get_human_angles(hand):

    """ Returns flexion and spread angles of human hand based on Leap Motion data.
    
    Calculates vectors from center of palm (Leap Motion palm position) to tips of thumb, index and middle finger
    and from center of palm to intermediate bone of index and middle finger. Flexion angles for thumb, index and 
    middle finger are calculated based on inner product of palm normal and the respective vectors. Spread angle
    between index and middle finger is based on inner product between those vectors.
    
    Parameters
    ----------
    hand : Hand
        ROS message coming from Leap Motion ROS driver
    
    Returns
    -------
    human_angles : float array
        flexion and spread angles of human hand

    Raises
    ------
    ValueError
        not all fingers visible
    """

    # all fingers must be visible
    if len(hand.finger_list) != 5:
        raise ValueError("Leap Motion does not detect all fingers.")

    # get info from leap ros message 
    thumb_tip = hand.finger_list[0].bone_list[2].bone_end.position
    index_tip = hand.finger_list[1].bone_list[2].bone_end.position
    middle_tip = hand.finger_list[2].bone_list[2].bone_end.position
    palm_center = hand.palm_center
    palm_normal = hand.normal
    index_middle = hand.finger_list[1].bone_list[1].center
    middle_middle = hand.finger_list[2].bone_list[1].center
    
    # vectors from palm center to each of the finger tips
    vec_thumb = [thumb_tip.x - palm_center.x, thumb_tip.y - palm_center.y, thumb_tip.z - palm_center.z]
    vec_index = [index_tip.x - palm_center.x, index_tip.y - palm_center.y, index_tip.z - palm_center.z]
    vec_middle = [middle_tip.x - palm_center.x, middle_tip.y - palm_center.y, middle_tip.z - palm_center.z]
    vec_normal = [palm_normal.x, palm_normal.y, palm_normal.z]
    vec_index_prox = [index_middle[0]- palm_center.x, index_middle[1] - palm_center.y, index_middle[2] - palm_center.z]
    vec_middle_prox = [middle_middle[0] - palm_center.x, middle_middle[1] - palm_center.y, middle_middle[2] - palm_center.z]

    ang_thumb = get_angle(vec_thumb, vec_normal, deg=True)
    ang_index = get_angle(vec_index, vec_normal, deg=True)
    ang_middle = get_angle(vec_middle, vec_normal, deg=True)
    ang_spread = get_angle(vec_index_prox, vec_middle_prox, deg=True)

    human_angles = [ang_index, ang_middle, ang_thumb, ang_spread]

    return human_angles

def get_robot_angles(min_human_angles, max_human_angles, min_robot_angles, max_robot_angles, human_angles, robot_angles):
    """ Creates a mapping from human finger flexion angles to robot commands.

    For the flexion angles of thumb, index and middle finger a high value for the human angle should result
    in a low value for the robotic hand angle. Contrarily, for the spread angle between index and middle
    finger a high value for the human angle should result in a high value for the robotic hand angle. The two
    code blocks are very similar, in that they clip the robotic hand angle to its specified maximum / minimum
    values if necessary and they map the human angle to the robotic hand angle if in the robotic hand's working
    range.

    Parameters
    ----------
    min_human_angles : float array
        minimum flexion and spread angles of human hand
    max_human_angles : float array
        maximum flexion and spread angles of human hand
    min_robot_angles : float array
        minimum flexion and spread angles of robotic hand
    max_robot_angles : float array
        maximum flexion and spread angles of robotic hand
    human_angles : float array
        flexion and spread angles of human hand
    robot_angles : float array
        flexion and spread angles of robotic hand

    Returns
    -------
    robot_angles : float array
        flexion and spread angles of robotic hand
    """

    # approach for index, middle and thumb: high finger human_angle should lead to small robot_angles
    for i in range(3):

        human_angle = human_angles[i]
        max_human_angle = max_human_angles[i]
        min_human_angle = min_human_angles[i]
        max_robot_angle = max_robot_angles[i]
        min_robot_angle = min_robot_angles[i]

        # preventing to open hand too far
        if human_angle >= max_human_angle:
            robot_angles[i] = min_robot_angle

        # preventing to close hand too far
        elif human_angle <= min_human_angle:
            robot_angles[i] = max_robot_angle

        # angles in valid working range, interpolating
        elif (human_angle < max_human_angle) and (human_angle > min_human_angle):
            hand_range = max_human_angle - min_human_angle
            robot_range = max_robot_angle - min_robot_angle
            robot_angles[i] = (max_human_angle - human_angle) * robot_range / hand_range

        else:
            robot_angles = min_robot_angles[:]
            break

    # approach for spread: high finger human_angle should lead to high robot_angles human_angle
    human_spread = human_angles[3]
    max_human_spread = max_human_angles[3]
    min_human_spread = min_human_angles[3]
    max_robot_spread = max_robot_angles[3]
    min_robot_spread = min_robot_angles[3]

    if human_spread >= max_human_spread:
        robot_angles[3] = max_robot_spread
    elif human_spread <= min_human_spread:
        robot_angles[3] = min_robot_spread
    elif (human_spread < max_human_spread) and (human_spread > min_human_spread):
        hand_range_spread = max_human_spread - min_human_spread
        robot_range_spread = max_robot_spread - min_robot_spread
        robot_angles[3] = (human_spread - min_human_spread) * robot_range_spread / hand_range_spread
    else:
        robot_angles = min_robot_angles[:]

    return robot_angles

def leap_callback(leap_data):
    """ Callback function which publishes commands to robotic hand.
    
    Function gets correct commands for robotic hand from human flexion angles if hand is detected. Else it keeps
    command in the open hand position. If flag for robotic hand control is active it publishes the command to the
    ROS network. 
    
    Parameters
    ----------
    leap_data : Human
        ROS message coming from Leap Motion ROS driver
    """
    global min_robot_angles, robot_angles

    hand = get_current_hand(leap_data)
    
    if hand is not None:
        # calculate hand and robot angles [index, middle, thumb, human_spread]
        try:
            human_angles = get_human_angles(hand)
            # print("human " + str(human_angles))
            robot_angles = get_robot_angles(min_human_angles, max_human_angles, min_robot_angles, max_robot_angles, human_angles, robot_angles)
            # print("robot " + str(robot_angles))
        except ValueError as e:
            rospy.loginfo(e)
            robot_angles = min_robot_angles[:]
        
    else:
        # keep robotic hand in zero position
        robot_angles = min_robot_angles[:]

    # publish
    if hand_active:
        robot_angles = convert_to_radians(robot_angles)
        pub_hand.publish(robot_angles)
    else:
        min_robot_angles = convert_to_radians(min_robot_angles)
        pub_hand.publish(min_robot_angles)

def convert_to_radians(array):
    """ Converts each array entry from degrees to radians. 
    
    Parameters
    ----------
    array : float array
        input array in degrees
    
    Returns
    -------
    array : float array
        output array in radians
    """
    
    for i in range(len(array)):
        array[i] = radians(array[i])

    return array 

def receive_status(status):
    """ Callback function which checks if received flag is true or false and sets global variable.

    Parameters
    ----------
    status : bool
        Flag published by Matlab indicating whether Leap Motion control of robotic hand should be active or not.
    """
    
    global hand_active

    # check if button in Matlab GUI pressed
    if status.data == True:
        hand_active = True
    else:
        hand_active = False

def calibrate_hand():
    """Positional and tactile calibration of robotic hand."""

    # calibrate and wait for it to finish
    calibrate_fingers = rospy.ServiceProxy(calibr_fingers_srv, Empty)
    calibrate_tactile = rospy.ServiceProxy(calibr_tactile_srv, Empty)
    calibrate_fingers()
    calibrate_tactile()
    rospy.sleep(2)

if __name__ == '__main__':

    rospy.init_node(node_name)
    pub_hand = rospy.Publisher(control_topic, RadianServoCommands, queue_size=1)
    calibrate_hand()

    if hand_only:
        # node runs without listening to boolean on ROS network 
        hand_active = True

    else:
        # node waits for someone to publish boolean
        while not rospy.is_shutdown():
            
            if is_topic_published(status_topic):
                break 
            else:
                rospy.loginfo("Waiting for messages on '" + status_topic + "' topic.")
                pub_hand.publish(min_robot_angles)
                rospy.sleep(1)

        rospy.Subscriber(status_topic, Bool, receive_status)

    rospy.Subscriber(leap_topic, Human, leap_callback, queue_size=1)
    rospy.loginfo("Check that Leap Deamon is running.")

    rospy.spin()