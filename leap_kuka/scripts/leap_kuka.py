#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" ROS Node to communicate between Leap Motion ROS driver and KUKA Sunrise Toolbox (Matlab).
"""

import rospy
import tf2_ros
import numpy as np

from leap_motion.msg import Human
from common.lib import *
from helpers import *
from transformations import *

__author__ = "Alexander Koenig"
__copyright__ = "Copyright 2019, Imperial College London"
__credits__ = ["Dr. Fabio Tatti", "Dr. Riccardo Secoli"]
__license__ = "MIT"
__version__ = "0.1"
__maintainer__ = "Alexander Koenig"
__email__ = "awckoenig@gmail.com"
__status__ = "Development"

rospy.init_node('leap_kuka_node')

# wait until matlab uploaded workspace center to param server
while not rospy.is_shutdown():
    if is_param_available("kuka", "ws_center"):
        break 
    else:
        rospy.loginfo("Waiting for parameter 'kuka/ws_center' on ROS parameter server.")
        rospy.sleep(1)

# load parameters from parameter server in dict
leap_params = load_params("leap", verbose = True)
kuka_params = load_params("kuka", verbose = True)

# variables
buffer_size = leap_params.get("buffer_size")
min_confidence = leap_params.get("min_confidence")
data_buffer = np.zeros((13, buffer_size))
counter = 0
streaming = False

# broadcasters
leap_tf_br = tf2_ros.StaticTransformBroadcaster()   # for virtual leap pose (in kuka frame)
hand_tf_br = tf2_ros.TransformBroadcaster()         # for hand pose (in leap frame)
tool_tf_br = tf2_ros.TransformBroadcaster()         # for tool pose (in hand frame)
flange_tf_br = tf2_ros.TransformBroadcaster()       # for flange pose (in tool frame)

def leap_callback(leap_data):

    global data_buffer
    global counter
    global streaming
    global min_confidence

    hand = get_current_hand(leap_data)
    
    if hand is not None:
        
        # upon first hand detection wait briefly for hand to stabilize
        if not streaming:
            rospy.sleep(0.1)

        try:
            # average leap motion data to reduce noise
            avg_data, data_buffer, counter = average_data(hand, data_buffer, counter)
            avg_confidence = avg_data.get("c")
            
            # send hand transform
            hand_tf = get_hand_transform(avg_data)
            hand_tf_br.sendTransform(hand_tf)
              
            # take confidence into account
            confident = True if avg_confidence > min_confidence else False
            
            if streaming and not confident:

                rospy.loginfo("Hand data not confident enough, stopping stream.")
                streaming = False

            if not streaming:
                
                # check if we should stream data
                start_stream = is_valid_start(avg_data, leap_params, kuka_params) 
  
                if start_stream and confident:

                    # update virtual leap motion pose before streaming
                    leap_tf = get_leap_transform(avg_data, leap_params, kuka_params)
                    leap_tf_br.sendTransform(leap_tf)
                    rospy.loginfo("Palm in starting position, starting stream.")
                    streaming = True
            
            else:

                # send flange and tool transforms
                tool_tf = get_tool_transform(hand_tf, leap_params, kuka_params)
                flange_tf = get_flange_transform(tool_tf, kuka_params)
                rospy.loginfo("Sending tool and flange pose.")
                tool_tf_br.sendTransform(tool_tf)
                flange_tf_br.sendTransform(flange_tf) 

        except NotEnoughSamples as e:
            rospy.loginfo(e)
       
        counter = counter + 1
       
    else:

        # lost hand, reset flags
        counter = 0
        streaming = False
        start_stream = False
        rospy.loginfo("No hand detected.")
       

rospy.loginfo("Check that Leap Deamon is running.")

rospy.Subscriber('leap_motion/leap_device', Human, leap_callback, queue_size=1)
rospy.spin()