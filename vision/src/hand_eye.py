#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" ROS node to publish depth image in KUKA base frame
"""

import time

import rospy
import tf

from sensor_msgs.msg import Image
from common.lib import *

__author__ = "Alexander Koenig"
__copyright__ = "Copyright 2019, Imperial College London"
__credits__ = ["Dr. Fabio Tatti", "Dr. Riccardo Secoli"]
__license__ = "MIT"
__version__ = "0.1"
__maintainer__ = "Alexander Koenig"
__email__ = "awckoenig@gmail.com"
__status__ = "Development"

depth_img = Image()

def depth_callback(depth_data):
    
    global depth_img
     
##### COMMENT #####
# work in progress ...


if __name__ == '__main__':
    
    rospy.init_node('depth_transformer_node')

    listener = tf.TransformListener()
    listener.waitForTransform('kuka/base_frame', 'camera/depth_frame')

    depth_sub = rospy.Subscriber('/camera/depth/image_rect', Image, depth_callback)
    depth_pub = rospy.Publisher('/camera/depth/image_rect_base', Image, queue_size = 1)

    rate = rospy.Rate(10)
    global depth_img

    while not rospy.is_shutdown():
        depth_img_msg = Image()







    listener = tf.TransformListener()

    # get latest transform
    while not rospy.is_shutdown():
            
        try:
            (trans, rot) = listener.lookupTransform('/camera', '/tag', rospy.Time(0))
            print(trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue 
    
    while not rospy.is_shutdown():
            
        if is_topic_published('flange_pose'):
            break 
        else:
            rospy.loginfo("Waiting for MATLAB to publish on flange_pose topic.")
            time.sleep(1)
    
    rospy.Subscriber('matlab/flange_pose', transform, talker)
    rospy.spin()