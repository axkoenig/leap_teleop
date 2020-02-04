#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" Small library of helper functions.
"""

import rospy
import numpy as np

__author__ = "Alexander Koenig"
__copyright__ = "Copyright 2019, Imperial College London"
__credits__ = ["Dr. Fabio Tatti", "Dr. Riccardo Secoli"]
__license__ = "MIT"
__version__ = "0.1"
__maintainer__ = "Alexander Koenig"
__email__ = "awckoenig@gmail.com"
__status__ = "Development"

def get_current_hand(leap_data):
    
    # get hand that is visible for the longest
    time_left = leap_data.left_hand.time_visible
    time_right = leap_data.right_hand.time_visible

    if time_left > time_right:
        hand = leap_data.left_hand
    elif time_left < time_right:
        hand = leap_data.right_hand
    else:
        # no hand detected
        hand = None
    
    return hand