"""
Script to test out different fitness funcitons on known good data to try and
figure out what parameters need to be tuned.
"""
#!/usr/bin/env python
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys

if len(sys.argv > 1):
    rosbag_file = sys.argv[1]
else:
    rosbag_file = 'ind_bare.bag'

bag = rosbag.Bag(rosbag_file)



