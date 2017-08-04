#!/usr/bin/env python
import rospy
import sys

rospy.init_node('test_yapflm_import')

rospy.loginfo('PYTHON_PATH:\n{}'.format(sys.path))

import yapflm
from yapflm import fisyaml
rospy.loginfo("Successfully import yapflm from {}".format(yapflm.__file__))

import irols.utils
rospy.loginfo("dir(yapflm)={}".format(dir(yapflm)))
