#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_recalibration_values')
import rospy
from pr2_recalibration_values.srv import *
import time

def test_controller():
    rospy.init_node('pr2_recalibration_values')
    set_joint = rospy.ServiceProxy('recalibration_controller/get_calibration_offset',GetCalibrationOffset)
    foo = GetCalibrationOffsetRequest();
    a = set_joint(foo)
    print a

if __name__ == '__main__':
    try:
        test_controller()
    except rospy.ROSInterruptException: pass

