#!/usr/bin/env python3
import numpy as np

import rospy
from crazyflie.srv import *
from geometry_msgs.msg import Point

from IPython.core.debugger import set_trace

import time


def run():


    rospy.wait_for_service('/cvo')
    try:
        cvo_service = rospy.ServiceProxy('/cvo', cvo)
        av1Xo = Point(0,0,0)
        # av1Xo.x = 0

        current_time = time.time()
        resp1 = cvo_service(Point(0,0,0),
                            Point(1,0,0),
                            Point(1.75,1.0,0),
                            1/30.0,
                            [Point(0,0,0)],
                            [Point(0,0,0)])
        print(time.time()-current_time)
        print(resp1)



    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():
	#initialize the node
	rospy.init_node('test_srv')

	#run testing
	run()

	# #spin
	# try:
	# 	rospy.spin()
	# except KeyBoardInterrupt:
	# 	print("Shutting down")
	# #OpenCV cleanup
	# cv.destroyAllWindows()

if __name__ == '__main__':
	main()
