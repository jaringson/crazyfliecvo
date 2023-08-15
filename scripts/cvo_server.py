#!/usr/bin/env python3


from __future__ import print_function

import numpy as np
from numpy.linalg import norm
from cvo_gekko import CVOGekko
from quad_wp_manager import WPManager
from gen_kalman_filter import GenKalmanFilter
import params

import rospy
import rospkg
from crazyflie.srv import * #add_subscriber, add_subscriberResponse
from crazyflie.msg import xhat, float32_1d
from geometry_msgs.msg import Pose, Point, PoseStamped
import copy
import math

from IPython.core.debugger import set_trace


class CVOServer:
    def __init__(self):
        self.allPositions_ = {}
        self.allVelocities_ = {}
        self.allQuats_ = {}
        self.allWPManagers_ = {}

        self.allKalFilters_ = {}
        self.allUncertaintyPos_ = {}
        self.allUncertaintyVel_ = {}

        self.range = params.collision_range

        self.mocap_ids_ = []
        # allTimes_ = []

        # self.cvoGekko = CVOGekko(params)

        self.time_d1 = rospy.get_time()

        self.allPubs = {}

        s_cvo = rospy.Service('/cvo', cvo, self.get_velocity)
        s_add_sub = rospy.Service('/add_subscriber', add_subscriber, self.add_sub)

    def truncate(self, number, digits) -> float:
        # set_trace()
        nbDecimals = len(str(number).split('.')[1])
        if nbDecimals <= digits:
            return number
        stepper = 10**digits
        return math.trunc(stepper*number)/stepper

    def rotp(self, quat, v):
        w = quat[0]
        bar = np.array([ [quat[1,0]], [quat[2,0]], [quat[3,0]] ])
        t = 2.0 * np.cross(v.T, bar.T)
        ret = v + w*t.T + np.cross(t, bar.T).T
        # set_trace()
        return ret


    def pose_callback(self, msg, mocap_id):

        # positionD1 = allPositions_[mocap_id]
        position = np.array([[msg.pose.position.x],
                             [msg.pose.position.y],
                             [msg.pose.position.z]])
        quatOrientation = np.array([[msg.pose.orientation.w],
                                    [msg.pose.orientation.x],
                                    [msg.pose.orientation.y],
                                    [msg.pose.orientation.z]])

        # print(position, type(position))
        # inXHat = np.block([[position], [np.zeros((9,1))]])
        if mocap_id not in self.allKalFilters_:
            self.allKalFilters_[mocap_id] = GenKalmanFilter(mocap_id, position)
            self.time_d1 = rospy.get_time()

        dt = rospy.get_time() - self.time_d1
        # if dt>1e-8:
        # print(dt)
        self.allKalFilters_[mocap_id].predict(dt)
        self.allKalFilters_[mocap_id].update_radar(position)
        self.time_d1 = rospy.get_time()

        xhat_array = self.allKalFilters_[mocap_id].xhat_
        P_mat = self.allKalFilters_[mocap_id].P_

        # velocity = xhat_array[3:6] #np.array([[msg.v[0]], [msg.v[1]], [msg.v[2]]])

        self.allPositions_[mocap_id] = xhat_array[0:3]
        self.allVelocities_[mocap_id] = xhat_array[3:6]
        self.allQuats_[mocap_id] = quatOrientation

        uncertaintyPos = [3.0*P_mat[0,0]**0.5, 3.0*P_mat[1,1]**0.5, 3.0*P_mat[2,2]**0.5]
        uncertaintyVel = [3.0*P_mat[3,3]**0.5, 3.0*P_mat[4,4]**0.5, 3.0*P_mat[5,5]**0.5]
        # uncertaintyPos[2] += 0.1
        self.allUncertaintyPos_[mocap_id] = uncertaintyPos
        self.allUncertaintyVel_[mocap_id] = uncertaintyVel
        # print(xhat)
        # print(dt)
        # print()
        # print(velocity)

        xhat_out = xhat()
        xhat_out.p = list(xhat_array[0:3].flatten())
        xhat_out.v = list(xhat_array[3:6].flatten())
        xhat_out.uncertaintyPos = uncertaintyPos
        xhat_out.uncertaintyVel = uncertaintyVel

        self.allPubs[mocap_id].publish(xhat_out)

    def add_sub(self, req):
        self.mocap_ids_.append(req.mocap_id)


        rospy.Subscriber(req.mocap_id, PoseStamped, self.pose_callback, (req.mocap_id))

        wpManager = WPManager(req.waypoints)
        self.allWPManagers_[req.mocap_id] = wpManager

        pub = rospy.Publisher(req.mocap_id+'_xhat', xhat, queue_size=10)
        self.allPubs[req.mocap_id] = pub

        return add_subscriberResponse(True)





    def get_velocity(self, req):
        av1Xo = copy.copy(self.allPositions_[req.mocap_id])
        av1Vo = copy.copy(self.allVelocities_[req.mocap_id])

        wpManager = self.allWPManagers_[req.mocap_id]
        av1VelDes = wpManager.updateWaypointManager(av1Xo)

        inRangePos = []
        inRangeVel = []
        inRangeUncertaintyPos = []
        inRangeUncertaintyVel = []

        inRangePos_msg = []
        inRangeVel_msg = []
        inRangeUncPos_msg = []
        inRangeUncVel_msg = []

        for mocap_id in self.allPositions_.keys():
            if req.mocap_id != mocap_id:
                # print(mocap_id, req.mocap_id)
                # print('here: ', self.allPositions_[mocap_id], " ", av1Xo)
                if norm(av1Xo - self.allPositions_[mocap_id]) < self.range:

                    # print(self.allPositions_[mocap_id], av1Xo)
                    # inRangePos.append(copy.copy(self.allPositions_[mocap_id]))
                    # inRangeVel.append(copy.copy(self.allVelocities_[mocap_id]))
                    #
                    # inRangeUncertaintyPos.append(self.allUncertaintyPos_[mocap_id])
                    # inRangeUncertaintyVel.append(self.allUncertaintyVel_[mocap_id])
                    inRangePos_1d = float32_1d()
                    inRangePos_1d.array = self.allPositions_[mocap_id]
                    inRangePos_msg.append(inRangePos_1d)

                    inRangeVel_1d = float32_1d()
                    inRangeVel_1d.array = self.allVelocities_[mocap_id]
                    inRangeVel_msg.append(inRangeVel_1d)

                    inRangeUncPos_1d = float32_1d()
                    inRangeUncPos_1d.array = self.allUncertaintyPos_[mocap_id]
                    inRangeUncPos_msg.append(inRangeUncPos_1d)

                    inRangeUncVel_1d = float32_1d()
                    inRangeUncVel_1d.array = self.allUncertaintyVel_[mocap_id]
                    inRangeUncVel_msg.append(inRangeUncVel_1d)

        # print(av1VelDes)
        # cvoGekko = CVOGekko(params)
        # sx, sy, sz = cvoGekko.get_best_vel(av1Xo, av1Vo, av1VelDes,
        #             inRangePos, inRangeVel,
        #             inRangeUncertaintyPos, inRangeUncertaintyVel)


        quat = self.allQuats_[req.mocap_id]
        # velCommand = copy.copy(np.array([[sx], [sy], [sz]]))
        # if req.mocap_id == '/cf5_enu':
        #     print(velCommand)
        # velCommand = copy.copy(av1VelDes)
        # velCommandBody = self.rotp(quat, velCommand)

        pose = Pose()
        point = Point()

        # print(velCommand)

        pose.position.x = av1Xo[0,0]
        pose.position.y = av1Xo[1,0]
        pose.position.z = av1Xo[2,0]

        pose.orientation.w = quat[0,0]
        pose.orientation.x = quat[1,0]
        pose.orientation.y = quat[2,0]
        pose.orientation.z = quat[3,0]

        # point.x = self.truncate(velCommandBody[0,0],3)
        # point.y = self.truncate(velCommandBody[1,0],3)
        # point.z = self.truncate(velCommandBody[2,0],3)




        return cvoResponse(pose,
                    av1Xo, av1Vo, av1VelDes,
                    inRangePos_msg, inRangeVel_msg,
                    inRangeUncPos_msg, inRangeUncVel_msg)


if __name__ == "__main__":
    rospy.init_node('cvo_server')
    cvoServer = CVOServer()

    rospy.spin()
