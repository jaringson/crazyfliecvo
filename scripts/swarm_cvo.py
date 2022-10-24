import asyncio
import math
import time
import xml.etree.cElementTree as ET
from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

from cflib.positioning.motion_commander import MotionCommander

import time

from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from crazyflie.srv import *

send_full_pose = True

# Change uris and sequences according to your setup
URI1 = 'radio://0/80/2M/E7E7E7E701'
URI2 = 'radio://0/80/2M/E7E7E7E702'

URI3 = 'radio://0/80/2M/E7E7E7E703'
URI4 = 'radio://0/80/2M/E7E7E7E704'

waypoints1 = [
    1.0, 1.0, 1.0, 0,
    -1.0, -1.0, 1.0, 0,
    1.0, 1.0, 1.0, 0,
    -1.0, -1.0, 1.0, 0
]

# waypoints2 = [
#     0.33, -0.5, 1.0, 0,
#     0.33, 0.5, 1.0, 0,
#     0.33, -0.5, 1.0, 0,
#     0.33, 0.5, 1.0, 0
# ]
#
# waypoints3 = [
#     -0.33, -0.5, 1.0, 0,
#     -0.33, 0.5, 1.0, 0,
#     -0.33, -0.5, 1.0, 0,
#     -0.33, 0.5, 1.0, 0
# ]

waypoints2 = [
    1.0, -1.0, 1.0, 0,
    -1.0, 1.0, 1.0, 0,
    1.0, -1.0, 1.0, 0,
    -1.0, 1.0, 1.0, 0
]

cvo_args = {
    URI1: [waypoints1, '/xhat1'],
    # URI2: [waypoints2, '/cf2_enu'],
    # URI3: [waypoints3, '/cf3_enu'],
    # URI4: [waypoints4, '/cf4_enu'],
}

uris = {
    URI1,
    # URI2,
    # URI3,
    # URI4,
}

def wait_for_position_estimator(scf, mocap_id, cvo_service):
    print('Waiting for estimators to find position...')

    dt = 0.1

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:

            # print(mocap_id)
            resp_cvo = cvo_service(mocap_id, dt)
            # print(resp_cvo)

            pose = resp_cvo.pose
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            qw = pose.orientation.w
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            if send_full_pose:
                scf.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
            else:
                scf.cf.extpos.send_extpos(x, y, z)

            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break
    print('Done waiting')


def _sqrt(a):
    """
    There might be rounding errors making 'a' slightly negative.
    Make sure we don't throw an exception.
    """
    if a < 0.0:
        return 0.0
    return math.sqrt(a)




def reset_estimator(scf, waypoints, mocap_id, add_sub_service, cvo_service):
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')

    time.sleep(1)
    wait_for_position_estimator(scf, mocap_id, cvo_service)


def activate_kalman_estimator(scf):
    scf.cf.param.set_value('stabilizer.estimator', '2')

    # Set the std deviation for the quaternion data pushed into the
    # kalman filter. The default value seems to be a bit too low.
    scf.cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def activate_high_level_commander(scf):
    scf.cf.param.set_value('commander.enHighLevel', '1')


def activate_controller(scf):
    scf.cf.param.set_value('stabilizer.controller', '1')


def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(0.25)
    print('Parameters downloaded for', scf.cf.link_uri)

def start_kalman(cf, mocap_id, dt, cvo_service):
    for _ in range(5):
        resp_cvo = cvo_service(mocap_id, dt)
        # print(resp_cvo)

        pose = resp_cvo.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        qw = pose.orientation.w
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        if send_full_pose:
            cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
        else:
            cf.extpos.send_extpos(x, y, z)

        time.sleep(dt)
    print("Kalman Filter Primed")

def run_sequence(scf, waypoints, mocap_id, add_sub_service, cvo_service):
    try:
        dt = 0.1
        cf = scf.cf
        # resp_add = add_sub_service(mocap_id, waypoints)
        # Sleep to make sure subscriber has time to connect
        time.sleep(1)
        start_kalman(cf, mocap_id, dt, cvo_service)

        resp_cvo = cvo_service(mocap_id, dt)
        print(resp_cvo)


        end_time = time.time() + 1.0

        # while time.time() < end_time:
        #     resp_cvo = cvo_service(mocap_id, dt)
        #     print(resp_cvo)
        #
        #     pose = resp_cvo.pose
        #     x = pose.position.x
        #     y = pose.position.y
        #     z = pose.position.z
        #     qw = pose.orientation.w
        #     qx = pose.orientation.x
        #     qy = pose.orientation.y
        #     qz = pose.orientation.z
        #     if send_full_pose:
        #         cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
        #     else:
        #         cf.extpos.send_extpos(x, y, z)
        #     vx = 0.0 #2*resp_cvo.velCommand.x
        #     vy = 0.0 #2*resp_cvo.velCommand.y
        #     vz = 1.0*resp_cvo.velCommand.z
        #
        #     cf.commander.send_velocity_world_setpoint(vx,
        #                                         vy,
        #                                         vz, 0)
        #     time.sleep(0.1)

        with MotionCommander(scf) as mc:
            # mc.up(5.5)
            time.sleep(1)
            for _ in range(100):
                resp_cvo = cvo_service(mocap_id, dt)
                # print(resp_cvo)

                pose = resp_cvo.pose
                x = pose.position.x
                y = pose.position.y
                z = pose.position.z
                qw = pose.orientation.w
                qx = pose.orientation.x
                qy = pose.orientation.y
                qz = pose.orientation.z
                if send_full_pose:
                    cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
                else:
                    cf.extpos.send_extpos(x, y, z)

                vx = resp_cvo.velCommand.x
                vy = resp_cvo.velCommand.y
                vz = resp_cvo.velCommand.z
                # print(vx,vy,vz)
                # mc.start_linear_motion(0.0, 0.0, vz, 0.0)
                # mc.start_linear_motion(vx*0.5, vy*0.5, vz, 0.0)
                mc.start_linear_motion(vx, vy, vz, 0.0)

                # cf.commander.send_velocity_world_setpoint(vx,
                #                                     vy,
                #                                     vz, 0)

                # cf.commander.send_velocity_world_setpoint(0,
                #                                     0,
                #                                     vz, 0)
                # cf.commander.send_position_setpoint(0,0,0,0)
                time.sleep(dt)
            # And we can stop
            mc.stop()
            time.sleep(1)
    except Exception as e:
        print(e)


if __name__ == '__main__':
    rospy.init_node('swarm_cvo')
    add_sub_service = rospy.ServiceProxy('/add_subscriber', add_subscriber)
    cvo_service = rospy.ServiceProxy('/cvo', cvo)

    for key in cvo_args:
        # print(key)
        # print(cvo_args[key])
        cvo_args[key].append(add_sub_service)
        cvo_args[key].append(cvo_service)
        resp_add = add_sub_service(cvo_args[key][1], cvo_args[key][0])
        # print(cvo_args[key])

    # logging.basicConfig(level=logging.DEBUG)
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.sequential(activate_kalman_estimator)
        swarm.sequential(activate_high_level_commander)
        swarm.sequential(activate_controller)
        swarm.parallel(reset_estimator, args_dict=cvo_args)


        # print('Waiting for parameters to be downloaded...')
        # swarm.parallel(wait_for_param_download)

        swarm.parallel(run_sequence, args_dict=cvo_args)
