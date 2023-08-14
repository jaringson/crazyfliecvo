import rosbag
import rospy

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from IPython.core.debugger import set_trace
from importlib import reload

import animation
reload(animation)
from animation import Animation

def roll(q):
    return np.arctan2(2.0*(q[0]*q[1] + q[2]*q[3]), 1.0 - 2.0*(q[1]*q[1] + q[2]*q[2]))

def pitch(q):
    val = 2.0 * (q[0]*q[2] - q[1]*q[3])

    if np.abs(val) > 1.0:
        return np.copysign(1.0, val) * np.pi / 2.0
    else:
        return np.arcsin(val)

def yaw(q):
    return np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]))


bag = rosbag.Bag('../bags/test.bag')

topics_to_plot = ['/cf4_enu'] #'/cf1_enu', '/cf2_enu', '/cf3_enu', '/cf4_enu']
topics_xhat = ['/cf4_enu_xhat']

all_x = {}
all_y = {}
all_z = {}
all_qw = {}
all_qx = {}
all_qy = {}
all_qz = {}
all_phi = {}
all_theta = {}
all_psi = {}
all_t_pose = {}

all_xh = {}
all_yh = {}
all_zh = {}
all_vx = {}
all_vy = {}
all_vz = {}
all_ux = {}
all_uy = {}
all_uz = {}
all_uvx = {}
all_uvy = {}
all_uvz = {}
all_t_xhat = {}

for topic in topics_to_plot:
    x = np.array([])
    y = np.array([])
    z = np.array([])
    qw = np.array([])
    qx = np.array([])
    qy = np.array([])
    qz = np.array([])
    phi = np.array([])
    theta = np.array([])
    psi = np.array([])
    t_pose = []

    all_x[topic] = x
    all_y[topic] = y
    all_z[topic] = z
    all_qw[topic] = qw
    all_qx[topic] = qx
    all_qy[topic] = qy
    all_qz[topic] = qz
    all_phi[topic] = phi
    all_theta[topic] = theta
    all_psi[topic] = psi
    all_t_pose[topic] = t_pose

for topic in topics_xhat:
    xh = np.array([])
    yh = np.array([])
    zh = np.array([])
    vx = np.array([])
    vy = np.array([])
    vz = np.array([])

    ux = np.array([])
    uy = np.array([])
    uz = np.array([])
    uvx = np.array([])
    uvy = np.array([])
    uvz = np.array([])

    t_xhat = []

    all_xh[topic] = xh
    all_yh[topic] = yh
    all_zh[topic] = zh
    all_vx[topic] = vx
    all_vy[topic] = vy
    all_vz[topic] = vz

    all_ux[topic] = ux
    all_uy[topic] = uy
    all_uz[topic] = uz
    all_uvx[topic] = uvx
    all_uvy[topic] = uvy
    all_uvz[topic] = uvz

    all_t_xhat[topic] = t_xhat

# set_trace()
tstart = 0
init_tstart = False

for topic, msg, t in bag.read_messages():
    if(not init_tstart):
        init_tstart = True
        tstart = t.to_time()

    for topic_plot in topics_to_plot:
        if(topic==topic_plot):
            all_x[topic_plot] = np.append(all_x[topic_plot], msg.pose.position.x)
            all_y[topic_plot] = np.append(all_y[topic_plot], msg.pose.position.y)
            all_z[topic_plot] = np.append(all_z[topic_plot], msg.pose.position.z)
            all_qw[topic_plot] = np.append(all_qw[topic_plot], msg.pose.orientation.w)
            all_qx[topic_plot] = np.append(all_qx[topic_plot], msg.pose.orientation.x)
            all_qy[topic_plot] = np.append(all_qy[topic_plot], msg.pose.orientation.y)
            all_qz[topic_plot] = np.append(all_qz[topic_plot], msg.pose.orientation.z)
            q = [msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
            all_phi[topic_plot] = np.append(all_phi[topic_plot], roll(q))
            all_theta[topic_plot] = np.append(all_theta[topic_plot], pitch(q))
            all_psi[topic_plot] = np.append(all_psi[topic_plot], yaw(q))
            all_t_pose[topic_plot].append(t.to_time() - tstart)

    for topic_xhat in topics_xhat:
        if(topic==topic_xhat):
            all_xh[topic_xhat] = np.append(all_xh[topic_xhat], msg.p[0])
            all_yh[topic_xhat] = np.append(all_yh[topic_xhat], msg.p[1])
            all_zh[topic_xhat] = np.append(all_zh[topic_xhat], msg.p[2])
            all_vx[topic_xhat] = np.append(all_vx[topic_xhat], msg.v[0])
            all_vy[topic_xhat] = np.append(all_vy[topic_xhat], msg.v[1])
            all_vz[topic_xhat] = np.append(all_vz[topic_xhat], msg.v[2])

            all_ux[topic_xhat] = np.append(all_ux[topic_xhat], msg.uncertaintyPos[0])
            all_uy[topic_xhat] = np.append(all_uy[topic_xhat], msg.uncertaintyPos[1])
            all_uz[topic_xhat] = np.append(all_uz[topic_xhat], msg.uncertaintyPos[2])
            all_uvx[topic_xhat] = np.append(all_uvx[topic_xhat], msg.uncertaintyVel[0])
            all_uvy[topic_xhat] = np.append(all_uvy[topic_xhat], msg.uncertaintyVel[1])
            all_uvz[topic_xhat] = np.append(all_uvz[topic_xhat], msg.uncertaintyVel[2])

            all_t_xhat[topic_xhat].append(t.to_time() - tstart)

# set_trace()
cut_off = -50
for topic in topics_xhat:
    all_xh[topic] = all_xh[topic][0:cut_off]
    all_yh[topic] = all_yh[topic][0:cut_off]
    all_zh[topic] = all_zh[topic][0:cut_off]
    all_vx[topic] = all_vx[topic][0:cut_off]
    all_vy[topic] = all_vy[topic][0:cut_off]
    all_vz[topic] = all_vz[topic][0:cut_off]
    all_ux[topic] = all_ux[topic][0:cut_off]
    all_uy[topic] = all_uy[topic][0:cut_off]
    all_uz[topic] = all_uz[topic][0:cut_off]
    all_uvx[topic] = all_uvx[topic][0:cut_off]
    all_uvy[topic] = all_uvy[topic][0:cut_off]
    all_uvz[topic] = all_uvz[topic][0:cut_off]
    all_t_xhat[topic] = all_t_xhat[topic][0:cut_off]


fig, axs = plt.subplots(3, 1, sharex=True)
axs[0].legend(markerscale=2)
for topic in topics_to_plot:
    axs[0].grid()
    axs[0].scatter(all_t_pose[topic], all_x[topic], label=topic, s=2)
    axs[0].set_ylabel('$x$ (m)')

    axs[1].grid()
    axs[1].scatter(all_t_pose[topic], all_y[topic], label=topic, s=2)
    axs[1].set_ylabel('$y$ (m)')

    axs[2].grid()
    axs[2].scatter(all_t_pose[topic], all_z[topic], label=topic, s=2)
    axs[2].set_ylabel('$z$ (m)')
    axs[2].set_xlabel('t (s)', labelpad=-1)
    # axs[2].set_xlim(0,40)

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# for topic in topics_to_plot:
#     ax.plot(all_x[topic], all_y[topic], all_z[topic])

topic = topics_to_plot[0]
topic_xhat = topics_xhat[0]
fig, axs = plt.subplots(6, 1, sharex=True)
axs[0].grid()
axs[0].plot(all_t_pose[topic], all_x[topic], label=topic)
axs[0].plot(all_t_xhat[topic_xhat], all_xh[topic_xhat], label=topic_xhat)
axs[0].plot(all_t_xhat[topic_xhat], all_xh[topic_xhat]+all_ux[topic_xhat], color='g')
axs[0].plot(all_t_xhat[topic_xhat], all_xh[topic_xhat]-all_ux[topic_xhat], color='g')
axs[0].set_ylabel('$x$ (m)')
axs[0].legend()

axs[1].grid()
axs[1].plot(all_t_pose[topic], all_y[topic], label=topic)
axs[1].plot(all_t_xhat[topic_xhat], all_yh[topic_xhat], label=topic_xhat)
axs[1].plot(all_t_xhat[topic_xhat], all_yh[topic_xhat]+all_uy[topic_xhat], color='g')
axs[1].plot(all_t_xhat[topic_xhat], all_yh[topic_xhat]-all_uy[topic_xhat], color='g')
axs[1].set_ylabel('$y$ (m)')

axs[2].grid()
axs[2].plot(all_t_pose[topic], all_z[topic], label=topic)
axs[2].plot(all_t_xhat[topic_xhat], all_zh[topic_xhat], label=topic_xhat)
axs[2].plot(all_t_xhat[topic_xhat], all_zh[topic_xhat]+all_uz[topic_xhat], color='g')
axs[2].plot(all_t_xhat[topic_xhat], all_zh[topic_xhat]-all_uz[topic_xhat], color='g')
axs[2].set_ylabel('$z$ (m)')
axs[2].set_xlabel('t (s)', labelpad=-1)

axs[3].grid()
axs[3].plot(all_t_xhat[topic_xhat], all_vx[topic_xhat], label=topic_xhat)
axs[3].set_ylabel('$vx$ (m/s)')
axs[3].legend()

axs[4].grid()
axs[4].plot(all_t_xhat[topic_xhat], all_vy[topic_xhat], label=topic_xhat)
axs[4].plot(all_t_xhat[topic_xhat], all_vy[topic_xhat]+all_uvy[topic_xhat], color='g')
axs[4].plot(all_t_xhat[topic_xhat], all_vy[topic_xhat]-all_uvy[topic_xhat], color='g')
axs[4].set_ylabel('$vy$ (m/s)')

axs[5].grid()
axs[5].plot(all_t_xhat[topic_xhat], all_vz[topic_xhat], label=topic_xhat)
axs[5].set_ylabel('$vz$ (m/s)')
axs[5].set_xlabel('t (s)', labelpad=-1)

# numVehicles = len(topics_to_plot)
# animation = Animation(numVehicles)
# jump = 10
# # plt.waitforbuttonpress()
# for i in range(len(all_t_pose[topics_to_plot[0]])//jump):
#     # print(i*jump)
#
#     animation.drawAll(all_x, all_y, all_z, all_phi, all_theta, all_psi, topics_to_plot, i*jump)
#     # print(quadStates[i*jump]['v'][0])
#     # print(quadCommandedStates[i*jump]['v'][0])
#
#     plt.pause(0.001)
#     # set_trace()

plt.show()
