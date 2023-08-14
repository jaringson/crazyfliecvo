import numpy as np

''' CVO '''
collision_radius = 0.2
collision_range = 3.0 #500


''' Gen Kalman Filter '''
sigmaQ_vel = 3
alphaQ_vel = 2.5
sigmaQ_jrk = 1.5 #0075
alphaQ_jrk = 1.5


sigmaR_range = 0.6/10
sigmaR_zenith =  0.01745/10
sigmaR_azimuth =  0.01745/10

radarPos = np.array([[0.0],
                    [-5.0],
                    [0.0]])


''' Waypoint Manager '''
Kp =  [0.5, 0.5, 0.5]
max_vel = 0.2
waypoint_threshold = 0.1
waypoint_velocity_threshold = 0.5
