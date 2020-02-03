# Kalman Filter 

### Abstract

Ensemble Kalman filteringwas developed as away to assimilate observed data to track the current state in a computational model. In this paper we showthat the ensemble approach makes possible an additional benefit: the timing of observations, whether they occur at the assimilation time or at some earlier or later time, can be effectively accounted for at low computational expense. In the case of linear dynamics, the technique is equivalent to instantaneously assimilating data as they are measured. The results of numerical tests of the technique on a simple model problem are shown.

#### Read Data File

Each row represents a sensor measurement where the first column tells you if the
measurement comes from radar (R) or lidar (L).

For a row containing radar data, the columns are: 
        sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth,vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.


For a row containing lidar data, the columns are:
    sensor_type, x_measured, y_measured,
    timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth,
    yaw_groundtruth, yawrate_groundtruth.
    
Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).
You will use the measurement values and timestamp in your Kalman filter algorithm.
Groundtruth, which represents the actual path the bicycle took, is for calculating root mean
squared error.
Yaw and yaw rate will be introduced in the UKF lecture (you will use the same data set for the
UKF project). You do not need to worry about yaw and yaw rate ground truth values.

The radar Cov matrix is:

    Radar<< 0.09, 0 , 0
    
0, 0.0009, 0
0, 0, 0.09
    
The lIdar cov matrix is:

    Lidar<< 0.0225, 0
0, 0225
