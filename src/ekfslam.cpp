#include "../include/sensor_info.h"
#include "../include/common.h"
#include "../include/Eigen/Dense"
#include "ekfslam.h"

/****** TODO *********/
// Overloaded Constructor
// Inputs:
// landmark_size - number of landmarks on the map
// robot_pose_size - number of state variables to track on the robot
// motion_noise - amount of noise to add due to motion
EKFSLAM::EKFSLAM(unsigned int landmark_size,
    unsigned int robot_pose_size = 3,
    float _motion_noise = 0.1){
    unsigned int l_size = landmark_size;
    unsigned int r_size = robot_pose_size;
    float noise = _motion_noise;
    mu          = VectorXd::Zero(2*l_size + r_size, 1);
    Sigma = MatrixXd::Zero(r_size+2*l_size, r_size+2*l_size);
    robotSigma = MatrixXd::Zero(r_size,r_size);
    mapSigma = MatrixXd::Zero(2*l_size, 2*l_size);



}

/****** TODO *********/
// Description: Prediction step for the EKF based off an odometry model
// Inputs:
// motion - struct with the control input for one time step
EKFSLAM::Prediction(const OdoReading& motion){

}

/****** TODO *********/
// Description: Correction step for EKF
// Inputs:
// observation - vector containing all observed landmarks from a laser scanner
EKFSLAM::Correction(const vector<LaserReading>& observation){

}
