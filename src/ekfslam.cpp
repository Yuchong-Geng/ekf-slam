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
    float _motion_noise = 0.1,
    float _sensor_noise = 0.5){

    unsigned int l_size = landmark_size;
    unsigned int r_size = robot_pose_size;
    float motion_noise = _motion_noise;
    float sensor_noise = _sensor_noise;
    mu          = VectorXd::Zero(2*l_size + r_size, 1);
    Sigma = MatrixXd::Zero(r_size+2*l_size, r_size+2*l_size);
    robotSigma = MatrixXd::Zero(r_size, r_size);
    mapSigma = MatrixXd::Zero(2*l_size, 2*l_size);
    robMapSigma = MatrixXd::Zero(r_size, 2*l_size);

    //write those matrices into the big matrix
    Sigma.topLeftCorner(r_size, r_size) = robotSigma;
    Sigma.topRightCorner(r_size, 2*l_size) = robMapSigma;
    Sigma.bottomLeftCorner(2*l_size, r_size) = robMapSigma.transpose();
    Sigma.bottomRightCorner(2*l_size, 2*l_size) = mapSigma;

    //iniilize noise matrix:
    R = MatrixXd::Zero(r_size+2*l_size, r_size+2*l_size);
    R.topLeftCorner(r_size, r_size) << motion_noise, 0, 0,
                                       0, motion_noise, 0,
                                       0, 0, motion_noise;

     Q = MatrixXd::Zero(r_size+2*l_size, r_size+2*l_size);
     Q.topLeftCorner(r_size, r_size) << sensor_noise, 0, 0,
                                        0, sensor_noise, 0,
                                        0, 0, sensor_noise;


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
