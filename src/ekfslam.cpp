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
    mu          = Eigen::VectorXd::Zero(2*l_size + r_size, 1);
    Sigma       = Eigen::MatrixXd::Zero(r_size+2*l_size, r_size+2*l_size);
    robotSigma  = Eigen::MatrixXd::Zero(r_size, r_size);
    mapSigma    = Eigen::MatrixXd::Zero(2*l_size, 2*l_size);
    robMapSigma = Eigen::MatrixXd::Zero(r_size, 2*l_size);

    //write those matrices into the big matrix
    Sigma.topLeftCorner(r_size, r_size) = robotSigma;
    Sigma.topRightCorner(r_size, 2*l_size) = robMapSigma;
    Sigma.bottomLeftCorner(2*l_size, r_size) = robMapSigma.transpose();
    Sigma.bottomRightCorner(2*l_size, 2*l_size) = mapSigma;

    //iniilize noise matrix:
    R = Eigen::MatrixXd::Zero(r_size+2*l_size, r_size+2*l_size);
    R.topLeftCorner(r_size, r_size) << motion_noise, 0, 0,
                                       0, motion_noise, 0,
                                       0, 0, motion_noise;

     Q = Eigen::MatrixXd::Zero(r_size+2*l_size, r_size+2*l_size);
     Q.topLeftCorner(r_size, r_size) << sensor_noise, 0, 0,
                                        0, sensor_noise, 0,
                                        0, 0, sensor_noise;

    //iniilize a vector to store info about if a certain landmark has been
    //before:
    observedLandmarks = Eigen::VectorXd::Zero(l_size, 1);


}

/****** TODO *********/
// Description: Prediction step for the EKF based off an odometry model
// Inputs:
// motion - struct with the control input for one time step
EKFSLAM::Prediction(const OdoReading& motion){

    double angle = mu(2);
    float r1 = motion.r1;
    float t  = motion.t;
    float r2 = motion.r2;
    //compute the Gtx matrix:
    Eigen::MatrixXd Gtx = MatrixXd::Zero(3, 3);
    Gtx << 1, 0, -t*sin(angle + r1),
           0, 1, t*cos(angle + r1),
           0, 0, 1;

    //update mu:
    mu(0) += t*cos(angle + r1);
    mu(1) += t*sin(angle + r1);
    mu(2) += r1 + r1;

    int cols = Sigma.col();
    //update  covariance matrix:
    Sigma.topLeftCorner(3, 3) = Gtx * Sigma.topLeftCorner(3, 3) * Gtx.transpose();
    Sigma.topRightCorner(3, cols - 3) = Gtx * Sigma.topRightCorner(3, cols -3);
    Sigma.bottomLeftCorner(cols - 3, 3) = Sigma.topRightCorner.transpose();
    //add motion noise to the covariance matrix:
    Sigma += R;
    return mu, Sigma;

}

/****** TODO *********/
// Description: Correction step for EKF
// Inputs:
// observation - vector containing all observed landmarks from a laser scanner
EKFSLAM::Correction(const vector<LaserReading>& observation){
    //iniilize data:
    unsigned int id;
    float range;
    float angle;
    float x;
    float y;
    for (const auto&one_observation: observation) {
      id = one_observation.id;
      range = one_observation.range;
      angle = one_observation.bearing;
      MatrixXd z = MatrixXd::Zero(2, 1);
      z << range,
           angle;

    }


}
