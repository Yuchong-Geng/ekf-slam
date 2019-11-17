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

    Q = Eigen::MatrixXd::Zero(2, 2);
    Q << 0.5, 0,
          0,  0.5;

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
    double range;
    double angle;
    double x;
    double y;
    int observation_size;
    observation_size = observation.size();
    Eigen::MatrixXd H;
    int l_size;
    l_size = observedLandmarks.size();
    H = Eigen::MatrixXd::Zero(2, 2*l_size + 3);
    Eigen::MatrixXd cor_Sigma, cor_mu;
    cor_Sigma = Eigen::MatrixXd::Zero(2*l_size + 3, 2*l_size + 3);
    cor_mu = Eigen::MatrixXd::Zero(3+2*l_size, 1);
    cor_mu = mu;
    cor_Sigma = Sigma;
    //identity matrix for updating covariance matrix Sigma:
    Eigen::MatrixXd identity;
    identity = Eigen::MatrixXd::identity(2*l_size + 3, 2*l_size + 3);
    //iniilize z matrix
    Eigen::MatrixXd z, expectedZ;
    z = MatrixXd::Zero(3+2*l_size, 1);
    expectedZ = MatrixXd::Zero(3+2*l_size, 1);
    for (size_t i = 0; i < observation_size; i++) {
      auto& one_observation;
      one_observation = observation[i];
      id = one_observation.id;
      range = one_observation.range;
      angle = one_observation.bearing;
      //record landmark location if never seen before
      if (observedLandmarks(id - 1) == 0) {
        observedLandmarks(id-1) = 1;
        mu(2*id + 1) = mu(0) + range*cos(angle + mu(2)); //landmark x coordination
        mu(2*id + 2) = mu(0) + range*sin(angle + mu(2));
      }
      //add acutal readings to z matrix:
      z(2*i) = range;
      z(2*i+1) = angle;

      Eigen::MatrixXd delta;
      delta << mu(2*id + 1) - mu(0),
              mu(2*id + 2) - mu(1);
      double q;
      q = delta.transpose()*delta; //transpose of a matrix times a matrix is a number for this case
      //add calculated value of reading to the expectedZ matrix:
      expectedZ(2*i) = sqrt(q);
      expectedZ(2*i+1) = atan(delta(1), delta(0)) - mu(2);
      //calculate H matrix:
      //iniilize F matrix first:
      Eigen::MatrixXd F;
      F = Eigen::MatrixXd::Zero(5, 3 + 2*l_size);
      F.block<3,3>(0,0) << 1, 0, 0,
                           0, 1, 0,
                           0, 0, 1;
      F.block<2,2>(3, 1+2*id) << 1, 0,
                                 0, 1;
      Eigen::MatrixXd before_H;
      before_H << -sqrt(q)*delta(0)/q, -sqrt(q)*delta(1)/q, 0, sqrt(q)*delta(0)/q, sqrt(q)*delta(1)/q,
                  delta(1)/q,          -delta(0)/q,        -1, -delta(1)/q,        delta(0)/q;
      H = before_H * F;
      //kalman gain:
      Eigen::MatrixXd K;
      K = Eigen::MatrixXd::Zero(3+2*l_size, 2);
      K = Sigma * H.transpose() * (H*Sigma*H.transpose() + Q).inverse();
      cor_mu += K * (z - expectedZ);
      cor_Sigma = (identity - K * H) * cor_Sigma;

    }

    //update:
    mu = cor_mu;
    Sigma = cor_Sigma;
}
