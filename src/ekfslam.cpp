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
    unsigned int robot_pose_size,
    float _motion_noise){

    unsigned int l_size = landmark_size;
    unsigned int r_size = robot_pose_size;
    float motion_noise = _motion_noise;

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
    R.topLeftCorner(2, 2) << motion_noise, 0,
                             0, motion_noise;

    Q = Eigen::MatrixXd::Zero(2, 2);
    Q << 0.8, 0,
          0,  0.8;
  //iniilize a vector to store info about if a certain landmark has been
  //before:
  // for (size_t i = 0; i < l_size; i++) {
  //   observedLandmarks.push_back(false);
  // }
  observedLandmarks.resize(l_size);
  fill(observedLandmarks.begin(), observedLandmarks.end(), false);

}

/**
* Destructor.
*/
EKFSLAM::~EKFSLAM() {}

/****** TODO *********/
// Description: Prediction step for the EKF based off an odometry model
// Inputs:
// motion - struct with the control input for one time step
void EKFSLAM::Prediction(const OdoReading& motion){

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
    mu(2) += r1 + r2;

    int cols = Sigma.cols();
    //update  covariance matrix:
    Sigma.topLeftCorner(3, 3) = Gtx * Sigma.topLeftCorner(3, 3) * Gtx.transpose();
    Sigma.topRightCorner(3, cols - 3) = Gtx * Sigma.topRightCorner(3, cols -3);
    Sigma.bottomLeftCorner(cols - 3, 3) = (Gtx*Sigma.topRightCorner(3, cols-3)).transpose();
    //add motion noise to the covariance matrix:
    Sigma += R;

}

/****** TODO *********/
// Description: Correction step for EKF
// Inputs:
// observation - vector containing all observed landmarks from a laser scanner
void EKFSLAM::Correction(const vector<LaserReading>& observation){
    //iniilize data:
    unsigned int id;
    double range;
    double angle;
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
    identity = Eigen::MatrixXd::Identity(2*l_size + 3, 2*l_size + 3);
    //iniilize z matrix
    Eigen::MatrixXd z, expectedZ;
    z = MatrixXd::Zero(2, 1);
    expectedZ = MatrixXd::Zero(2, 1);
    for (int i = 0; i < observation_size; i++) {
      auto& one_observation = observation[i];
      id = one_observation.id;
      range = one_observation.range;
      angle = one_observation.bearing;
      //normalize angle if out of range:
      if (angle > 3.14) {
        angle = -3.14 + angle - 3.14;
      }
      if (angle < -3.14) {
        angle = 3.14 + (angle + 3.14);
      }
      //record landmark location if never seen before
      if (!observedLandmarks.at(id - 1)) {
        observedLandmarks.at(id-1) = true;
        cor_mu(2*id + 1) = cor_mu(0) + range*cos(angle + mu(2)); //landmark x coordination
        cor_mu(2*id + 2) = cor_mu(1) + range*sin(angle + mu(2));
        // std::cout << "x " << mu(2*id + 1) << '\n';
        // std::cout << "y " << mu(2*id + 2) << '\n';

      }
      //add acutal readings to z matrix:
      z(0) = range;
      z(1) = angle;
      Eigen::MatrixXd delta;
      delta = Eigen::MatrixXd::Zero(2, 1);
      delta << cor_mu(2*id + 1) - cor_mu(0),
              cor_mu(2*id + 2) - cor_mu(1);
      // std::cout << "after delta" << '\n';
      double q;
      q = pow(delta(0), 2) + pow(delta(1), 2); //transpose of a matrix times a matrix is a number for this case
      // std::cout << "after q" << '\n';
      //add calculated value of reading to the expectedZ matrix:
      expectedZ(0) = sqrt(q);
      expectedZ(1) = atan2(delta(1), delta(0)) - cor_mu(2);
      //calculate H matrix:
      //iniilize F matrix first:
      Eigen::MatrixXd F;
      // std::cout << "at F" << '\n';
      F = Eigen::MatrixXd::Zero(5, 3 + 2*l_size);
      F.block<3,3>(0,0) << 1, 0, 0,
                           0, 1, 0,
                           0, 0, 1;
      F.block<2,2>(3, 1+2*id) << 1, 0,
                                 0, 1;
      Eigen::MatrixXd before_H;
      before_H = Eigen::MatrixXd::Zero(2, 5);
      before_H << -sqrt(q)*delta(0)/q, -sqrt(q)*delta(1)/q, 0, sqrt(q)*delta(0)/q, sqrt(q)*delta(1)/q,
                  delta(1)/q,          -delta(0)/q,        -1, -delta(1)/q,        delta(0)/q;
      H = before_H * F;
      // std::cout << "after H * F" << '\n';
      //kalman gain:
      Eigen::MatrixXd K;
      K = Eigen::MatrixXd::Zero(3+2*l_size, 2);
      K = Sigma * H.transpose() * (H*Sigma*(H.transpose()) + Q).inverse();
      // std::cout << "after K" << '\n';
      // std::cout << K.rows() << '\n';
      // std::cout << K.cols() << '\n';
      // std::cout << (z - expectedZ).rows() << '\n';
      // std::cout << (z - expectedZ).cols() << '\n';
      cor_mu += K * (z - expectedZ);
      //normalize angle if out of range:
      if (cor_mu(2)> 3.14) {
        cor_mu(2) = -3.14 + cor_mu(2) - 3.14;
      }
      if (cor_mu(2) < -3.14) {
        cor_mu(2) = 3.14 + (cor_mu(2) + 3.14);
      }

      cor_Sigma = (identity - K * H) * cor_Sigma;

    }

    //update:
    mu = cor_mu;
    Sigma = cor_Sigma;
}
