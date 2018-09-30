#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (0 == estimations.size() || estimations.size() != ground_truth.size()) {
    return rmse;
  }

  // Accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd res = estimations[i] - ground_truth[i];
    res = res.array() * res.array();
    rmse += res;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();

  // Calculate the squared root
  rmse = rmse.array().sqrt();

  // Return the result
  return rmse;
}