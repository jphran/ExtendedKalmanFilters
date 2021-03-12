#include <iostream>
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth);

int main() {
    /**
     * Compute RMSE
     */
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // the input list of estimations
    VectorXd e(4);
    e << 1, 1, 0.2, 0.1;
    estimations.push_back(e);
    e << 2, 2, 0.3, 0.2;
    estimations.push_back(e);
    e << 3, 3, 0.4, 0.3;
    estimations.push_back(e);

    // the corresponding list of ground truth values
    VectorXd g(4);
    g << 1.1, 1.1, 0.3, 0.2;
    ground_truth.push_back(g);
    g << 2.1, 2.1, 0.4, 0.3;
    ground_truth.push_back(g);
    g << 3.1, 3.1, 0.5, 0.4;
    ground_truth.push_back(g);

    // call the CalculateRMSE and print out the result
    cout << CalculateRMSE(estimations, ground_truth) << endl;

    return 0;
}

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the inputs:
    if(estimations.empty() || estimations.size() != ground_truth.size()) {
        cout << "Estimation vector is either empty or not the same size as the ground truth vector" << endl;
        return rmse;
    }

    VectorXd squared_residuals(estimations[0].size());
    for (int i=0; i < estimations.size(); ++i) {
        squared_residuals = squared_residuals.array() +
                ((estimations[i] - ground_truth[i]).array() * (estimations[i] - ground_truth[i]).array());
    }

    // calculate the mean
    VectorXd mean = squared_residuals.array() / estimations.size();

    // calculate the squared root
    rmse = mean.array().sqrt();

    // return the result
    return rmse;
}