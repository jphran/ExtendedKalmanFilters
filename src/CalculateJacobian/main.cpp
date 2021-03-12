#include <iostream>
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

MatrixXd CalculateJacobian(const VectorXd& x_state);

int main() {
    // predicted state example
    // px = 1, py = 2, vx = 0.2, vy = 0.4
    VectorXd x_predicted(4);
    x_predicted << 1, 2, 0.2, 0.4;

    MatrixXd Hj = CalculateJacobian(x_predicted);

    cout << "Hj:" << endl << Hj << endl;

    return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3,4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    double rho_magnitude = sqrt(pow(px, 2) + pow(py, 2));

    // check division by zero
    if(fabs(rho_magnitude) < 1e-3) {
        cout << "Error - Division by zero" << endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj << px / rho_magnitude, py / rho_magnitude, 0, 0,
            -py / (pow(rho_magnitude,2)), px / (pow(rho_magnitude, 2)), 0, 0,
            (py * (vx*py - vy*px))/(pow(rho_magnitude,3)), (px*(vy*px - vx*py)) / (pow(rho_magnitude, 3)), px / rho_magnitude, py / rho_magnitude;

    return Hj;
}