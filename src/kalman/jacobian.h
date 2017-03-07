#include <iostream>
#include <vector>
#include "Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd CalculateJacobian(const VectorXd& x_state);

MatrixXd CalculateJacobian(const VectorXd& x_state)
{

    MatrixXd Hj(3, 4);
    // recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    // pre-compute a set of terms to avoid repeated calculation
    double c1 = px * px + py * py;
    double c2 = sqrt(c1);
    double c3 = (c1 * c2);

    // check division by zero
    if (fabs(c1) < 0.0001)
    {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj << (px / c2), (py / c2), 0, 0, -(py / c1), (px / c1), 0, 0, py * (vx * py - vy * px) / c3,
        px * (px * vy - py * vx) / c3, px / c2, py / c2;

    return Hj;
}