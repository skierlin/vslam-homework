//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream fs1, fs2;
    fs1.open(p2d_file); 
    if(!fs1)
    {
        cout << "input file 1 err!" << endl;
    }
    while(!fs1.eof())
    {
        Vector2d p2;
        fs1 >> p2(0) >> p2(1);
        p2d.push_back(p2);
    }

    fs2.open(p3d_file);
    if(!fs2)
    {
        cout << "input file 2 err!" << endl;
    }
    while(!fs2.eof())
    {
        Vector3d p3;
        fs2 >> p3(0) >> p3(1) >> p3(2);
        p3d.push_back(p3);
    }        
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Vector2d residual;
            Matrix<double, 4, 1> p3dd;
            p3dd << p3d[i](0), p3d[i](1), p3d[i](2), 1;
            Vector3d p3d_ = (T_esti.matrix() * p3dd).topRows(3);//p_c
            residual(0) = p2d[i](0) - (K * (T_esti.matrix() * p3dd).topRows(3))(0) / p3d_(2);
            residual(1) = p2d[i](1) - (K * (T_esti.matrix() * p3dd).topRows(3))(1) / p3d_(2);
            cost += 0.5 * sqrt(residual(0) * residual(0) + residual(1) * residual(1));
	        // END YOUR CODE HERE

	        // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            J(0, 0) = -fx / p3d_(2);
            J(0, 1) = 0;
            J(0, 2) = (fx * p3d_(0) / p3d_(2) / p3d_(2));
            J(0, 3) = (fx * p3d_(0) * p3d_(1) / p3d_(2) / p3d_(2));
            J(0, 4) = -fx - (fx * p3d_(0) * p3d_(0) / p3d_(2) / p3d_(2));
            J(0, 5) = (fx * p3d_(1) / p3d_(2));
            J(1, 0) = 0;
            J(1, 1) = -1 * fy / p3d_(2);
            J(1, 2) = (fy * p3d_(1) / p3d_(2) / p3d_(2));
            J(1, 3) = fy + (fy * p3d_(1) * p3d_(1) / p3d_(2) / p3d_(2));
            J(1, 4) = -1 * (fy * p3d_(0) * p3d_(1) / p3d_(2) / p3d_(2));
            J(1, 5) = -1 * (fy * p3d_(0) / p3d_(2));
	        // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * residual;
        }

	    // solve dx 
        Vector6d dx;

        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3::exp(dx) * T_esti;
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
