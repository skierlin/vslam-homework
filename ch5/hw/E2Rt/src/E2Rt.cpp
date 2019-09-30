//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullV | Eigen::ComputeFullU);
    Eigen::Vector3d x = svd.singularValues();
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();    

    double singular[3] = {x(0),x(1),x(2)};
    sort(singular, singular+3);
    x = Eigen::Vector3d((singular[1] + singular[2])/2, (singular[1] + singular[2])/2, 0);
    Eigen::Matrix3d sigma = x.asDiagonal();

    Sophus::SO3 Rz (0, 0, 0.5*3.1415926);
    Sophus::SO3 Rz1(0, 0,-0.5*3.1415926);
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1 = U * Rz.matrix() * sigma * U.transpose();
    Matrix3d t_wedge2 = U * Rz1.matrix() * sigma * U.transpose();

    Matrix3d R1 = U * Rz.matrix() * V.transpose();
    Matrix3d R2 = U * Rz1.matrix() * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}