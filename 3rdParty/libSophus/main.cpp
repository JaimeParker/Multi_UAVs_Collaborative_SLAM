#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

using namespace std;

int main() {
    // 沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Eigen::AngleAxisd v(M_PI / 2, Eigen::Vector3d(0, 0, 1)); // rotation vector seems to be useless

    Sophus::SO3d SO3_R(R);              // 直接从旋转矩阵构造
    Eigen::Quaterniond q(R);            // 或者四元数
    Sophus::SO3d SO3_q(q);
    cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
    cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
    cout << "they are equal" << endl;

    // log map getting Lie
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;
    // hat: vector to matrix
    Eigen::Matrix3d so3_hat = Sophus::SO3d::hat(so3);
    cout << "so3 hat=\n" << so3_hat << endl;
    // vee: matrix to vector
    cout << "so3 hat vee = " << Sophus::SO3d::vee(so3_hat).transpose() << endl;

    //


    return 0;
}


