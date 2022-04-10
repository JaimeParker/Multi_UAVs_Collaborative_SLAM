#include <iostream>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
// There would be so many Eigen::function, can we use an alia instead, like import numpy as np in python?
// here is the solution
//https://www.geeksforgeeks.org/how-to-use-namespace-alias-qualifier-in-c-sharp/
// However, it doesn't seem to work out

#define MATRIX_SIZE 50

int main() {
    // define a matrix of float elements, 2 rows and 3 cols
    Eigen::Matrix<float, 2, 3> matrix_23;

    matrix_23 << 1, 2, 3, 4, 5, 6;  // send data to matrix_23
    cout << matrix_23 << endl;

    // define a vector of double elements
    Eigen::Vector3d V_3d;
    V_3d << 1, 2, 3;
    // have to cast the type of variable manually
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * V_3d;
    cout << result << endl;

    // using Matrix3d and solving eigen value
    Eigen::Matrix<double, 3, 3> matrix_33 = Eigen::Matrix3d::Zero();
    matrix_33 = Eigen::Matrix3d::Random();
    cout << "matrix_33 =" << endl;
    cout << matrix_33 << endl;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver (matrix_33);
    cout << "Eigen values ="  << endl << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors =" << endl << eigen_solver.eigenvectors() << endl;

    // calculating the equation: matrix_NN * x = V_Nd
    // inverse matrix or divide the matrix
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock();
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time in normal inverse is:" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;

    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time in Qr is" << 1000 * (clock() - time_stt) / (double)CLOCKS_PER_SEC << "ms" << endl;

    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity(); // Identity
    // use AngleAxis to compose a rotation vector
    Eigen::AngleAxisd rotation_vector (M_PI / 4, Eigen::Vector3d(0, 0, 1)); // axis Z, PI/4 rad
    cout .precision(3);
    cout << "rotation matrix = \n" << rotation_vector.matrix() << endl; // turning vector to matrix using matrix()
    // toRotationMatrix: returns an equivalent 3x3 matrix
    rotation_matrix = rotation_vector.toRotationMatrix();

    Eigen::Vector3d v (1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1, 0, 0) after rotation:" << v_rotated.transpose() << endl;

    v_rotated = rotation_matrix * v;
    cout << "(1, 0, 0) after rotation:" << v_rotated.transpose() << endl;

    // Euler transfer and matrix
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // 2,1,0 means ZYX, namely yaw-pitch-roll
    cout << "yaw-pitch-roll=" << euler_angles.transpose() << endl;

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.rotate(rotation_matrix); // use rotation vector or matrix
    T.pretranslate(Eigen::Vector3d (1, 2, 3));
    cout << "Transformation Matrix = \n" << T.matrix() << endl;

    Eigen::Vector3d v_transformed = T * v;
    cout << "v_transformed = " << v_transformed.transpose() << endl;

    // Quaternion
    Eigen::Quaterniond q = Eigen::Quaterniond (rotation_vector);
    cout << "Quaternion(double) = \n" << q.coeffs() << endl;
    // also use rotation matrix
    q = Eigen::Quaterniond (rotation_matrix);
    cout << "Quaternion(double) = \n" << q.coeffs() << endl;

    v_rotated = q * v; // actually qvq^{-1} in formula
    cout << "(1, 0, 0) after rotation" << v_rotated.transpose() << endl;

    return 0;
}
