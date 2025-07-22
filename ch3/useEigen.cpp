#include <iostream>
using namespace std;

#include <ctime>

// Eigen core
#include <Eigen/Core>

// Albebraic operations of dense matrices(inverse, eigenvalues, etc....)
#include <Eigen/Dense>
using namespace Eigen;

#define MATRIX_SIZE 50

int main(){
    // Matrix<dtype, row, col> name;
    Matrix<float, 2, 3> matrix_23;

    // typedef -> many built-in types, bottom layer is Eigen::Matrix
    // ex) Vector3d  == Eigen::Matrix<double,3,1> 
    Vector3d v_3d;
    Matrix<float, 3, 1> vd_3d;

    // Matrix3d == Eigen::Matrix<double,3,3>
    Matrix3d matrix_33 = Matrix3d::Zero();

    // Dynamic allocation
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    // or
    MatrixXd matrix_x;

    // Operation of the Eigen matrix
    // initialize
    matrix_23 << 1,2,3,4,5,6;
    // print
    cout << "matrix 2x3 from 1to6 : \n" << matrix_23 << endl;

    // () == access elements in matrix
    cout << "print 2x3 matrix using () operator : " << endl;
    for (int i=0; i<2; i++){
        for (int j=0; j<3; j++){
            cout << matrix_23(i,j) << " \t";
        }
        cout << endl;
    }

    // Matrix * Vector (== Matrix * Matrix)
    // Eigen can't mix two different types of matrices matrix_23 * v_3d;
    // should be explicitly converted
    v_3d << 3,2,1;
    v_3d << 4,5,6;

    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << "[1,2,3,4,5,6] * [3,2,1] = " << result.transpose() << endl;

    Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << "[1,2,3,4,5,6] * [4,5,6] = " << result2.transpose() << endl;

    // can't misjudge the dimension of the matrix
    // Matrix<double, 2,3> result_wrong_dimension = matrix_23.cast<double> * v_3d;

    // some matrix operations
    // + - * /
    matrix_33 = Matrix3d::Random();
    cout << "random matrix : \n" << matrix_33 << endl;
    cout << "transpose : \n" << matrix_33.transpose() << endl;
    cout << "sum : " << matrix_33.sum() << endl;
    cout << "trace : " << matrix_33.trace() << endl;
    cout << "times 10 : \n" << 10* matrix_33 << endl;
    cout << "inverse : \n" << matrix_33.inverse() << endl;
    cout << "det: " << matrix_33.determinant() << endl;

    // eigen value
    // symmetric matrix -> diagonalization 
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose()* matrix_33);
    cout << "Eigen value : \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors: \n" << eigen_solver.eigenvectors() << endl;

    // Soving equations
    // matrix_NN * x = v_Nd, N -> previous macro, random num 
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    //semi-positive definite 
    matrix_NN = matrix_NN * matrix_NN.transpose(); 
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock();
    Matrix<double, MATRIX_SIZE,1> x = matrix_NN.inverse() * v_Nd;
    cout << "time of normal inverse is " << 1000*(clock() - time_stt)/ (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;


    // QR decomposition is faster
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time of Qr decomposition is" << 1000* (clock() * - time_stt)/ (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;
    return 0;

    // in Square matrix, cholesky decomposition
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "time of cholesky decomposition is" << 1000* (clock() * - time_stt)/ (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x = " << x.transpose() << endl;

    

}