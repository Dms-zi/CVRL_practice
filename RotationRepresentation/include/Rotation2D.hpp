#ifndef __ROTATION2D_HPP__
#define __ROTATION2D_HPP__

#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

class Rotation2D{

public:
    Matrix2d angle2Matrix(const double Angle);
    double matrix2Angle(const Matrix2d& R);
    Matrix2d complex2Matrix(const comeplex& z)
    complex<double> matrix2Complex(const Matrix2d& R)
    complex<double> angle2Complex(const double Angle);
    double complex2Angle(const comeplex& z);

protected:

}