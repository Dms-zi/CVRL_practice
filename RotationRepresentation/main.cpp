#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;


/* Rotation2D
    [[ cos(theta), -sin(theta)],[sin(theta), cos(theta)]]
*/

int main(){

    double theta_degree;
    cout << "Angle (degrees) : ";
    cin >> theta_degree;
    
    double theta = theta_degree * M_PI / 180.0;

    // 2D Rotation

    // Angle == Rotation2D class (angle_)
    Rotation2D<double> R(theta); 
    cout << "Angle from ROtation2D class: \n" << R.angle() << endl;
    

    // Unit Complex Number z
    complex<double> z(cos(theta),sin(theta));
    cout << "Unit complex number: \n" << z << endl;


    // 1. Angle -> Rotation Matrix
    Matrix2d angle2Matrix = R.toRotationMatrix();
    cout << "Rotation Matrix from Angle:\n" << angle2Matrix << endl;


    // 2. Rotation Matrix -> Angle
    double matrix2Angle = atan2(angle2Matrix(1,0), angle2Matrix(0,0));
    cout << "Angle from Rotation Matrix:\n" << matrix2Angle << endl;


    // 3. Unit Complex Number -> Angle
    double hatz2Angle = atan2(z.real(), z.imag());
    cout << "Angle from Unit Complex Number: \n" << hatz2Angle << endl;
 

    // 4. Angle -> Unit Complex Number
    complex<double> angle2Hatz(cos(R.angle()), sin(R.angle()));
    cout << "Unit Complex Number from Angle:\n" << angle2Hatz << endl;


    // 5. Unit Complex Number -> Rotation Matrix
    Matrix2d hatz2Matrix;
    hatz2Matrix << z.real(), -z.imag(),
                  z.imag(),  z.real();
    cout << "Rotation Matrix from Unit Complex Number: \n" << hatz2Matrix << endl;


    // 6. Rotation Matrix -> Unit Complex Number
    complex<double> matrix2Hatz(hatz2Matrix(0,0),hatz2Matrix(1,1));
    cout << "Unit Complex Number from Rotation Matrix: \n" << matrix2Hatz << endl;

    // 3D Rotation

    // 7. Euler Angles -> Rotation Matrix
    // 8. Rotation Matrix -> Euler Angles
    // 9. Axis-Angle -> Rotation Matrix
    // 10. Rotation Matrix -> Axis Angle
    // 11. Axis-Angle -> Unit Quaternion
    // 12. Unit Quaternion -> Axis-Angle
    // 13. Unit Quaternion -> Rotation Matrix 
    // 14. Rotation Matrix -> Unit Quaternion
    // 15. Euler Angles -> Unit Quaternion
    // 16. Unit Quaternion -> Euler ANgles
    // 17. Euler Angles -> Axis-Angle
    // 18. Axis-Angle -> Euler Angles



}
