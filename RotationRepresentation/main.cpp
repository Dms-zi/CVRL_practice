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
    Isometry3d T = Isometry3d::Identity();
    T.rotate(AngleAxisd(theta, Vector3d(0,0,1))); //or Vector3d(UnitZ);


    // 8. Rotation Matrix -> Euler Angles
    Matrix3d R_matrix = T.rotation();
    Vector3d matrix2Euler = R_matrix.eulerAngles(2,1,0);
    cout << "Euler from Rotation Matrix : \n" << R_matrix.transpose() << endl;


    // 10. Rotation Matrix -> Axis Angle
    AngleAxisd matrix2AxisAngle = AngleAxisd(R_matrix);
    cout << "AxisAngle from Matri(axis): \n" << matrix2AxisAngle.axis().transpose() << endl;
    cout << "AxisAngle from Matri(angle): \n" << matrix2AxisAngle.angle() << endl;


    // 14. Rotation Matrix -> Unit Quaternion
    Quaterniond matrix2Quaternion(R_matrix);
    cout << "Unit Quaternion from Rotation Matrix: \n" << matrix2Quaternion.coeffs().transpose() << endl;


    // 9. Axis-Angle -> Rotation Matrix
    Matrix3d axisangle2Matrix = matrix2AxisAngle.toRotationMatrix();
    cout << "Rotation Matrix from AngleAxis : \n " << axisangle2Matrix << endl;
    

    // 11. Axis-Angle -> Unit Quaternion
    Quaterniond axisangle2Quaternion(matrix2AxisAngle); 
    cout << "Unit Quaternion from Axis-Angle: \n" << axisangle2Quaternion.coeffs().transpose() << endl;


    // 18. Axis-Angle -> Euler Angles
    // convert AxisAngle to Rotation Matrix first
    Vector3d axisangle2Euler = matrix2AxisAngle.toRotationMatrix().eulerAngles(2,1,0);
    cout << "Euler Angles from Axis-Angle: \n" << axisangle2Euler.transpose() << endl;


    // 12. Unit Quaternion -> Axis-Angle
    AngleAxisd quaternion2AxisAngle(axisangle2Quaternion);
    cout << "Axis-Angle from Unit Quaternion(Axis) : \n" << quaternion2AxisAngle.axis().transpose() << endl;
    cout << "Axis-Angle from Unit Quaternion(Angle) : \n" << quaternion2AxisAngle.angle() << endl;


    // 13. Unit Quaternion -> Rotation Matrix 
    Matrix3d quaternion2Matrix = axisangle2Quaternion.toRotationMatrix();
    cout << "Rotation Matrix from Unit Quaternion: \n" << quaternion2Matrix.transpose() << endl;


    // 16. Unit Quaternion -> Euler ANgles
    // convert Quaternion to Rotation Matrix first
    Vector3d quaternion2Euler = axisangle2Quaternion.toRotationMatrix().eulerAngles(2,1,0);
    cout << "Euler Angle from Unit Quaternion: \n " << quaternion2Euler.transpose() << endl;


    // 7. Euler Angles -> Rotation Matrix
    // Vector3d(Euler Angle) has not .matrix()
    Matrix3d euler2Matrix;
    euler2Matrix = AngleAxisd(matrix2Euler(0), Vector3d::UnitZ()) *
                   AngleAxisd(matrix2Euler(1), Vector3d::UnitY()) *
                   AngleAxisd(matrix2Euler(2), Vector3d::UnitX());
    cout << "Matrix from EulerAngles : \n" << euler2Matrix.transpose() << endl;


    // 15. Euler Angles -> Unit Quaternion
    // convert Euler Angles to Rotation Matrix first
    Quaterniond euler2Quaternion(euler2Matrix);
    cout << "Unit Quaternion from Euler Angles: \n" << euler2Quaternion.coeffs().transpose() << endl;


    // 17. Euler Angles -> Axis-Angle
    // convert Euler Angles to Rotation Matrix first
    AngleAxisd euler2AxisAngle(euler2Matrix);
    cout << "Axis Angle from Euler Angles: \n" << euler2AxisAngle.axis().transpose() << endl;
    cout << "Axis Angle from Euler Angles: \n" << euler2AxisAngle.angle() << endl;

    return 0;
}
