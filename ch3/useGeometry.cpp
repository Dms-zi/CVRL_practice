#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

/*

# Eigen module provides a variety of rotation and translation
<Eigen/Core> -> base operation for matrix, vector
<Eigen/Dense> -> Core + SVD,LU,QR ...
<Eigen/Geometry> -> Core + Transformation, Quaternion..

# Core Class
1. Matrix -> 2d Array (Matrix3d A = Matrix3d::Identity();)
    - Matrix<float, 2, 2> : 2x2 float matrix
    - MatrixXd : Dynamic double matrix
2. Vector -> column vector, not Rotation values (Matrix<N,1>, Vector3d v(1,2,3);)
    - Vector3d : 3 dimension
3. RowVector -> Row vector (Matrix<1,N>, RowVector3d r;)
4. Array -> element wise operation (ArrayXXf arr;)

# Geometry Class
1. Quarternion (Quarterniond q(x,y,z,w);)
2. AngleAxis (AngleAxisd aa;)
3. Transform -> rotation + translation (Transform<dtype,dimension,Mode(Affine,Isometry, Projective, AffineCompact))
- Rotation(Matrix3d, Quterniond, AngleAxisd), Translation(Translation class)
typedef
- Isometry3d == Transform<double, 3, Isometry>; -> not scaling, rotation + translation (4x4)
-  Affine3d == Transform<double, 3, Affine>; -> scaling + rotation + translation (4x4)
-  Projective3d == Transform<double, 3, Projective>; -> projection + rotation + translation (4x4)
4. Translation (Transform<dtype,dimension>)
typedef
- Translatin3d == Transform<double, 3>; -> translation + homogeneous coordinate(4x4)

# Decomposition Class
1. JacobiSVD (JacobiSVD<MatrixXd> svd(a);)
2. EigenSolver (EigenSolver<MatrixXd> es(A);)
3. LLT -> cholesky decomposition (LLT< MatrixXd> llt(A);)

# Map class
- stadard list, array in C,C++ -> Eigen mapping
1. Map (Map<Vector3d> v(data);)

# Table of Contents
1. Rotation Matrix(Matrix3d)
2. Rotation Vector(AxisAngled)
- Rotation Vector -> Rotation Matrix
- Coordinate Transformation
3. Euler Angle (Vector3d)
- Roll pitch yaw (order sequence -> index)
- Rotation Matrix -> Euler Angle
4. Euclidean Transformation -> Isometry
- rotation -> pretranslation
- translation3d * AxisAngled -> T(R(p))
5. Quaternion
- Q -> AxisAngle
*/
int main(int argc, char **argv){
    //3d rotation matrix = Matrix3D or Matrix 3f
    Matrix3d rotation_matrix = Matrix3d::Identity();

    //Axis-Angle = AngleAxisd, operator overloading -> can be treated as a matrix operation
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0,0,1) ); // angle == (pi/4), axis == z

    cout.precision(3);

    // Rotation Vector -> Rotation Matrix
    // 1. return Rotation Matrix in rotation_vector object
    cout << "Rotation Matrix(vector > matrix) =\n" << rotation_vector.matrix() << endl;
    // 2. convert Rotation Vector (Axis-angle, SO(3) log) to matrix
    rotation_matrix = rotation_vector.toRotationMatrix();

    // Coordinate Transformation with Axis-Angle, |v|=1
    Vector3d v(1,0,0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation by Axis-angle = " << v_rotated.transpose() << endl;

    // Coordinate Transformation with Rotation Matrix
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation by Rotation Matrix = " << v_rotated.transpose() << endl;


    // Rotation Matrix -> Euler Angle
    // ZYX order == roll pitch yaw , index (x==0, y==1, z==2)
    Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);  
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;


    // Euclidean Transformation Matrix T using Eigen::Isometry, 4*4
    // 1. rotation -> translation
    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Vector3d(1,3,4)); 
    cout << "Transform Matrix = \n" << T.matrix() << endl; // .matrix() -> convert object(T) to Matrix4d

    // 2. one line, T = [R,t,0,1]^T
    // AngleAxisd(3x1) promote 4x4 homogeneous transform when multiplicate with Translation3d
    Isometry3d T2 = Translation3d(1,3,4) * AngleAxisd(M_PI/4, Vector3d(0,0,1));
    cout << "Transfom Matrix, oneline = \n" << T2.matrix() << endl;


    // Quaternion
    // AxisAngle -> q
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "Vector in q : " << q.vec() << endl;
    cout << "Real number w in q : " << q.w() << endl;
    cout << "Quarternion from rotation vector (column vector): " << q.coeffs() << endl;
    cout << "Quarternion from rotation vector : " << q.coeffs().transpose() << endl;

    // Rotated point p = qpq*
    v_rotated = q * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() <<"  == qpq* = " << (q * Quaterniond(0,1,0,0) * q.inverse()).coeffs().transpose() << endl;


    return 0;





}