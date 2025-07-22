#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

/*
W == world frame
^{target_frame}\textrm{T}_{source_frame} 
^{R1}\textrm{T}_{W} = [q1, t1, 0 ,1]^T
^{R2}\textrm{T}_{W} = [q2, t2, 0 ,1]^T

q_1 = [0.35, 0.2, 0.3, 0.1]^T
t_1 = [0.3, 0.1, 0.1]^T

q_2 = [-0.5,0.4,-0.1,0.2]^T
t_2 = [-0.1, 0.5, 0.3]^T

^{R_1}\textrm{P} = [0.5, 0, 0.2]^T

^{R_2}\textrm{P} = ??

-------------------------------

^{R_2}\textrm{P} = ^{R_2}\textrm{T}_{W} * (^{W}\textrm{T}_{R_1} * ^{R_1}\textrm{P})
= ^{R_2}\textrm{T}_{W} * ({q_1}^\ast * (^{R_1}\textrm{P} - t_1)
= q_2 * ({q_1}^\ast * (^{R_1}\textrm{P} - t_1) + t_2

^{W}\textrm{T}_{R_1} = ((^{R_1}\textrm{T}_{W})^{-1} = [q_1, t_1, 0 ,1]^T)^{-1} ==> inverse tranformation matrix

T = [R_n, t_n, 0, 1]^T = (R_n * p) + t_n ==> Rotation -> Translation
T^{-1} = [{R_n}^T, -{R_n}^T*t_n, 0, 1]^T = {R_n}^T * (p - t_n) ==> Translation(R_t) -> Rotation
Rotation == Quaternion => q->q*

*/

int main(int argc, char** argv){
    
    Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5,0.4,-0.1,0.2);
    // Should be unit quaternion q1,q2 -> |(q1+q2)|=1
    q1.normalize();
    q2.normalize();

    Translation3d t1(0.3,0.1,0.1), t2(-0.1,0.5,0.3);
    Vector3d p1(0.5,0,0.2);

    Isometry3d R1_T_W  = t1 * q1;
    Isometry3d R2_T_W = t2 * q2;
    

    Vector3d p2 = R2_T_W * R1_T_W.inverse() * p1;

    cout  << "\n p2 : " << p2.transpose() << endl; 

    
}