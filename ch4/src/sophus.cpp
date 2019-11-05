#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
using namespace Eigen;

int main() {

  // Rotation Matrix
  // Matrix3d R = AngleAxisd(M_PI/2, Vector3d(0, 0, 1)).matrix();
  Matrix3d R = AngleAxisd(M_PI/2, Vector3d(0, 0, 1)).toRotationMatrix();

  // Quaterniond
  Quaterniond q(R);

  // SO3 from rotation matrix and SO3 from quaterniond
  Sophus::SO3d SO3_R(R);
  Sophus::SO3d SO3_Q(q);

  cout << "SO3 from rotation matrix: \n" << SO3_R.matrix() << endl;
  cout << "SO3 from quaterniond: \n" << SO3_Q.matrix() << endl;

  // SO3 log
  Vector3d so3 = SO3_R.log();
  Vector3d so3_inverse = SO3_R.inverse().log();
 
  cout << "so3 = " << so3.transpose() << endl;
  cout << "so3_inverse = " << so3_inverse.transpose() << endl;

  // hat
  cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;

  // vee
  cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

  // Update
  Vector3d update_so3(1e-4, 0, 0);
  Sophus::SO3d SO3_Update = Sophus::SO3d::exp(update_so3) * SO3_R;
  
  cout << "SO3 Updated: \n" << SO3_Update.matrix() << endl;

  cout << "=================================================" << endl;

  // Translate Vector
  Vector3d t(1, 0, 0);
  // Construct SE3 from R,t and from q,t
  Sophus::SE3d SE3_Rt(R, t);
  Sophus::SE3d SE3_Qt(q, t);

  cout << "SE3 from R, t = \n" << SE3_Rt.matrix() << endl;
  cout << "SE3 from q, t = \n" << SE3_Qt.matrix() << endl;

  // SE3 inverse
  cout << "SE3 inverse: \n" << SE3_Rt.inverse().matrix() << endl;

  // check inverse
  cout << "Check SE3 inverse: \n" << SE3_Rt.inverse().matrix() * SE3_Rt.matrix() << endl;

  // SE3 log
  typedef Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  Vector6d se3_inverse = SE3_Rt.inverse().log();

  cout << "se3 = " << se3.transpose() << endl;
  cout << "se3_inverse = " << se3_inverse.transpose() << endl;   

  // hat
  cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
  // vee
  cout << "se3 hat vee = \n" << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

  // Update
  Vector6d update_se3;
  update_se3.setZero();
  update_se3(0, 0) = 1e-4;  

  Sophus::SE3d SE3_Update = Sophus::SE3d::exp(update_se3) * SE3_Rt;

  cout << "SE3 Updated: \n" << SE3_Update.matrix() << endl;



  return 0;
}
