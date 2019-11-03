#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

int main() {

  // Rotation Matrix
  Matrix3d rotation_matrix = Matrix3d::Identity();

  // Angle Axis    rotate 45 by Z axis
  AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));
  cout.precision(3);
  cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;

  rotation_matrix = rotation_vector.matrix();
  cout.precision(3);
  cout << "rotaion_matrix value = \n" << rotation_matrix << endl;
  
  // use Angle Axis to rotate
  // Vector3d v(0, 0, 1);	// prove the axis is (0, 0, 1)
  Vector3d v(1, 0, 0);
  Vector3d v_rotate = rotation_vector * v;
  
  // cout << "(0, 0, 1) rotated by Angle Axis: \n" << v_rotate << endl;
  cout << "(1, 0, 0) rotated by Angle Axis: \n" << v_rotate << endl;

  // use Rotate Matrix to rotate
  v_rotate = rotation_matrix * v;
  cout << "(1, 0, 0) rotated by Rotate Matrix: \n" << v_rotate << endl;
  
  // convert Rotation Matrix to Euler Angle (rpy angle)
  Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
  cout << "yaw pitch roll = \n" << euler_angles.transpose() << endl;
  
  // Transport Matrix ---- Eigen::Isometry
  Isometry3d T = Isometry3d::Identity();
  T.rotate(rotation_vector);		// rotate
  T.pretranslate(Vector3d(1, 3, 4));	// translate
 
  cout << "Transport Matrix: \n"  << T.matrix() << endl;

  // use Transport Matrix to change
  Vector3d v_transport = T * v;
  cout << "(1, 0, 0) transported by Transport Matrix: \n" << v_transport << endl;
  
  // Quaterniond using Angle Axis
  Quaterniond q = Quaterniond(rotation_vector);
  cout << "Quaterniond q using Angle Axis: " << q.coeffs().transpose() << endl;

  // Quaterniond using Rotation Matrix
  q = Quaterniond(rotation_matrix);
  cout << "Quaterniond q using Rotation Matrix: " << q.coeffs().transpose() << endl;

  // using q to rotate
  v_rotate = q * v;
  cout << "(1, 0, 0) rotated by Quaterniond: \n" << v_rotate << endl;

  // using normal math
  Quaterniond qv_rotate = q * Quaterniond(0, 1, 0, 0) * q.inverse();
  cout << "(1, 0, 0) rotated by normal math(Quaterniond): \n" << qv_rotate.coeffs().transpose() << endl;

  return 0;
}
