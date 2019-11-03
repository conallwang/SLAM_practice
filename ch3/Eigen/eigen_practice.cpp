#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

int main() {

  // Quaterniond q1 and q2
  Quaterniond q1(0.35, 0.2, 0.3, 0.1);
  Quaterniond q2(-0.5, 0.4, -0.1, 0.2);
  q1.normalize();
  q2.normalize();

  // Angle Axisd a1, a2
  // AngleAxisd a1(q1);
  // AngleAxisd a2(q2);

  // Transport Matrix
  Isometry3d T1(q1), T2(q2);
  // T1.rotate(a1);
  // T2.rotate(a2);
  T1.pretranslate(Vector3d(0.3, 0.1, 0.1));
  T2.pretranslate(Vector3d(-0.1, 0.5, 0.3));

  cout << "Transport T1: \n" << T1.matrix() << endl;
  cout << "Transport T2: \n" << T2.matrix() << endl;

  Vector3d v(0.5, 0, 0.2);
  Vector3d v_transported;

  v_transported = T2 * T1.inverse() * v;
  cout << "(0.5, 0, 0.2) was transported to: " << v_transported.transpose() << endl;

  return 0;
}

