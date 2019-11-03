#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Eigen;

int main() {

  Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
  q1.normalize();  q2.normalize();

  Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
  Vector3d p1(0.5, 0, 0.2);

  Isometry3d T1w(q1), T2w(q2);
  T1w.pretranslate(t1);
  T2w.pretranslate(t2);

  cout << "Transport T1: \n" << T1w.matrix() << endl;
  cout << "Transport T2: \n" << T2w.matrix() << endl;

  Vector3d p2 = T2w * T1w.inverse() * p1;
  cout << endl << p2.transpose() << endl;

  return 0;
}
