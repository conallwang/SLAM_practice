#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;

int main() {

  Matrix<double, 10, 10> bigMatrix;
  bigMatrix = Matrix<double, 10, 10>::Random();

  cout << "Origin Big Matrix: \n" << bigMatrix << endl;

  for (size_t i=0;i<3;i++) {
    for (size_t j=0;j<3;j++) {
      if (i == j)
        bigMatrix(i, j) = 1;
      else 
        bigMatrix(i, j) = 0;
    }
  }

  cout << "Changed Big Matrix: \n" << bigMatrix << endl;

  return 0;
}
