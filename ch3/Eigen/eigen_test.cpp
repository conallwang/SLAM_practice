#include <iostream>
using namespace std;

#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;

int main() {
  
  // Matrix 3x2
  Matrix<float, 3, 2> Matrix_32;
  Matrix_32 << 1, 2, 3, 4, 5, 6;
  cout << "Matrix 3x2: \n" << Matrix_32 << endl; 

  // Matrix 2x3
  Matrix<int, 2, 3> Matrix_23;
  Matrix_23 << 1, 2, 3, 6, 5, 4;
  cout << "Matrix 2x3: \n" << Matrix_23 << endl;

  // Product, using Dynamic
  Matrix<float, Dynamic, Dynamic> Matrix_res = Matrix_32 * Matrix_23.cast<float>();
  cout << "The Product of them: \n" << Matrix_res << endl;
  
  // Random Matrix
  Matrix3d Matrix_33 = Matrix3d::Random();
  cout << "Random Matrix: \n" << Matrix_33 << endl;

  // transpose of 3x2
  cout << "transpose of Matrix 3x2: \n" << Matrix_32.transpose() << endl;

  // sum of 2x3
  cout << "sum of Matrix 2x3 \n" << Matrix_23.sum() << endl;

  // trace of Matrix 3x2
  cout << "trace of Matrix 3x2 \n" << Matrix_32.trace() << endl;

  // inverse of Matrix 3x3
  cout << "inverse of Matrix 3x3 \n" << Matrix_33.inverse() << endl;

  // determinant of Matrix 3x3
  cout << "determinant of Matrix 3x3 \n" << Matrix_33.determinant() << endl;

  // Sovle Func
  Matrix<double, 50, 50> Matrix_NN = MatrixXd::Random(50, 50);
  Matrix_NN = Matrix_NN * Matrix_NN.transpose();
  Matrix<double, 50, 1> v_Nd = MatrixXd::Random(50, 1);

  clock_t time_stt = clock();
  Matrix<double, 50, 1> x = Matrix_NN.inverse() * v_Nd;
  cout << "time of normal inverse is: " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  time_stt = clock();
  x = Matrix_NN.colPivHouseholderQr().solve(v_Nd);
  cout << "time of Qr decomposition is: " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  time_stt = clock();
  x = Matrix_NN.ldlt().solve(v_Nd);
  cout << "time of ldlt decomposition is: " << 1000 * (clock() - time_stt) / (double CLOCKS_PER_SEC) << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  return 0;
}

