#include <iostream>
#include <chrono>
#include <cstring>
#include <string>
#include <cmath>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

string image_file = "/home/johnson/distorted.png";

int main(int argc, char* argv[]) {

  if (argc > 2) {
    string tmp(argv[1]);
    image_file = tmp;
  }
  cout << "Process Image: " << image_file << endl;

  double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-5, k3 = 0;
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

  cv::Mat image = cv::imread(image_file, 0);
  
  // cv::imshow("image", image);
  // cv::waitKey(0);

  int rows = image.rows;
  int cols = image.cols;
  
  cv::Mat image_undis(rows, cols, CV_8UC1);

  for (size_t v=0;v<rows;v++) {
    for (size_t u=0;u<cols;u++) {
      // compute (u, v) in image and (u_dis, v_dis) in distorted
      double x = (u - cx) / fx;
      double y = (v - cy) / fy;
      double r = sqrt(x*x + y*y);

//      double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
//      double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
//      double u_distorted = fx * x_distorted + cx;
//      double v_distorted = fy * y_distorted + cy;
     
      double x_distorted = x * (1 + k1*pow(r, 2) + k2*pow(r, 4) + k3*pow(r, 6)) + 2*p1*x*y + p2*(pow(r, 2) + 2*pow(x, 2));
      double y_distorted = y * (1 + k1*pow(r, 2) + k2*pow(r, 4) + k3*pow(r, 6)) + p1*(pow(r, 2) + 2*pow(y, 2)) + 2*p2*x*y;

      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;
 
      if (u_distorted >=0 && v_distorted >=0 && u_distorted < cols && v_distorted < rows)
        image_undis.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
      else
        image_undis.at<uchar>(v, u) = 0;
    }
  }

  cv::imshow("image_undistorted", image_undis);
  cv::imshow("image_distorted", image);
  cv::waitKey(0);

  return 0;
}
