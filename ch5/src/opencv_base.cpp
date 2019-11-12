#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char* argv[]) {

  // Check if have args
  if (argc < 2) {
    cout << "USAGE: ./opencv_base IMAGE_PATH" << endl;
    return 1;
  }

  // Read Image
  cv::Mat image;
  image = cv::imread(argv[1]);

  if (!image.data) {
    cout << "Not Found: " << argv[1] << endl;
    return 1;
  }

  // Some basic arguments
  cout << "width: " << image.cols << endl;
  cout << "height: " << image.rows << endl;
  cout << "channels: "  << image.channels() << endl;

  // Show Image
  cv::imshow("image", image);
  cv::waitKey(0);
  
  // Check if U8C1(grey) or U8C3(rgb)
  if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
    cout << "Please input GREY or RGB image!" << endl;
    return 1;
  }

  // Timing
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

  // process each pixel
  for (size_t y=0;y<image.rows;y++) {
    // Get row_ptr
    unsigned char *row_ptr = image.ptr<unsigned char>(y);
    for (size_t x=0;x<image.cols;x++) {
      unsigned char *data_ptr = &row_ptr[x * image.channels()];
      
      for (size_t c=0;c<image.channels();c++) { 
        unsigned char data = data_ptr[c];
      }
    }
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

  // chrono::duration<double> can not output
  // cout << "Spend " << time << " seconds" << endl;
 
  cout << "Spend " << time.count() << " seconds" << endl; 

  // direct
  cv::Mat image_copy = image;		// image
  image_copy(cv::Rect(0, 0, 100, 100)).setTo(0);
  
  cv::imshow("image_copy", image_copy);
  cv::waitKey(0);

  // cv::Mat::clone
  image_copy = image.clone();
  image_copy(cv::Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("image2", image);
  cv::imshow("image_copy", image_copy);
  cv::waitKey();

  return 0;
}
