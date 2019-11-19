#include <iostream>
#include <chrono>
#include "orb_utils.hpp"
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";

int main(int argc, char* argv[]) {
    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
    }

    printf("[INFO] IMAGE_PATH_1: %s\n", img_1.c_str());
    printf("[INFO] IMAGE_PATH_2: %s\n", img_2.c_str());

    // Open images 
    printf("[STATUS] Starting open images ... \n");

    cv::Mat image_1 = cv::imread(img_1, CV_LOAD_IMAGE_COLOR);
    cv::Mat image_2 = cv::imread(img_2, CV_LOAD_IMAGE_COLOR);

    if (image_1.data == nullptr || image_2.data == nullptr) {
        printf("[ERROR] Cannot open image. \n");
        printf("        IMAGE_1: %s\n", img_1.c_str());
        printf("        IMAGE_2: %s\n", img_2.c_str());
        printf("EXIT!\n");
        return 1;
    }

    printf("[STATUS] Open images finish. \n");

    // Feature points extracting
    printf("\n[STATUS] Starting extract feature points ... \n");
    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::FAST(image_1, keypoints_1, 40);
    cv::FAST(image_2, keypoints_2, 40);

    printf("[STATUS] Extract finish. \n");

    // Compute descriptors
    printf("\n[STATUS] Starting computing descriptor ... \n");
    cv::Mat descriptors_1, descriptors_2;

    ComputeORB(image_1, keypoints_1, descriptors_1);
    ComputeORB(image_2, keypoints_2, descriptors_2);

    chrono::steady_clock::time_point finish_time = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);

    printf("[STATUS] Computing descriptors finish. \n");
    cout << "[INFO] Time used: " << time_used.count() << " seconds" << endl;

    // Match
    vector<cv::DMatch> matches;
    Match(descriptors_1, descriptors_2, matches);

    // Good Match

    // show

}