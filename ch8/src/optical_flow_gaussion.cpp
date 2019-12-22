#include "flow_utils.hpp"
#include <orb_utils.hpp>
#include <string.h>
#include <cstring>

string img_1 = "/home/johnson/SLAM/ch8/pics/LK1.png";
string img_2 = "/home/johnson/SLAM/ch8/pics/LK2.png";



void CalculateOpticalFlowSingleLevel(cv::Mat image_1, cv::Mat image_2, vector<mKeyPoint> keypoints_1, vector<mKeyPoint>& keypoints_2, 
                                    vector<bool>& success, bool inverse = false, int iter_num = 100, int half_patch_size = 4) {
    keypoints_2.resize(keypoints_1.size());
    success.resize(keypoints_1.size());
    OpticalFlow tracker(image_1, image_2, keypoints_1, keypoints_2, success, inverse, iter_num, half_patch_size);
    cv::parallel_for_(cv::Range(0, keypoints_1.size()),
                std::bind(&OpticalFlow::CalculateOpticalFlow, &tracker, placeholders::_1));
    keypoints_2 = tracker.GetResult();
}

void ShowFlow(string gname, cv::Mat image, vector<mKeyPoint> points_1, vector<mKeyPoint> points_2, int radius = 2,
    cv::Scalar color_1 = cv::Scalar(0, 0, 0), cv::Scalar color_2 = cv::Scalar(255, 255, 255)) {
    if (points_1.size() != points_2.size()) {
        cout << "[ERROR] points num is diff!" << endl;
        return ;
    }

    int p_size = points_1.size();
    for (int i=0;i<p_size;i++) {
        auto p_1 = points_1[i];
        auto p_2 = points_2[i];

        cv::circle(image, cv::Point(p_1.GetPt().first, p_1.GetPt().second), radius, color_1);
        cv::circle(image, cv::Point(p_2.GetPt().first, p_2.GetPt().second), radius, color_2);
        cv::line(image, cv::Point(p_1.GetPt().first, p_1.GetPt().second), cv::Point(p_2.GetPt().first, p_2.GetPt().second), color_1);
    }

    cv::imshow(gname, image);
}

int main() {

    cout << "[INFO] IMAGE_PATH_1: " << img_1 << endl;
    cout << "[INFO] IMAGE_PATH_2: " << img_2 << endl;

    cout << "\n[STATUS] Starting read images ... " << endl;
    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, cv::IMREAD_GRAYSCALE);

    if (image_1.data == nullptr || image_2.data == nullptr) {
        cout << "[ERROR] Cannot read images !" << endl;
        return 1;
    }

    cout << "\n[STATUS] Starting Extract points in image_1" << endl;
    vector<mKeyPoint> keypoints_1, keypoints_2;
    if (!FeatureExtract(image_1, keypoints_1)) return 1;

    cout << "\n[STATUS] Starting calculate Optical Flow using Gaussion ... \n";
    vector<bool> success;
    CalculateOpticalFlowSingleLevel(image_1, image_2, keypoints_1, keypoints_2, success, false, 500);

    DrawKeypoints("keypoints_2", image_2, keypoints_2);
    ShowFlow("Flow", image_2, keypoints_1, keypoints_2);
    cv::waitKey(0);

    return 0;
}