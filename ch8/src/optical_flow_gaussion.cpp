#include "flow_utils.hpp"
#include <orb_utils.hpp>
#include <string.h>
#include <cstring>

string img_1 = "/home/johnson/SLAM/ch8/pics/LK1.png";
string img_2 = "/home/johnson/SLAM/ch8/pics/LK2.png";


void CalculateOpticalFlowSingleLevel(cv::Mat image_1, cv::Mat image_2, vector<mKeyPoint> keypoints_1, vector<mKeyPoint>& keypoints_2, 
                                    vector<bool>& success, bool inverse = false, bool has_initial = false, int iter_num = 100, int half_patch_size = 4) {
    keypoints_2.resize(keypoints_1.size());
    success.resize(keypoints_1.size());
    OpticalFlow tracker(image_1, image_2, keypoints_1, keypoints_2, success, inverse, has_initial, iter_num, half_patch_size);
    cv::parallel_for_(cv::Range(0, keypoints_1.size()),
                std::bind(&OpticalFlow::CalculateOpticalFlow, &tracker, placeholders::_1));
    keypoints_2 = tracker.GetResult();
}

void CalculateOpticalFlowMultiLevel(cv::Mat image_1, cv::Mat image_2, vector<mKeyPoint> keypoints_1, vector<mKeyPoint>& keypoints_2, vector<bool> success, 
                                    int iter_num = 100, int pyramid_level = 4, double scale = 0.5, bool inverse = false, int half_patch_size = 4) {
    vector<cv::Mat> pyramids_1;
    vector<cv::Mat> pyramids_2;
    double* scales = new double[pyramid_level];

    // initalize the scales
    for (size_t i=0;i<pyramid_level;i++) {
        scales[i] = pow(scale, static_cast<double>(i));
    }

    // create pyramids
    for (size_t i=0;i<pyramid_level;i++) {
        cv::Mat im_1, im_2;
        cv::resize(image_1, im_1, cv::Size(static_cast<int>(image_1.cols * scales[i]), static_cast<int>(image_1.rows * scales[i])));
        cv::resize(image_2, im_2, cv::Size(static_cast<int>(image_2.cols * scales[i]), static_cast<int>(image_2.rows * scales[i])));

        pyramids_1.push_back(im_1);
        pyramids_2.push_back(im_2);
    }

    // initialize the beginning keypoints_2
    size_t p_size = keypoints_1.size();
    // cout << p_size << endl;
    vector<mKeyPoint> kp1, kp2;
    for (size_t i=0;i<p_size;i++) {
        mKeyPoint pt1, pt2;
        pt1.SetPt(static_cast<int>(keypoints_1[i].GetPt().first * scales[pyramid_level - 1]), static_cast<int>(keypoints_1[i].GetPt().second * scales[pyramid_level - 1]));
        // cout << pt1.GetPt().first << " " << pt1.GetPt().second << endl;
        kp1.push_back(pt1);
        kp2.push_back(pt1);
    }

    // cout << pyramids_1[0].rows << " " << pyramids_1[0].cols << endl;
    // cout << pyramids_2[0].rows << " " << pyramids_2[0].cols << endl;
    // calculate multi Level
    for (int i=pyramid_level - 1;i>=0;i--) {
        // cout << pyramids_1[i].rows << " " << pyramids_1[i].cols << endl;
        // cout << pyramids_2[i].rows << " " << pyramids_2[i].cols << endl;
        cout << "========================================================================= " << i << "==============================================================================" << endl << endl;
        sleep(1);
        CalculateOpticalFlowSingleLevel(pyramids_1[i], pyramids_2[i], kp1, kp2, success, inverse, true);
        
        if (i > 0) {
            for (size_t i=0;i<p_size;i++) {
                kp1[i].SetPt(static_cast<int>(kp1[i].GetPt().first / scale), static_cast<int>(kp1[i].GetPt().second / scale));
                kp2[i].SetPt(static_cast<int>(kp2[i].GetPt().first / scale), static_cast<int>(kp2[i].GetPt().second / scale));
            }
        }
    }   

    cout << "\n[STATUS] Saving Result ... \n";
    // save results
    keypoints_2 = kp2;
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
    vector<mKeyPoint> keypoints_1, keypoints_2, keypoints_2_multi;
    if (!FeatureExtract(image_1, keypoints_1)) return 1;

    cout << "\n[STATUS] Starting calculate Optical Flow using Gaussion ... \n";
    vector<bool> success;
    // CalculateOpticalFlowSingleLevel(image_1, image_2, keypoints_1, keypoints_2, success);

    cout << "\n[STATUS] Starting calculate Optical Flow using pyramids ... \n";
    CalculateOpticalFlowMultiLevel(image_1, image_2, keypoints_1, keypoints_2_multi, success);

    cout << "\n[STATUS] Drawing ... \n";
    // DrawKeypoints("keypoints_2", image_2, keypoints_2);
    DrawKeypoints("keypoints_2_multi", image_2, keypoints_2_multi);
    // ShowFlow("Flow", image_2, keypoints_1, keypoints_2);
    cv::waitKey(0);

    return 0;
}