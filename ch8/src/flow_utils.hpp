#ifndef FLOW_UTILS
#define FLOW_UTILS

#include <iostream>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <orb_utils.hpp>

using namespace std;
using namespace Eigen;

class OpticalFlow {
    private:
    cv::Mat image_1, image_2;
    vector<mKeyPoint> keypoints_1, keypoints_2;
    vector<bool> success;
    bool inverse;
    bool has_initial;
    int half_patch_size;
    int iter_num;
    
    public:
    OpticalFlow(cv::Mat image_1, cv::Mat image_2, vector<mKeyPoint> keypoints_1, vector<mKeyPoint> keypoints_2, vector<bool> success, 
            bool inverse = false, bool has_initial = false, int iter_num = 100, int half_patch_size = 4);

    void CalculateOpticalFlow(const cv::Range& range);

    vector<mKeyPoint> GetResult();
};

#endif