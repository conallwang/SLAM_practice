/*
 *  阅读ORB Paper， 并对Paper中的Oriented FAST、steer BRIEF进行复现。
 *  
 *  Author:         @ Johnson 
 *  Date:           2019.11.19
 *  Description:    
 */
#ifndef ORB_UTILS_
#define ORB_UTILS_
#include <iostream>
#include <algorithm>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

typedef pair<int, int> Point;

// KeyPoint
class mKeyPoint {
private :
    Point pt;                   // position
    Vector2d orientation;       // orientation
    double cos_theta;           // cos
    double sin_theta;           // sin
public :

    // Constructor
    mKeyPoint() {}

    // Get
    Point GetPt() {
        return pt;
    }

    Vector2d GetOrientation() {
        return orientation;
    }

    double GetCos() {
        return cos_theta;
    }

    double GetSin() {
        return sin_theta;
    }

    // Set 
    void SetPt(int x, int y) {
        pt.first = x;
        pt.second = y;
    }

    void SetPt(Point pt) {
        this->pt = pt;
    }

    void SetOrientation(Vector2d orientation) {
        this->orientation = orientation;
    }

    void SetCos(double cos) {
        this->cos_theta = cos;
    }

    void SetSin(double sin) {
        this->sin_theta = sin;
    }
};

// Descriptor
// 256 bits
struct Descriptor {
public :
    uint32_t desc[8];

    void Zero() {
        for (int i=0;i<8;i++) {
            desc[i] = 0;
        }
        return ;
    }
};

// FAST algorithm
bool FeatureExtract(cv::Mat image, vector<mKeyPoint>& keypoints, int radius = 12, double threshold = 40, int patch_size = 16);

// Show KeyPoints
void ShowKeyPoints(string gragh_name, cv::Mat image, vector<mKeyPoint>& keypoints, cv::Scalar color);

// Compute ORB descriptor   256 bit
void ComputeORB(cv::Mat image, vector<mKeyPoint> keypoints, vector<Descriptor>& descriptor);

// Brute full match
void Match(vector<Descriptor> descriptors_1, vector<Descriptor> descriptors_2, vector<cv::DMatch>& matches, int d_max = 60);

// Show Match Results
void ShowMatch(string gragh_name, cv::Mat image_1, vector<mKeyPoint> keypoints_1, cv::Mat image_2, vector<mKeyPoint> keypoints_2, vector<cv::DMatch> matches);

// Compute essential matrix and fundamental matrix
void PoseEstimate2d2d(vector<mKeyPoint> keypoints_1, vector<mKeyPoint> keypoints_2, vector<cv::DMatch> matches, cv::Mat& R, cv::Mat& t);

// trianglar measurement
void Triangulation(vector<mKeyPoint> keypoints_1, vector<mKeyPoint> keypoints_2, vector<cv::DMatch> matches, cv::Mat R, cv::Mat t, vector<cv::Point3d>& points);

#endif