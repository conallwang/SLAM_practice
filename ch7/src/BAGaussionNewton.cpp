#include <iostream>
#include <string.h>
#include <cstring>

#include "orb_utils.hpp"

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <sophus/se3.hpp>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

const int iter_num = 100;

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";

string depth = "/home/johnson/SLAM/ch7/pics/depth/1_depth.png";


// Convert pixel to camera axis
cv::Point2f pixel2cam(cv::Point2f p, cv::Mat K) {
    return cv::Point2f(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

Vector2d cam2pixel(cv::Point2f p, cv::Mat K) {
    Vector2d res;
    res[0] = p.x * K.at<double>(0, 0) + K.at<double>(0, 2);
    res[1] = p.y * K.at<double>(1, 1) + K.at<double>(1, 2);
    return res;
}

// Bundle Adjustment using Gaussion Newton
void BAGaussionNewton(vector<cv::Point3f> points3d, vector<cv::Point2f> points2d, cv::Mat K, Sophus::SE3d& pose) {
    if (points3d.size() != points2d.size()) {
        printf("The point num of 3d: %d\n", (int)points3d.size());
        printf("The point num of 2d: %d\n", (int)points2d.size());
        return ;
    }

    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    int p_size = points3d.size();
    for (int epoch=0;epoch<iter_num;epoch++) {
        Matrix<double, 6, 6> JJT = Matrix<double, 6, 6>::Zero();
        Vector6d Je = Vector6d::Zero();
        double error = 0.0;

        // JT
        Matrix<double, 2, 6> J;
        Vector2d e;

        for (int i=0;i<p_size;i++) {
            // 3D point
            Vector3d p3(points3d[i].x, points3d[i].y, points3d[i].z);
            Vector2d p2(points2d[i].x, points2d[i].y);

            // Compute point in camera
            p3 = pose * p3;
            double x = p3[0];
            double y = p3[1];
            double inv_z = 1 / p3[2];
            double inv_z2 = inv_z * inv_z;

            // Compute J
            J(0, 0) = - fx * inv_z; 
            J(0, 1) = 0; 
            J(0, 2) = fx * x * inv_z2;
            J(0, 3) = x * y * fx * inv_z2; 
            J(0, 4) =  - fx - (fx * x * x * inv_z2);
            J(0, 5) = fx * y * inv_z;
            J(1, 0) = 0;
            J(1, 1) = - fy * inv_z;
            J(1, 2) = fy * y * inv_z2;
            J(1, 3) = fy + fy * y * y * inv_z2;
            J(1, 4) = - fy * x * y * inv_z2;
            J(1, 5) = - fy * x * inv_z;

            // Compute e
            Vector2d pe = cam2pixel(cv::Point2f(p3[0] / p3[2], p3[1] / p3[2]), K);
            e = p2 - pe;

            // Report error
            error += e.squaredNorm();

            JJT += J.transpose() * J;
            Je -= J.transpose() * e;
        }   

        cout << "\n[INFO] " << "epoch = " << epoch << ", total error = " << error << endl;

        // Compute delta T
        // Vector6d dx = JJT.inverse() * Je;
        Vector6d dx = JJT.ldlt().solve(Je);

        if (isnan(dx[0])) {
            cout << "[ERROR] result is nan !" << endl;
            break;
        }

        if (dx.norm() < 1e-6) {
            // converge
            break;
        }

        pose = Sophus::SE3d::exp(dx) * pose;
    }

    cout << "pose by g-n: \n" << pose.matrix() << endl;
}

int main (int argc, char* argv[]) {

    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
        depth = argv[3];
    }

    printf("[INFO] IMAGE_PATH_1: %s", img_1.c_str());
    printf("[INFO] IMAGE_PATH_2: %s", img_2.c_str());
    printf("[INFO] DEPTH_PATH: %s", depth.c_str());

    printf("[STATUS] Starting read images ... \n");
    
    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, cv::IMREAD_GRAYSCALE);
    cv::Mat depth_1 = cv::imread(depth, cv::IMREAD_UNCHANGED);

    if (image_1.data == nullptr || image_2.data == nullptr || depth_1.data == nullptr) {
        printf("[ERROR] Cannot open image. \n");
        printf("        IMAGE_1: %s\n", img_1.c_str());
        printf("        IMAGE_2: %s\n", img_2.c_str());
        printf("        DEPTH: %s\n", depth.c_str());
        printf("EXIT!\n");
        return 1;
    }

    // Show depth
    // cout << "[INFO] DEPTH_IMAGE: \n" << depth_1 << endl;

    printf("\n[STATUS] Starting Feature Extract ... \n");
    vector<mKeyPoint> keypoints_1, keypoints_2;
    if (!FeatureExtract(image_1, keypoints_1)) return 1;
    if (!FeatureExtract(image_2, keypoints_2)) return 2;

    printf("\n[STATUS] Starting Compute ORB descriptors ... \n");
    vector<Descriptor> descriptors_1, descriptors_2;
    ComputeORB(image_1, keypoints_1, descriptors_1);
    ComputeORB(image_2, keypoints_2, descriptors_2);

    printf("\n[STATUS] Starting Matching ... \n ");
    vector<cv::DMatch> matches;
    Match(descriptors_1, descriptors_2, matches, 35);

    // Compute 3D position according to depth_1
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    vector<cv::Point3f> points3d;
    vector<cv::Point2f> points2d;
    for (cv::DMatch m: matches) {
        cv::Point3f point3d;
        cv::Point2f point2d;
        ushort d = depth_1.ptr<unsigned short>((int)keypoints_1[m.queryIdx].GetPt().second)[(int)keypoints_1[m.queryIdx].GetPt().first];
        if (d == 0)
            continue;
        float dd = d / 5000.0;
        point3d.x = ((keypoints_1[m.queryIdx].GetPt().first - K.at<double>(0, 2)) / K.at<double>(0, 0)) * dd;
        point3d.y = ((keypoints_1[m.queryIdx].GetPt().second - K.at<double>(1, 2)) / K.at<double>(1, 1)) * dd;
        point3d.z = dd;
        point2d.x = keypoints_2[m.trainIdx].GetPt().first;
        point2d.y = keypoints_2[m.trainIdx].GetPt().second;
        cout << point3d << endl;
        cout << point2d << endl;
        points3d.push_back(point3d);
        points2d.push_back(point2d);
    }

    cout << "[INFO] 3d-2d pairs: " << points3d.size() << endl;

    Sophus::SE3d pose_gn;
    BAGaussionNewton(points3d, points2d, K, pose_gn);

    return 0;
}