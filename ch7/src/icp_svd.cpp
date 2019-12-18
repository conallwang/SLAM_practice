#include <iostream>
#include <cstring>
#include <string.h>

#include "orb_utils.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";
string d_1 = "/home/johnson/SLAM/ch7/pics/depth/1_depth.png";
string d_2 = "/home/johnson/SLAM/ch7/pics/depth/2_depth.png";

Vector2d pixel2cam(Point p, cv::Mat K) {
    return Vector2d(
        (p.first - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.second - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

void ICP_SVD(vector<Vector3d> points3d_1, vector<Vector3d> points3d_2, Matrix3d& R, Vector3d& t) {
    if (points3d_1.size() != points3d_2.size()) {
        cout << "[ERROR] Vector Size Dismatch !" << endl;
        return;
    }

    int p_size = points3d_1.size();

    // compute center of points
    Vector3d p1(0, 0, 0);
    Vector3d p2(0, 0, 0);
    for (size_t i=0;i<p_size;i++) {
        p1 += points3d_1[i];
        p2 += points3d_2[i];
    }
    p1 /= p_size;
    p2 /= p_size;

    // Manage to get rotate matrix R
    for (size_t i=0;i<p_size;i++) {
        points3d_1[i] -= p1;
        points3d_2[i] -= p2;
    }

    Matrix3d W = Matrix3d::Zero();
    for (size_t i=0;i<p_size;i++) {
        W += points3d_1[i] * points3d_2[i].transpose();
    }

    // SVD
    JacobiSVD<Matrix3d> svd(W, ComputeFullU | ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();

    cout << "\n[INFO] U = \n" << U << endl;
    cout << "\n[INFO] V = \n" << V << endl;

    R = U * V.transpose();

    // Compute t
    t = p1 - R * p2;

    cout << "\n[INFO] R = \n" << R << endl;
    cout << "\n[INFO] t = \n" << t << endl;
}

int main(int argc, char* argv[]) {

    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
        d_1 = argv[3];
        d_2 = argv[4];
    }

    printf("[INFO] IMAGE_PATH_1: %s\n", img_1.c_str());
    printf("[INFO] IMAGE_PATH_2: %s\n", img_2.c_str());
    printf("[INFO] DEPTH_PATH_1: %s\n", d_1.c_str());
    printf("[INFO] DEPTH_PATH_2: %s\n", d_2.c_str());

    printf("\n[STATUS] Starting read images ... \n");
    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, cv::IMREAD_GRAYSCALE);
    cv::Mat depth_1 = cv::imread(d_1, cv::IMREAD_UNCHANGED);
    cv::Mat depth_2 = cv::imread(d_2, cv::IMREAD_UNCHANGED);

    printf("\n[STATUS] Starting Feature Extracting ...\n");
    vector<mKeyPoint> keypoints_1, keypoints_2;
    if (!FeatureExtract(image_1, keypoints_1)) return 1;
    if (!FeatureExtract(image_2, keypoints_2)) return 1;

    printf("\n[STATUS] Starting Compute ORB descriptors ... \n");
    vector<Descriptor> descriptors_1, descriptors_2;
    ComputeORB(image_1, keypoints_1, descriptors_1);
    ComputeORB(image_2, keypoints_2, descriptors_2);

    printf("\n[STATUS] Starting Matching ... \n");
    vector<cv::DMatch> matches;
    Match(descriptors_1, descriptors_2, matches, 35);

    printf("\n[STATUS] Starting compute 3D points ... \n");
    vector<Vector3d> points3d_1, points3d_2;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for (cv::DMatch m: matches) {
        float dd_1 = depth_1.at<unsigned short>(keypoints_1[m.queryIdx].GetPt().second, keypoints_1[m.queryIdx].GetPt().first);
        float dd_2 = depth_2.at<unsigned short>(keypoints_2[m.trainIdx].GetPt().second, keypoints_2[m.trainIdx].GetPt().first);
        if (dd_1 == 0 || dd_2 == 0) continue;

        double d_1 = dd_1 / 5000.0;
        double d_2 = dd_2 / 5000.0;

        Vector2d p2d_1 = pixel2cam(keypoints_1[m.queryIdx].GetPt(), K);
        Vector2d p2d_2 = pixel2cam(keypoints_2[m.trainIdx].GetPt(), K);

        points3d_1.push_back(Vector3d(p2d_1[0] * d_1, p2d_1[1] * d_1, d_1));
        points3d_2.push_back(Vector3d(p2d_2[0] * d_2, p2d_2[1] * d_2, d_2));
    }

    printf("\n[INFO] points num: %d\n", (int)points3d_2.size());

    // ICP
    printf("\n[STATUS] Starting compute ICP using SVD... \n");
    Matrix3d R;
    Vector3d t;
    ICP_SVD(points3d_1, points3d_2, R, t);

    return 0;
}