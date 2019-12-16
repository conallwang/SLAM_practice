#include <iostream>
#include <string.h>
#include <cstring>
#include <vector>

#include "orb_utils.hpp"

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";

string depth = "/home/johnson/SLAM/ch7/pics/depth/1_depth.png";

int main(int argc, char* argv[]) {

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

    vector<cv::Point3d> points3d;
    vector<cv::Point2d> points2d;
    for (cv::DMatch m: matches) {
        cv::Point3d point3d;
        cv::Point2d point2d;
        point3d.z = depth_1.at<unsigned short>(keypoints_1[m.queryIdx].GetPt().second, keypoints_1[m.queryIdx].GetPt().first);
        if (point3d.z == 0)
            continue;
        point3d.z = point3d.z / 5000.0;
        point3d.x = (keypoints_1[m.queryIdx].GetPt().first - K.at<double>(0, 2)) / K.at<double>(0, 0) * point3d.z;
        point3d.y = (keypoints_1[m.queryIdx].GetPt().second - K.at<double>(1, 1)) / K.at<double>(1, 2) * point3d.z;
        point2d.x = keypoints_2[m.trainIdx].GetPt().first;
        point2d.y = keypoints_2[m.trainIdx].GetPt().second;
        cout << point3d << endl;
        cout << point2d << endl;
        points3d.push_back(point3d);
        points2d.push_back(point2d);
    }
    // Compute R, t using opencv PnP
    cv::Mat r, t;
    cv::solvePnP(points3d, points2d, K, cv::Mat(), r, t, false);

    cv::Mat R;
    cv::Rodrigues(r, R);

    cout << "\n[INFO] R= \n" << R << endl;
    cout << "\n[INFO] t= \n" << t << endl;

    return 0;
}