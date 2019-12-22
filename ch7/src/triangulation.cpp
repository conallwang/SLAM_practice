#include <iostream>
#include <string.h>
#include <cstring>
#include <vector>
#include <unistd.h>

#include "orb_utils.hpp"

#include <pangolin/pangolin.h>

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";

int main(int argc, char* argv[]) {

    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
    }

    printf("[INFO] IMAGE_PATH_1: %s", img_1.c_str());
    printf("[INFO] IMAGE_PATH_2: %s", img_2.c_str());

    printf("[STATUS] Starting read images ... \n");

    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, cv::IMREAD_GRAYSCALE);

    if (image_1.data == nullptr || image_2.data == nullptr) {
        printf("[ERROR] Cannot open image. \n");
        printf("        IMAGE_1: %s\n", img_1.c_str());
        printf("        IMAGE_2: %s\n", img_2.c_str());
        printf("EXIT!\n");
        return 1;
    }

    printf("\n[STATUS] Starting Extract Features ...\n");
    vector<mKeyPoint> keypoints_1, keypoints_2;

    if (!FeatureExtract(image_1, keypoints_1)) return 1;
    if (!FeatureExtract(image_2, keypoints_2)) return 2;

    printf("\n[STATUS] Starting Compute ORB descriptors\n");
    vector<Descriptor> descriptors_1, descriptors_2;

    ComputeORB(image_1, keypoints_1, descriptors_1);
    ComputeORB(image_2, keypoints_2, descriptors_2);

    printf("\n[STATUS] Starting Matching ... \n");
    vector<cv::DMatch> matches;
    Match(descriptors_1, descriptors_2, matches, 35);

    printf("\n[STATUS] Compute R and t of 2 compared to 1");
    cv::Mat R, t;
    PoseEstimate2d2d(keypoints_1, keypoints_2, matches, R, t);

    printf("\n[STATUS] Starting Compute triangulation ... \n");
    vector<cv::Point3d> cloudpoints;
    Triangulation(keypoints_1, keypoints_2, matches, R, t, cloudpoints);

    // Show pointcloud
    for (cv::Point3d p: cloudpoints) {
        printf("[%f, %f, %f]\n", p.x, p.y, p.z);
    }
    DrawCloud(cloudpoints);

    return 0;
}