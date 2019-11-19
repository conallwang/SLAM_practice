#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <algorithm>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";

void printUsage() {
    printf("USAGE: ./orb_cv [IMAGE_1] [IMAGE_2]\n\n");
    printf("ARGUMENTS: \n");
}

bool cmp(cv::DMatch a, cv::DMatch b) {
    return a.distance < b.distance;
}

int main(int argc, char* argv[]) {
    // Set Image Path
    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
    }

    printf("[INFO] IMAGE_PATH_1: %s\n", img_1.c_str());
    printf("[INFO] IMAGE_PATH_2: %s\n", img_2.c_str());

    // Open Images
    printf("\n[STATUS] Starting to open images ... \n");
    cv::Mat image_1 = cv::imread(img_1, CV_LOAD_IMAGE_COLOR);
    cv::Mat image_2 = cv::imread(img_2, CV_LOAD_IMAGE_COLOR);

    if (image_1.data == nullptr || image_2.data == nullptr) {
        printf("[ERROR] Image Read Error: \n");
        printf("        1: %s", img_1.c_str());
        printf("        2: %s", img_2.c_str());
        printf("EXIT");
        return 1;
    }

    printf("[STATUS] Open image SUCCESS. \n\n");

    // Create OpenCV detector, descripter, matcher
    vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // detecte the position of angle point
    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    detector->detect(image_1, keypoints_1);
    detector->detect(image_2, keypoints_2);

    // compute descriptor
    descriptor->compute(image_1, keypoints_1, descriptors_1);
    descriptor->compute(image_2, keypoints_2, descriptors_2);

    // Timing 
    chrono::steady_clock::time_point finish_time = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);

    cout << "[INFO] Time used = " << time_used.count() << " seconds" << endl;

    printf("[INFO] keypoint_1 size: %d\n", (int)keypoints_1.size());
    printf("[INFO] keypoint_2 size: %d\n\n", (int)keypoints_2.size());
    printf("[INFO] descriptor_1 size: \n");
    printf("                    rows: %d\n", descriptors_1.rows);
    printf("                    cols: %d\n", descriptors_1.cols);
    printf("[INFO] descriptor_2 size: \n");
    printf("                    rows: %d\n", descriptors_2.rows);
    printf("                    cols: %d\n", descriptors_2.cols);
    // Show image with keypoints
    cv::Mat outImg;
    cv::drawKeypoints(image_1, keypoints_1, outImg);

    cv::imshow("orb feature", outImg);
    cv::waitKey(0);

    // Match
    printf("\n[STATUS] Starting Matching ... \n");

    vector<cv::DMatch> matches;

    start_time = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    finish_time = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);

    printf("[STATUS] Matching finish\n");
    cout << "[INFO] Time used = " << time_used.count() << " seconds" << endl;

    // 
    auto min_max = minmax_element(matches.begin(), matches.end(), cmp);

    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    printf("\n[INFO] Max dist: %f \n", max_dist);
    printf("[INFO] Min dist = %f \n", min_dist);

    // Choose Good Match
    vector<cv::DMatch> good_match;
    for (int i=0;i<descriptors_1.rows;i++) {
        if (matches[i].distance <= max(2*min_dist, 30.0)) {
            good_match.push_back(matches[i]);
        }
    }

    printf("[INFO] Full matches size: %d\n", (int)matches.size());
    printf("[INFO] Good matches size: %d\n", (int)good_match.size());

    // show results
    cv::Mat fullImg;
    cv::Mat goodImg;

    cv::drawMatches(image_1, keypoints_1, image_2, keypoints_2, matches, fullImg);
    cv::drawMatches(image_1, keypoints_1, image_2, keypoints_2, good_match, goodImg);

    cv::imshow("Full Match", fullImg);
    cv::imshow("Good Match", goodImg);

    cv::waitKey(0);
}