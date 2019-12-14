#include <cstring>
#include <string.h>
#include <vector>

#include "orb_utils.hpp"

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";

int main(int argc, char* argv[]) {

    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
    }

    printf("[INFO] IMAGE_PATH_1: %s\n", img_1);
    printf("[INFO] IMAGE_PATH_2: %s\n", img_2);

    // Read Images
    printf("[STATUS] Starting open images ...\n");

    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, cv::IMREAD_GRAYSCALE);

    if (image_1.data == nullptr || image_2.data == nullptr) {
        printf("[ERROR] Cannot open image. \n");
        printf("        IMAGE_1: %s\n", img_1.c_str());
        printf("        IMAGE_2: %s\n", img_2.c_str());
        printf("EXIT!\n");
        return 1;
    }

    // Extract Features ---- FAST12
    vector<mKeyPoint> keypoints_1, keypoints_2;
    
    printf("\n[STATUS] Starting feature extract ...\n");
    if (!FeatureExtract(image_1, keypoints_1)) return 1;
    if (!FeatureExtract(image_2, keypoints_2)) return 2;

    // Compute ORB ---- 256 descriptors
    printf("\n[STATUS] Starting compute ORB descriptors ... \n");
    vector<Descriptor> descriptors_1, descriptors_2;

    ComputeORB(image_1, keypoints_1, descriptors_1);
    ComputeORB(image_2, keypoints_2, descriptors_2);

    // Full brute match
    printf("\n[STATUS] Starting matching ... \n");
    vector<cv::DMatch> matches;
    Match(descriptors_1, descriptors_2, matches, 35);

    // Compute essential matrix and fundamental matrix
    cv::Mat R, t;
    PoseEstimate2d2d(keypoints_1, keypoints_2, matches, R, t);

    // Compute t^R, and compare with E
    cv::Mat t_x = (cv::Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0), 
                                    t.at<double>(2, 0), 0, -t.at<double>(0, 0), 
                                    -t.at<double>(1, 0), t.at<double>(0, 0), 0);
    cout << "\nt^R: \n" << t_x * R << endl;

    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    
    // Verify 2d - 2d
    for (cv::DMatch m: matches) {
        Point p1 = keypoints_1[m.queryIdx].GetPt();
        Point p2 = keypoints_2[m.trainIdx].GetPt();

        cv::Mat x1 = (cv::Mat_<double>(3, 1) << (p1.first - K.at<double>(0, 2))/K.at<double>(0, 0), (p1.second - K.at<double>(1, 2)) / K.at<double>(1, 1), 1);
        cv::Mat x2 = (cv::Mat_<double>(1, 3) << (p2.first - K.at<double>(0, 2))/K.at<double>(0, 0), (p2.second - K.at<double>(1, 2)) / K.at<double>(1, 1), 1);

        cout << "[INFO] verified: "  << x2 * t_x * R * x1 << endl;
    }

    return 0;
}