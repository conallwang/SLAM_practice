#include <chrono>
#include "orb_utils.hpp"

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";

int main(int argc, char* argv[]) {
    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
    }

    printf("[INFO] IMAGE_PATH_1: %s\n", img_1.c_str());
    printf("[INFO] IMAGE_PATH_2: %s\n", img_2.c_str());

    // Process Image_1
    printf("[STATUS] Starting open image ... \n");

    cv::Mat image_1 = cv::imread(img_1, CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, CV_LOAD_IMAGE_GRAYSCALE);

    if (image_1.data == nullptr || image_2.data == nullptr) {
        printf("[ERROR] Cannot open image. \n");
        printf("        IMAGE_1: %s\n", img_1.c_str());
        printf("        IMAGE_2: %s\n", img_2.c_str());
        printf("EXIT!\n");
        return 1;
    }

    printf("[STATUS] Open images finish. \n");

    // Feature points extracting
    printf("\n[STATUS] Starting extract feature points ... \n");
    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    vector<mKeyPoint> keypoints_1, keypoints_2;
    if (!FeatureExtract(image_1, keypoints_1, 12, 40)) return 1;
    if (!FeatureExtract(image_2, keypoints_2, 12, 40)) return 1;

    printf("[INFO] KeyPoints_1 Size: %d\n", (int)keypoints_1.size());
    printf("[INFO] KeyPoints_2 Size: %d\n", (int)keypoints_2.size());

    printf("[STATUS] Extract finish. \n");

    // Show KeyPoints
    // ShowKeyPoints("KeyPoint_1", image_1, keypoints_1, cv::Scalar(0, 0, 255));
    // ShowKeyPoints("KeyPoint_2", image_2, keypoints_2, cv::Scalar(0, 0, 255));
    // cv::waitKey(0);
    
    // Compute descriptors
    printf("\n[STATUS] Starting computing descriptor ... \n"); 
    vector<Descriptor> descriptors_1, descriptors_2;

    ComputeORB(image_1, keypoints_1, descriptors_1);
    ComputeORB(image_2, keypoints_2, descriptors_2);

    chrono::steady_clock::time_point finish_time = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);

    printf("[STATUS] Computing descriptors finish. \n");
    cout << "[INFO] Time used: " << time_used.count() << " seconds" << endl;

    // Match
    printf("\n[STATUS] Starting to match ... \n");
    vector<cv::DMatch> matches;
    start_time = chrono::steady_clock::now();
    Match(descriptors_1, descriptors_2, matches, 35);
    finish_time = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);
    
    printf("[STATUS] Computing descriptors finish. \n");
    cout << "[INFO] Time used: " << time_used.count() << " seconds" << endl;
    // Good Match
    

    // show
    ShowMatch("Full Match", image_1, keypoints_1, image_2, keypoints_2, matches);
    cv::waitKey(0);

    return 0;
}