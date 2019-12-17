#include <iostream>
#include <string.h>
#include <cstring>
#include <vector>
#include <unistd.h>

#include "orb_utils.hpp"

#include <pangolin/pangolin.h>

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";

void DrawCloud(vector<cv::Point3d> points) {
    if (points.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    // Create pangolin window and plot
    pangolin::CreateWindowAndBind("Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/ 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit()) {
        // clear buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        

        glPointSize(2);
        glBegin(GL_POINTS);
        for (cv::Point3d p: points) {
            glColor3f(0.0f, 0.0f, 0.0f);
            glVertex3d(p.x, p.y, p.z);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);
    }
}

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