#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <pangolin/pangolin.h>

using namespace Eigen;

string leftFile = "../image/left.png";
string rightFile = "../image/right.png";

// Pangolin
void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

int main(int argc, char* argv[]) {

    if (argc != 1 && argc != 3) {
        cout << argc << endl;
        cout << "USAGE: ./opencv_stereo [LEFT_IMAGE_PATH] [RIGHT_IMAGE_PATH]" << endl;
        return 1;
    }
    
    if (argc == 3) {
        leftFile = argv[1];
        rightFile = argv[2];
    }

    cout << "Left Image: " << leftFile << endl;
    cout << "Right Image: " << rightFile << endl;

    // camera
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;   
    
    // baseline
    double b = 0.573;

    // Read Image
    cv::Mat left_img = cv::imread(leftFile, 0);
    cv::Mat right_img = cv::imread(rightFile, 0);

    cout << "test" << endl;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 96, 9, 8*9*9, 32*9*9, 1, 63, 10, 100, 32);
    cv::Mat disparity_sgbm, disparity;
    
    sgbm->compute(left_img, right_img, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    // PointCloud
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointCloud;

    // Computing
    for (size_t v=0;v<left_img.rows;v++) {
        for (size_t u=0;u<left_img.cols;u++) {
            if (disparity.at<float>(v, u) <= 10.0 || disparity.at<float>(v, u) >= 96.0) {
                continue;
            }

            Vector4d point(0, 0, 0, left_img.at<uchar>(v, u) / 255.0);

            float x = (u - cx) / fx;
            float y = (v - cy) / fy;
            float depth = fx * b / disparity.at<float>(v, u);
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointCloud.push_back(point);
        }
    }

    cv::imshow("left", left_img);
    cv::imshow("right", right_img);
    cv::imshow("disparity", disparity / 96.0f);
    cv::waitKey(0);

    // Plot PointCloud
    showPointCloud(pointCloud);

    return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}