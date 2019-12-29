#include <iostream>
#include <string.h>
#include <cstring>
#include <unistd.h>

#include <pangolin/pangolin.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
double baseline = 0.573;
Matrix<double, 3, 3> K; 

void DrawCloud(vector<Vector4d, aligned_allocator<Vector4d>> points) {
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
        for (Vector4d p: points) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);
    }
}

string img_1 = "/home/johnson/SLAM/ch8/pics/left.png";
string d_1 = "/home/johnson/SLAM/ch8/pics/disparity.png";

int main() {

    K << fx, 0, cx, 
         0, fy, cy,
         0, 0, 1;

    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat depth_1 = cv::imread(d_1, cv::IMREAD_GRAYSCALE);

    vector<Vector4d, aligned_allocator<Vector4d>> points;
    for (int y=0;y<image_1.rows;y++) {
        for (int x=0;x<image_1.cols;x++) {
            double Z = fx * baseline / depth_1.at<uchar>(y, x);
            double X = (x - cx) * Z / fx;
            double Y = (y - cy) * Z / fy;

            points.push_back(Vector4d(X, Y, Z, image_1.at<uchar>(y, x) / 255.0));
            // cout << "[INFO] X = " << X << ", Y = " << Y << ", Z = " << Z << endl;
        }
    }

    DrawCloud(points);

    return 0;
}