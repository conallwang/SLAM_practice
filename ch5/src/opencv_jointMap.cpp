#include <iostream>
#include <fstream>
#include <unistd.h>
using namespace std;

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pangolin/pangolin.h>

#include <boost/format.hpp>

#include "sophus/se3.hpp"
using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

string base_path = "/home/johnson/SLAM/ch5/";

// camera calib
double cx = 325.5, cy = 253.5, fx = 518.0, fy = 519.0;
double depth_scale = 1000.0;

// showPointCloud
void showPointCloud(vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud) {
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
            glColor3f(p[3]/255.0, p[4]/255.0, p[5]/255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

int main(int argc, char* argv[]) {

    vector<cv::Mat> colorImgs, depthImgs;
    vector<Sophus::SE3d> poses;

    ifstream fin(base_path + "pose.txt");
    if (!fin) {
        cout << "ERROR: cannot find " << base_path + "pose.txt" << endl;
        return 1;
    }

    for (int i=0;i<5;i++) {
        boost::format fmt("%s/%d.%s");
        colorImgs.push_back(cv::imread(base_path + (fmt % "color" % (i+1) % "png").str()));
        depthImgs.push_back(cv::imread(base_path + (fmt % "depth" % (i+1) % "pgm").str(), -1));

        double data[7] = {0};
        for (auto &d: data) {
            fin >> d;
        }  
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]), Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(pose);
    }

    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointCloud;
    pointCloud.reserve(1000000);

    for (int i=0;i<5;i++) {
        cout << "Process Image " << i+1 << endl;
        cv::Mat colorImg = colorImgs[i];
        cv::Mat depthImg = depthImgs[i];

        cout << colorImg.rows << " " << colorImg.cols << endl;
        cv::imshow("colorImg", colorImg);
        cv::waitKey(0);

        Sophus::SE3d T = poses[i];
        for (int v=0;v<colorImg.rows;v++) {
            for (int u=0;u<colorImg.cols;u++) {
                Vector3d point(0, 0, 0);
                
                point[2] = depthImg.at<unsigned short>(v, u) / 1000.0;
                point[1] = (v - cy) * point[2] / fy;
                point[0] = (u - cx) * point[2] / fx;

                Vector3d pointWorld = T * point;

                Vector6d p;
                p.head<3>() = pointWorld;
                p[5] = colorImg.data[v * colorImg.step + u * colorImg.channels()];                  // B
                p[4] = colorImg.data[v * colorImg.step + u * colorImg.channels() + 1];              // G
                p[3] = colorImg.data[v * colorImg.step + u * colorImg.channels() + 2];              // R

                pointCloud.push_back(p);
            }
        }
    }

    cout << "Total number: " << pointCloud.size() << endl;
    showPointCloud(pointCloud);

    return 0;
}