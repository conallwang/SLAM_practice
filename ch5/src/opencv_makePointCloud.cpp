#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <pangolin/pangolin.h>
using namespace Eigen;

// camera calib
double cx = 325.5, cy = 253.5, fx = 518.0, fy = 519.0;

// number to string
string num2str(int num) {
    string res;
    stringstream ss;
    ss << num;
    ss >> res;
    return res;
}

// Show Point Cloud
void showPointCloud(vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud) {
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

// Make pointCloud
// file format:
//      Color:  [ID].png
//      Depth:  [ID].pgm
void makePointCloud(int id) {
    string colorFile = "../color/" + num2str(id) + ".png";
    string depthFile = "../depth/" + num2str(id) + ".pgm";
    
    // Read Image
    cv::Mat colorImg, depthImg;
    colorImg = cv::imread(colorFile, 0);
    depthImg = cv::imread(depthFile, -1);

    // show Image
    cv::imshow("colorImg", colorImg);
    cv::imshow("depthImg", depthImg);
    cv::waitKey(0);

    // ** Debug **  480 640
    // cout << depthImg.rows << " " << depthImg.cols << endl;
    // cout << colorImg.rows << " " << colorImg.cols << endl;

    // 1041
    // cout << depthImg.at<short int>(472, 597) << endl;
    // cout << depthImg << endl;

    // pointCloud of [ID]
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointCloud;
    
    for (size_t v=0;v<colorImg.rows;v++) {
        for (size_t u=0;u<colorImg.cols;u++) {
            Vector4d point(0, 0, 0, colorImg.at<uchar>(v, u) / 255.0);
            
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double z = depthImg.at<short int>(v, u) / 1000.0;
            point[0] = x * z;
            point[1] = y * z;
            point[2] = z;
                
            pointCloud.push_back(point);
         }
    }

    /* int p_size = pointCloud.size();
    for (size_t i=0;i<p_size;i++) {
        cout << pointCloud[i].matrix().transpose() << endl;
    }*/
    showPointCloud(pointCloud);
    cv::waitKey(0);
}

int main(int argc, char* argv[]) {

    if (argc < 1) {
        cout << "USAGE: ./opencv_makePointCloud ID" << endl;
        return 1;
    }

    makePointCloud(atoi(argv[1]));

    return 0;
}