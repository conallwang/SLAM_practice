#include <iostream>
#include <string.h>
#include <cstring>
#include <boost/format.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>

using namespace std;
using namespace Eigen;

double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
double baseline = 0.573;
Matrix3d K;

const int images_num = 10;
boost::format fmt_left("/home/johnson/Dataset/kitti/groundtruth/sequences/01/image_0/%06d.png");
boost::format fmt_right("/home/johnson/Dataset/kitti/groundtruth/sequences/01/image_1/%06d.png");

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

int main() {
    
    K << fx, 0, cx,
         0, fy, cy,
         0, 0, 1;
    cout << "[INFO] K = \n" << K.matrix() << endl;

    cout << "\n[STATUS] Starting read images ... \n";

    vector<cv::Mat> images_left;
    vector<cv::Mat> images_right;

    for (int i=0;i<images_num;i++) {
        cout << "[INFO] Reading image " << i << " ...\n";
        cv::Mat img_l, img_r;
        img_l = cv::imread((fmt_left % i).str(), cv::IMREAD_GRAYSCALE);
        img_r = cv::imread((fmt_right % i).str(), cv::IMREAD_GRAYSCALE);

        if (img_l.data == nullptr || img_r.data == nullptr) {
            cerr << "[ERROR] cannot read: " << (fmt_left % i).str() << ", " << (fmt_right % i).str() << endl;
            return 1;
        }
        images_left.push_back(img_l);
        images_right.push_back(img_r);
    }

    cout << "\n[STATUS] Computing disparity image ...\n";
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0, 96, 9, 8*9*9, 32*9*9, 1, 63, 10, 100, 32
        );
    vector<cv::Mat> disparities;
    for (int i=0;i<images_num;i++) {
        cout << "[INFO] Process image " << i << " ...\n";
        cv::Mat disparity_tmp, disparity;
        sgbm->compute(images_left[i], images_right[i], disparity_tmp);
        disparity_tmp.convertTo(disparity, CV_32F, 1.0 / 16.0f);
        disparities.push_back(disparity);
    }

    cout << "\n[STATUS] Computing 3D points ...\n";
    vector<Vector4d, aligned_allocator<Vector4d>> points;
    for (int i=0;i<images_num;i++) {
        cout << "[INFO] Process image " << i << " ..." << endl;
        cv::Mat image = images_left[i];
        cv::Mat disparity = disparities[i];
        for (int y=0;y<images_left[i].rows;y++) {
            for (int x=0;x<images_right[i].cols;x++) {
                if (disparity.at<float>(y, x) <= 10.0 || disparity.at<float>(y, x) >= 96.0) continue;

                double Z = fx * baseline / disparity.at<float>(y, x);
                double X = (x - cx) * Z / fx;
                double Y = (y - cy) * Z / fy;
                
                points.push_back(Vector4d(X, Y, Z, image.at<uchar>(y, x) / 255.0));
            }
        }
    }

    cout << "\n[STATUS] Drawing ... \n";
    DrawCloud(points);

    return 0;
}