#include <iostream>
#include <string.h>
#include <cstring>
#include <vector>
#include <list>
#include <mutex>
#include <thread>
#include <boost/format.hpp>
#include "orb_utils.hpp"

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

// Camera
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
double baseline = 0.573;
Matrix<double, 3, 3> K; 

int GetPixel(cv::Mat image, int x, int y) {
    int rows = image.rows;
    int cols = image.cols;

    if (x < 0) x = 0;
    if (x >= cols) x = cols - 1;
    if (y < 0) y = 0;
    if (y >= rows) y = rows - 1;
    return image.at<uchar>(y, x);
}

class JacobianAccumulator {
    private:
    cv::Mat image_1;
    cv::Mat image_2;
    vector<Vector3d> points;
    Sophus::SE3d& T;
    std::mutex hession_mutex;


    Matrix<double, 6, 6> JJT = Matrix<double, 6, 6>::Zero();
    Vector6d Je = Vector6d::Zero();
    double error = 0.0;
    vector<Vector2d> projection_points;

    public:
    // Construction
    JacobianAccumulator(cv::Mat image_1, cv::Mat image_2, vector<Vector3d> points, Sophus::SE3d& T) : image_1(image_1), image_2(image_2), points(points), T(T) {}

    // reset
    void reset() {
        JJT = Matrix<double, 6, 6>::Zero();
        Je = Vector6d::Zero();
        error = 0.0;
        projection_points.clear();
    }

    // Compute Jacobian
    void accumulate_jacobian(const cv::Range& range) {
        const int half_patch_size = 1;
        Matrix<double, 6, 6> hession = Matrix<double, 6, 6>::Zero();
        Vector6d bias = Vector6d::Zero();
        double cost = 0;
        int cnt_good = 0;

        for (int i=range.start;i<range.end;i++) {
            Vector3d p3d = points[i];

            double X = p3d[0];
            double Y = p3d[1];
            double Z = p3d[2];
            double Z2 = Z * Z;

            // invalid depth
            if (Z <= 0) {
                continue;
            }
            
            // p2d
            Vector2d p2d = ((K * p3d) / p3d[2]).head(2);

            // compute e
            Vector3d p3d_prone = T * p3d;
            Vector2d p2d_prone = ((K * p3d_prone) / p3d_prone[2]).head(2);

            projection_points.push_back(p2d_prone);
            cnt_good++;
            for (int dy=-half_patch_size;dy<=half_patch_size;dy++) {
                for (int dx=-half_patch_size;dx<=half_patch_size;dx++) {
                    double e = GetPixel(image_1, p2d[0] + dx, p2d[1] + dy) - GetPixel(image_2, p2d_prone[0] + dx, p2d_prone[1] + dy);

                    // compute J
                    Matrix<double, 2, 6> right;
                    right << fx / Z, 0, - fx * X / Z2, - fx * X * Y / Z2, fx + fx * X * X / Z2, - fx * Y / Z,
                             0, fy / Z, - fy * Y / Z2, - fy - fy * Y * Y / Z2, fy * X * Y / Z2, fy * X / Z;
                    Matrix<double, 1, 2> left; 
                    left << 0.5 * (GetPixel(image_2, p2d_prone[0] + dx + 1, p2d_prone[1] + dy) - GetPixel(image_2, p2d_prone[0] + dx - 1, p2d_prone[1] + dy)),
                            0.5 * (GetPixel(image_2, p2d_prone[0] + dx, p2d_prone[1] + dy + 1) - GetPixel(image_2, p2d_prone[0] + dx, p2d_prone[1] + dy - 1));

                    Matrix<double, 1, 6> J = - left * right;
                    
                    hession += J.transpose() * J;
                    bias += - J.transpose() * e;
                    cost += e * e;
                }
            }
        }

        if (cnt_good) {
            std::unique_lock<std::mutex> lck(hession_mutex);
            JJT += hession;
            Je += bias;
            error += cost / cnt_good;
        }
    }

    // Get JJT
    Matrix<double, 6, 6> GetJJT() { return JJT; }

    // Get Je
    Vector6d GetJe() { return Je; }

    // Geterror
    double GetError() { return error; }

    // Get Projection
    vector<Vector2d> GetProjection() { return projection_points; }
    
};

void DirectSingle(cv::Mat image_1, cv::Mat image_2, vector<Vector3d> points, Sophus::SE3d& T, int iter_num = 100) {
    JacobianAccumulator jcaobian_accumulator(image_1, image_2, points, T);
    for (int i=0;i<iter_num;i++) {
        // JacobianAccumulator jcaobian_accumulator(image_1, image_2, points, T);
        jcaobian_accumulator.reset();
        cv::parallel_for_(cv::Range(0, points.size()), std::bind(&JacobianAccumulator::accumulate_jacobian, &jcaobian_accumulator, std::placeholders::_1));

        Matrix<double, 6, 6> JJT = jcaobian_accumulator.GetJJT();
        Vector6d Je = jcaobian_accumulator.GetJe();
        double error = jcaobian_accumulator.GetError();

        cout << "[INFO] epoch = " << i << ", " << "error = " << error << endl;

        Vector6d dT = JJT.ldlt().solve(Je);

        if (dT[0] == 0) {
            cout << "[ERROR] Update is nan." << endl;
            break;
        }

        if (dT.norm() <= 1e-3) {
            // converge
            break;
        }
        
        // update T
        T = Sophus::SE3d::exp(dT) * T;
    }

    cout << "\n[INFO] T = \n" << T.matrix() << endl;

    // Draw
    vector<mKeyPoint> keypoints_2;
    vector<Vector2d> projection_points = jcaobian_accumulator.GetProjection();
    for (Vector2d p: projection_points) {
        mKeyPoint tmp;
        tmp.SetPt(p[0], p[1]);
        keypoints_2.push_back(tmp);
    }

    DrawKeypoints("image_2", image_2, keypoints_2);
}

void DirectMulti(cv::Mat image_1, cv::Mat image_2, vector<Vector3d> points, Sophus::SE3d& T, int iter_num = 100) {
    // pyramid param
    int pyramid_layers = 4;
    vector<cv::Mat> pyramids_img_1, pyramids_img_2;
    double scale[] = {1, 0.5, 0.25, 0.125};

    // generate pyramid image
    for (int i=0;i<pyramid_layers;i++) {
        cv::Mat img_1, img_2;
        cv::resize(image_1, img_1, cv::Size(static_cast<int>(image_1.cols * scale[i]), static_cast<int>(image_1.rows * scale[i])));
        cv::resize(image_2, img_2, cv::Size(static_cast<int>(image_2.cols * scale[i]), static_cast<int>(image_2.rows * scale[i])));

        pyramids_img_1.push_back(img_1);
        pyramids_img_2.push_back(img_2);
    }

    // direct method
    double fxG = fx;
    double fyG = fy;
    double cxG = cx;
    double cyG = cy;
    Matrix3d KG = K;
    for (int i=pyramid_layers - 1;i>=0;i--) {
        vector<Vector3d> points_tmp;
        for (Vector3d p: points) {
            points_tmp.push_back(Vector3d(p[0] * scale[i] * scale[i], p[1] * scale[i] * scale[i], p[2] * scale[i]));
        }   

        fx = fxG * scale[i];
        fy = fyG * scale[i];
        cx = cxG * scale[i];
        cy = cyG * scale[i];
        K = KG * scale[i];
        DirectSingle(pyramids_img_1[i], pyramids_img_2[i], points_tmp, T, iter_num);
    }
}

string img_1 = "/home/johnson/SLAM/ch8/pics/left.png";
string depth = "/home/johnson/SLAM/ch8/pics/disparity.png";
string img_2 = "/home/johnson/SLAM/ch8/pics/000001.png";
boost::format fmt_others("/home/johnson/SLAM/ch8/pics/%06d.png");

int main(int argc, char* argv[]) {

    K <<    fx, 0, cx, 
            0, fy, cy, 
            0, 0, 1;

    cout << "\n[STATUS] Starting read images ... \n";
    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, cv::IMREAD_GRAYSCALE);
    cv::Mat depth_1 = cv::imread(depth, cv::IMREAD_GRAYSCALE);

    if (image_1.data == nullptr || image_2.data == nullptr || depth_1.data == nullptr) {
        cout << "[ERROR] Read images error !" << endl;
        return 1;
    }

    cout << "\n[STATUS] Starting extract features in image_1 ... \n";
    vector<mKeyPoint> keypoints_1;
    
    // if (!FeatureExtract(image_1, keypoints_1)) return 1;

    // random 
    cv::RNG rng;
    int nPoints = 2000;
    int border = 20;
    for (int i=0;i<2000;i++) {
        int x = rng.uniform(border, image_1.cols - border);
        int y = rng.uniform(border, image_1.rows - border);

        mKeyPoint p;
        p.SetPt(x, y);
        keypoints_1.push_back(p);
    }

    cout << "[INFO] Num of points: " << keypoints_1.size() << endl;

    cout << "\n[STATUS] Compute depth points ... \n";
    vector<Vector3d> points;
    for (mKeyPoint p: keypoints_1) {
        double x = p.GetPt().first;
        double y = p.GetPt().second;

        double z = fx * baseline / depth_1.at<uchar>(y, x);
        points.push_back(Vector3d((x - cx) * z / fx, (y - cy) * z / fy, z));
    }

    cout << "\n[STATUS] Direct Method ... \n";
    Sophus::SE3d T;

    for (int i=1;i<6;i++) {
        cv::Mat next_img = cv::imread((fmt_others % i).str(), cv::IMREAD_GRAYSCALE);
        DirectMulti(image_1, next_img, points, T, 1000);
        DrawKeypoints("image_1", image_1, keypoints_1);
        cv::waitKey(0);
    }

    return 0;
}