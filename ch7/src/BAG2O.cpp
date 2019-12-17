#include <iostream>
#include <string.h>
#include <cstring>

#include "orb_utils.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <sophus/se3.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";

string depth = "/home/johnson/SLAM/ch7/pics/depth/1_depth.png";

// Vertex and Edge
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    virtual void oplusImpl(const double *update) override {
        Vector6d update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(istream& in) override {}
    virtual bool write(ostream& out) const override {}
};

class EdgeProjection : public g2o::BaseUnaryEdge<2, Vector2d, VertexPose> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjection(const Vector3d& pos, const Matrix3d& K) : _pos3d(pos), _K(K) {}

    virtual void computeError() {
        const VertexPose* v = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Vector3d pos_pixel = _K * (T * _pos3d);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() override {
        const VertexPose* v = static_cast<VertexPose *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        Vector3d pos_cam = T * _pos3d;
        double fx = _K(0, 0);
        double fy = _K(1, 1);
        double cx = _K(0, 2);
        double cy = _K(1, 2);
        double x = pos_cam[0];
        double y = pos_cam[1];
        double inv_z = 1 / pos_cam[2];
        double inv_z2 = inv_z * inv_z;
        _jacobianOplusXi 
        <<  -fx * inv_z, 0, fx * x * inv_z2, fx * x * y * inv_z2, -fx - (fx * x * x * inv_z2), fx * y * inv_z,
            0, -fy * inv_z, fy * y * inv_z2, fy + fy * y * y * inv_z2, -fy * x * y * inv_z2, -fy * x * inv_z;

    }

    virtual bool read(istream& in) override {}
    virtual bool write(ostream& out) const override {}

    private:
    Vector3d _pos3d;
    Matrix3d _K;
};

void BAG2O(vector<cv::Point3f> points3d, vector<cv::Point2f> points2d, cv::Mat K, Sophus::SE3d pose) {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    // 
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>())
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // vertex
    VertexPose* vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Sophus::SE3d());
    optimizer.addVertex(vertex_pose);

    // K
    Matrix<double, 3, 3> K_eigen;
    K_eigen <<  K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
                K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
                K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

    // edges
    int index = 1;
    int p_size = points2d.size();
    for (size_t i=0;i<p_size;i++) {
        auto p2d = points2d[i];
        auto p3d = points3d[i];
        EdgeProjection* edge = new EdgeProjection(Vector3d(p3d.x, p3d.y, p3d.z), K_eigen);
        edge->setId(index);
        edge->setVertex(0, vertex_pose);
        edge->setMeasurement(Vector2d(p2d.x, p2d.y));
        edge->setInformation(Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    cout << "[INFO] pose estimated by g2o: \n" << vertex_pose->estimate().matrix() << endl;
    pose = vertex_pose->estimate();
}

int main(int argc, char* argv[]) {

    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
        depth = argv[3];
    }

    printf("[INFO] IMAGE_PATH_1: %s", img_1.c_str());
    printf("[INFO] IMAGE_PATH_2: %s", img_2.c_str());
    printf("[INFO] DEPTH_PATH: %s", depth.c_str());

    printf("[STATUS] Starting read images ... \n");
    
    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, cv::IMREAD_GRAYSCALE);
    cv::Mat depth_1 = cv::imread(depth, cv::IMREAD_UNCHANGED);

    if (image_1.data == nullptr || image_2.data == nullptr || depth_1.data == nullptr) {
        printf("[ERROR] Cannot open image. \n");
        printf("        IMAGE_1: %s\n", img_1.c_str());
        printf("        IMAGE_2: %s\n", img_2.c_str());
        printf("        DEPTH: %s\n", depth.c_str());
        printf("EXIT!\n");
        return 1;
    }

    // Show depth
    // cout << "[INFO] DEPTH_IMAGE: \n" << depth_1 << endl;

    printf("\n[STATUS] Starting Feature Extract ... \n");
    vector<mKeyPoint> keypoints_1, keypoints_2;
    if (!FeatureExtract(image_1, keypoints_1)) return 1;
    if (!FeatureExtract(image_2, keypoints_2)) return 2;

    printf("\n[STATUS] Starting Compute ORB descriptors ... \n");
    vector<Descriptor> descriptors_1, descriptors_2;
    ComputeORB(image_1, keypoints_1, descriptors_1);
    ComputeORB(image_2, keypoints_2, descriptors_2);

    printf("\n[STATUS] Starting Matching ... \n ");
    vector<cv::DMatch> matches;
    Match(descriptors_1, descriptors_2, matches, 35);

    // Compute 3D position according to depth_1
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    vector<cv::Point3f> points3d;
    vector<cv::Point2f> points2d;
    for (cv::DMatch m: matches) {
        cv::Point3f point3d;
        cv::Point2f point2d;
        ushort d = depth_1.ptr<unsigned short>((int)keypoints_1[m.queryIdx].GetPt().second)[(int)keypoints_1[m.queryIdx].GetPt().first];
        if (d == 0)
            continue;
        float dd = d / 5000.0;
        point3d.x = ((keypoints_1[m.queryIdx].GetPt().first - K.at<double>(0, 2)) / K.at<double>(0, 0)) * dd;
        point3d.y = ((keypoints_1[m.queryIdx].GetPt().second - K.at<double>(1, 2)) / K.at<double>(1, 1)) * dd;
        point3d.z = dd;
        point2d.x = keypoints_2[m.trainIdx].GetPt().first;
        point2d.y = keypoints_2[m.trainIdx].GetPt().second;
        cout << point3d << endl;
        cout << point2d << endl;
        points3d.push_back(point3d);
        points2d.push_back(point2d);
    }

    cout << "[INFO] 3d-2d pairs: " << points3d.size() << endl;

    Sophus::SE3d pose_gn;
    BAG2O(points3d, points2d, K, pose_gn);

    return 0;
}