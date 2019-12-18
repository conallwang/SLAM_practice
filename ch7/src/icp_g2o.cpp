#include <iostream>
#include <string.h>
#include <cstring>

#include "orb_utils.hpp"

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

typedef Matrix<double, 6, 1> Vector6d;

class PoseVertex : public g2o::BaseVertex<6, Sophus::SE3d> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    virtual void oplusImpl(const double* update) override {
        Vector6d update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
    }

    virtual bool read(istream& in) override {}
    virtual bool write(ostream& out) const override {}
};

class EdgeProjection : public g2o::BaseUnaryEdge<3, Vector3d, PoseVertex> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    EdgeProjection(Vector3d pos) : _pos3d(pos) {}

    virtual void computeError() override {
        PoseVertex* v = static_cast<PoseVertex *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        _error = _measurement - T * _pos3d;
    }

    virtual void linearizeOplus() override {
        PoseVertex* v = static_cast<PoseVertex *>(_vertices[0]);
        Sophus::SE3d T = v->estimate();
        _jacobianOplusXi.block<3, 3>(0, 0) = -Matrix3d::Identity();
        _jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(T * _pos3d);
    }

    virtual bool read(istream& in) override {}
    virtual bool write(ostream& out) const override {}

    private:
    Vector3d _pos3d;
};

string img_1 = "/home/johnson/SLAM/ch7/pics/rgbd/1.png";
string img_2 = "/home/johnson/SLAM/ch7/pics/rgbd/2.png";
string d_1 = "/home/johnson/SLAM/ch7/pics/depth/1_depth.png";
string d_2 = "/home/johnson/SLAM/ch7/pics/depth/2_depth.png";

Vector2d pixel2cam(Point p, cv::Mat K) {
    return Vector2d(
        (p.first - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.second - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

void ICP_G2O(vector<Vector3d> points3d_1, vector<Vector3d> points3d_2, Matrix3d& R, Vector3d& t) {
    if (points3d_1.size() != points3d_2.size()) return;
    
    typedef g2o::BlockSolverX BlockSovlerType;
    typedef g2o::LinearSolverDense<BlockSovlerType::PoseMatrixType> LinearSovlerType;

    auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<BlockSovlerType>(g2o::make_unique<LinearSovlerType>())
    );

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // vertex
    PoseVertex* pose_vertex = new PoseVertex();
    pose_vertex->setId(0);
    pose_vertex->setEstimate(Sophus::SE3d());
    optimizer.addVertex(pose_vertex);

    // edges
    int p_size = points3d_2.size();
    for (int i=0;i<p_size;i++) {
        auto p3d_1 = points3d_1[i];
        auto p3d_2 = points3d_2[i];
        EdgeProjection* edge = new EdgeProjection(p3d_2);
        edge->setVertex(0, pose_vertex);
        edge->setMeasurement(p3d_1);
        edge->setInformation(Matrix3d::Identity());
        optimizer.addEdge(edge);
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    cout << "[INFO] pose estimated by g2o: \n" << pose_vertex->estimate().matrix() << endl;
    
    R = pose_vertex->estimate().rotationMatrix();
    t = pose_vertex->estimate().translation();

    cout << "\n[INFO] R = \n" << R << endl;
    cout << "\n[INFO] t = \n" << t << endl;
}

int main(int argc, char* argv[]) {

    if (argc > 1) {
        img_1 = argv[1];
        img_2 = argv[2];
        d_1 = argv[3];
        d_2 = argv[4];
    }

    printf("[INFO] IMAGE_PATH_1: %s\n", img_1.c_str());
    printf("[INFO] IMAGE_PATH_2: %s\n", img_2.c_str());
    printf("[INFO] DEPTH_PATH_1: %s\n", d_1.c_str());
    printf("[INFO] DEPTH_PATH_2: %s\n", d_2.c_str());

    printf("\n[STATUS] Starting read images ... \n");
    cv::Mat image_1 = cv::imread(img_1, cv::IMREAD_GRAYSCALE);
    cv::Mat image_2 = cv::imread(img_2, cv::IMREAD_GRAYSCALE);
    cv::Mat depth_1 = cv::imread(d_1, cv::IMREAD_UNCHANGED);
    cv::Mat depth_2 = cv::imread(d_2, cv::IMREAD_UNCHANGED);

    printf("\n[STATUS] Starting Feature Extracting ...\n");
    vector<mKeyPoint> keypoints_1, keypoints_2;
    if (!FeatureExtract(image_1, keypoints_1)) return 1;
    if (!FeatureExtract(image_2, keypoints_2)) return 1;

    printf("\n[STATUS] Starting Compute ORB descriptors ... \n");
    vector<Descriptor> descriptors_1, descriptors_2;
    ComputeORB(image_1, keypoints_1, descriptors_1);
    ComputeORB(image_2, keypoints_2, descriptors_2);

    printf("\n[STATUS] Starting Matching ... \n");
    vector<cv::DMatch> matches;
    Match(descriptors_1, descriptors_2, matches, 35);

    printf("\n[STATUS] Starting compute 3D points ... \n");
    vector<Vector3d> points3d_1, points3d_2;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    for (cv::DMatch m: matches) {
        float dd_1 = depth_1.at<unsigned short>(keypoints_1[m.queryIdx].GetPt().second, keypoints_1[m.queryIdx].GetPt().first);
        float dd_2 = depth_2.at<unsigned short>(keypoints_2[m.trainIdx].GetPt().second, keypoints_2[m.trainIdx].GetPt().first);
        if (dd_1 == 0 || dd_2 == 0) continue;

        double d_1 = dd_1 / 5000.0;
        double d_2 = dd_2 / 5000.0;

        Vector2d p2d_1 = pixel2cam(keypoints_1[m.queryIdx].GetPt(), K);
        Vector2d p2d_2 = pixel2cam(keypoints_2[m.trainIdx].GetPt(), K);

        points3d_1.push_back(Vector3d(p2d_1[0] * d_1, p2d_1[1] * d_1, d_1));
        points3d_2.push_back(Vector3d(p2d_2[0] * d_2, p2d_2[1] * d_2, d_2));
    }

    printf("\n[INFO] points num: %d\n", (int)points3d_2.size());

    Matrix3d R;
    Vector3d t;
    ICP_G2O(points3d_1, points3d_2, R, t);

    return 0;
}