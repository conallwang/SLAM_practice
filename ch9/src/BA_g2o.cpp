#include <iostream>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "common.h"

using namespace std;
using namespace Eigen;

// 相机参数为长度为9的向量
// [0:2]: 旋转向量
// [3:5]: 平移向量
// [6]:   相机焦距
// [7:8]: 畸变参数  
struct PoseAndIntrinsics {
    public:
    Sophus::SO3d rotation;
    Vector3d t;
    double focal;
    double k1, k2;

    // 构造函数
    PoseAndIntrinsics() {};

    PoseAndIntrinsics(double* camera) {
        rotation = Sophus::SO3d::exp(Vector3d(camera[0], camera[1], camera[2]));
        t = Vector3d(camera[3], camera[4], camera[5]);
        focal = camera[6];
        k1 = camera[7];
        k2 = camera[8];
    }

    // 恢复为原结构数组
    void set_to(double* camera) {
        auto r = rotation.log();
        for (int i = 0; i < 3; ++i) camera[i] = r[i];
        for (int i = 0; i < 3; ++i) camera[i + 3] = t[i];
        camera[6] = focal;
        camera[7] = k1;
        camera[8] = k2;
    }
};

class VertexPoseAndIntrinsics : public g2o::BaseVertex<9, PoseAndIntrinsics> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 构造函数
    VertexPoseAndIntrinsics() {};

    // 顶点初始化
    virtual void setToOriginImpl() override {
        _estimate = PoseAndIntrinsics();
    }

    // 顶点更新
    virtual void oplusImpl(const double* update) override {
        _estimate.rotation = Sophus::SO3d::exp(Vector3d(update[0], update[1], update[2])) * _estimate.rotation;
        _estimate.t = _estimate.t + Vector3d(update[3], update[4], update[5]);
        _estimate.focal += update[6];
        _estimate.k1 += update[7];
        _estimate.k2 += update[8];
    } 

    // 根据相机参数投影
    Vector2d projection(Vector3d point3d) {
        // 变换到相机坐标系
        Vector3d p = _estimate.rotation * point3d + _estimate.t;

        // 归一化
        Vector2d res;
        res[0] = - p[0] / p[2];
        res[1] = - p[1] / p[2];

        // 畸变
        double r = res[0] * res[0] + res[1] * res[1];
        double distortion = 1.0 + r * (_estimate.k1 + _estimate.k2 * r);

        return Vector2d(
            _estimate.focal * distortion * res[0],
            _estimate.focal * distortion * res[1]
        );
    }

    virtual bool read(istream& in) {};
    virtual bool write(ostream& out) const {};

};

class VertexPoints : public g2o::BaseVertex<3, Vector3d> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // 构造函数
    VertexPoints() {};

    // 顶点初始化
    virtual void setToOriginImpl() override {
        _estimate = Vector3d(0, 0, 0);
    }

    // 顶点更新
    virtual void oplusImpl(const double* update) override {
        _estimate += Vector3d(update[0], update[1], update[2]);
    }

    virtual bool read(istream& in) {};
    virtual bool write(ostream& out) const {};
};

class EdgeProjection : public g2o::BaseBinaryEdge<2, Vector2d, VertexPoseAndIntrinsics, VertexPoints> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void computeError() override {
        auto v0 = (VertexPoseAndIntrinsics *) _vertices[0];
        auto v1 = (VertexPoints *) _vertices[1];
        auto proj = v0->projection(v1->estimate());

        _error = proj - _measurement;
    }

    virtual bool read(istream& in) {};
    virtual bool write(ostream& out) const {};
};

void SolveBA(BALProblem& bal_problem);

int main(int argc, char* argv[]) {
    if (argc < 2) {
        cout << "USAGE: BA_g2o [BAL_DATA]" << endl;
        return 1;
    }

    BALProblem bal_problem(argv[1]);
    bal_problem.Normalize();
    bal_problem.Perturb(0.1, 0.5, 0.5);
    bal_problem.WriteToPLYFile("initial.ply");
    SolveBA(bal_problem);
    bal_problem.WriteToPLYFile("final.ply");

    return 0;
}

void SolveBA(BALProblem& bal_problem) {
    const int camera_block_size = bal_problem.camera_block_size();
    const int point_block_size = bal_problem.point_block_size();
    double* cameras = bal_problem.mutable_cameras();
    double* points = bal_problem.mutable_points();

    // pose dimention 9, landmark is 3
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    const double* observations = bal_problem.observations();

    // 声明相机顶点和路标顶点
    vector<VertexPoseAndIntrinsics *> vertex_cameras;
    vector<VertexPoints *> vertex_points;
    for (int i=0;i<bal_problem.num_cameras();i++) {
        VertexPoseAndIntrinsics *v = new VertexPoseAndIntrinsics();
        double* camera = cameras + camera_block_size * i;
        v->setId(i);
        v->setEstimate(PoseAndIntrinsics(camera));
        optimizer.addVertex(v);
        vertex_cameras.push_back(v);
    }

    for (int i=0;i<bal_problem.num_points();i++) {
        VertexPoints *v = new VertexPoints();
        double* point = points + point_block_size * i;
        v->setId(i + bal_problem.num_cameras());
        v->setEstimate(Vector3d(point[0], point[1], point[2]));
        v->setMarginalized(true);
        optimizer.addVertex(v);
        vertex_points.push_back(v);
    }

    // 声明边
    for (int i=0;i<bal_problem.num_observations();i++) {
        EdgeProjection* edge = new EdgeProjection();
        edge->setVertex(0, vertex_cameras[bal_problem.camera_index()[i]]);
        edge->setVertex(1, vertex_points[bal_problem.point_index()[i]]);
        edge->setMeasurement(Vector2d(observations[2*i], observations[2*i + 1]));
        edge->setInformation(Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(50);

    // set to bal_problem
    for (int i=0;i<bal_problem.num_cameras();i++) {
        double* camera = cameras + camera_block_size * i;
        auto vertex = vertex_cameras[i];
        auto estimate = vertex->estimate();
        estimate.set_to(camera);
    }

    for (int i=0;i<bal_problem.num_points();i++) {
        double* point = points + point_block_size * i;
        auto vertex = vertex_points[i];
        auto estimate = vertex->estimate();
        for (int j=0;j<3;j++) point[j] = estimate[j];
    }
}

