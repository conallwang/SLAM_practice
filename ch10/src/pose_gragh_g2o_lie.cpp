#include <iostream>
#include <fstream>
#include <string>

#include <eigen3/Eigen/Core>

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

Matrix6d JrInv(Sophus::SE3d e) {
    Matrix6d J;
    J.block<3, 3>(0, 0) = Sophus::SO3d::hat(e.so3().log());
    J.block<3, 3>(0, 3) = Sophus::SO3d::hat(e.translation());
    J.block<3, 3>(3, 0) = Matrix3d::Zero();
    J.block<3, 3>(3, 3) = Sophus::SO3d::hat(e.so3().log());

    return J*0.5 + Matrix6d::Identity();
}

class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual bool read(istream& in) override {
        double data[7];
        for (int i=0;i<7;i++) in >> data[i];

        setEstimate(Sophus::SE3d(Quaterniond(data[6], data[3], data[4], data[5]), Vector3d(data[0], data[1], data[2])));
        return true;
    }

    virtual bool write(ostream& out) const override {
        out << id() << " ";
        Quaterniond q = _estimate.unit_quaternion();
        out << _estimate.translation().transpose() << " ";
        out << q.coeffs()[0] << " " <<  q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl;

        return true;
    }

    virtual void setToOriginImpl() override {
        _estimate = Sophus::SE3d();
    }

    virtual void oplusImpl(const double* update) override {
        Vector6d upt;
        upt << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(upt) * _estimate;
    }
};

class EdgePose : public g2o::BaseBinaryEdge<6, Sophus::SE3d, VertexPose, VertexPose> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual bool read(istream& in) override {
        double data[7];
        for (int i=0;i<7;i++) in >> data[i];

        setMeasurement(Sophus::SE3d(Quaterniond(data[6], data[3], data[4], data[5]), Vector3d(data[0], data[1], data[2])));
        for (int i=0;i<information().rows() && in.good();i++) {
            for (int j=i;j<information().cols() && in.good();j++) {
                in >> information()(i, j);
                if (i != j) information()(j, i) = information()(i, j);
            }
        }

        return true;
    }

    virtual bool write(ostream& out) const override {
        VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        VertexPose *v1 = static_cast<VertexPose *>(_vertices[1]);

        out << v0->id() << " " << v1->id() << " ";
        Quaterniond q = _measurement.unit_quaternion();
        out << _measurement.translation().transpose() << " ";
        out << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        for (int i=0;i<information().rows() && out.good();i++) {
            for (int j=i;j<information().cols() && out.good();j++) {
                out << information()(i, j) << " "; 
            }
        }
        out << endl;

        return true;
    }

    virtual void computeError() override {
        VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        VertexPose *v1 = static_cast<VertexPose *>(_vertices[1]);

        Sophus::SE3d s0 = v0->estimate();
        Sophus::SE3d s1 = v1->estimate();

        _error = (_measurement.inverse() * s0.inverse() * s1).log();
    }

    virtual void linearizeOplus() override {
        VertexPose *v0 = static_cast<VertexPose *>(_vertices[0]);
        VertexPose *v1 = static_cast<VertexPose *>(_vertices[1]);

        Matrix6d J = JrInv(Sophus::SE3d::exp(_error));
        
        _jacobianOplusXi = - J * (v1->estimate().inverse()).Adj();
        _jacobianOplusXj = J * (v1->estimate().inverse()).Adj();
    }
};

int main(int argc, char* argv[]) {

    if (argc < 2) {
        cout << "USAGE: pose_gragh_g2o_lie [G2O_DATA]. " << endl;
        return 1;
    }

    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist. " << endl;
        return 1;
    }

    // set g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>())
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    int VertexCnt = 0, EdgeCnt = 0;
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            VertexPose *v = new VertexPose();
            int id;
            fin >> id;
            v->setId(id);
            v->read(fin);
            optimizer.addVertex(v);
            VertexCnt++;
        }
        else if (name == "EDGE_SE3:QUAT") {
            EdgePose *e = new EdgePose();
            int idx1, idx2;
            fin >> idx1 >> idx2;
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->setId(EdgeCnt++);
            e->read(fin);
            optimizer.addEdge(e);
        }
        if (!fin.good()) break;
    }

    cout << "\ntotal " << VertexCnt << " vertices and " << EdgeCnt << " edges. " << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving results ... " << endl;
    optimizer.save("result_lie.g2o");

    return 0;
}