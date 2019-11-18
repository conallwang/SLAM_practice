#include <iostream>
#include <chrono>
#include <cmath>
using namespace std;

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>

class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // reset
    virtual void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }

    // update
    virtual void oplusImpl(const double* update) override {
        _estimate += Eigen::Vector3d(update);
    }

    // 
    virtual bool read(istream &in) {};
    virtual bool write(ostream &out) const {};

};

class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

    virtual void computeError() override {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex*>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        _error(0, 0) = _measurement - std::exp(abc(0, 0)*_x*_x + abc(1, 0)*_x + abc(2, 0));
    }

    virtual void linearizeOplus() override {
        const CurveFittingVertex *v = static_cast<const CurveFittingVertex *>(_vertices[0]);
        const Eigen::Vector3d abc = v->estimate();
        double y = exp(abc[0]*_x*_x + abc[1]*_x + abc[2]);
        _jacobianOplusXi[0] = -_x*_x*y;
        _jacobianOplusXi[1] = -_x*y;
        _jacobianOplusXi[2] = -y;
    }

    virtual bool read(istream &in) {}
    virtual bool write(ostream &out) const {}
public:
    double _x;
};

// args of function
int a = 0;
int b = 0;
int c = 0;

int N = 100;
double start = 0.0;
double finish = 1.0;

double w_sigma = 1.0;

void printUsage() {
    printf("USAGE: ./curveFitting_g2o a=[SELF_ARG] b=[SELF_ARG] c=[SELF_ARG] [ARGS]\n\n");
    printf("ARGUMENTS: \n");
    printf("    a           args of func exp(ax^2+bx+c)\n");
    printf("    b           args of func exp(ax^2+bx+c)\n");
    printf("    c           args of func exp(ax^2+bx+c)\n");
}

void parseArgument(char* arg) {
    char buf[1000];
    int option;
    float foption;

    if (1==sscanf(arg, "a=%f", &foption)) {
        a = foption;
        return ;
    }

    if (1==sscanf(arg, "b=%f", &foption)) {
        b = foption;
        return;
    }
    
    if (1==sscanf(arg, "c=%f", &foption)) {
        c = foption;
        return;
    }
}

int main(int argc, char* argv[]) {

    if (argc == 1) {
        printUsage();
        return 1;
    }

    for (int i=1;i<argc;i++) parseArgument(argv[i]);

    cv::RNG rng;    
    vector<double> x_data, y_data;
    
    double tmpl = start;
    double step = (finish - start) / N;
    while (tmpl < finish) {
        x_data.push_back(tmpl);
        y_data.push_back(exp(a*tmpl*tmpl+b*tmpl+c) + rng.gaussian(w_sigma * w_sigma));
        tmpl += step;
    }

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmGaussNewton(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate(Eigen::Vector3d(2.0, -1.0, 5.0));
    v->setId(0);
    optimizer.addVertex(v);

    for (int i=0;i<N;i++) {
        CurveFittingEdge* edge = new CurveFittingEdge(x_data[i]);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(y_data[i]);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma));
        optimizer.addEdge(edge);
    }

    cout << "[STATUS] Starting optimization ... \n" << endl;
    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point finish_time = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);
    cout << "solve time cost = " << time_used.count() << "seconds" << endl;

    Eigen::Vector3d abc_estimated = v->estimate();
    cout << "estimated model: " << abc_estimated.transpose() << endl;

    return 0;
}