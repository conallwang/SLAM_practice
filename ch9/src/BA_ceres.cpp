#include <iostream>

#include <opencv2/opencv.hpp>

#include <ceres/ceres.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "rotation.h"
#include "random.h"
#include "common.h"

using namespace std;
using namespace Eigen;


class CamProjectionError {
    private:
    double observation_x, observation_y;

    public:
    // 构造函数
    CamProjectionError(double x, double y) : observation_x(x), observation_y(y) {}

    // 定义误差
    template<typename T>
    bool operator() (const T* const camera, const T* const point, T* residual) const {
        T prediction[2];
        // 通过相机投影预测像素坐标
        ComputeProjectionAxis(camera, point, prediction);

        residual[0] = prediction[0] - observation_x;
        residual[1] = prediction[1] - observation_y;

        return true;
    }

    // 通过camera，计算point的投影像素坐标
    // 其中camera为长度为9的向量
    // camera[0:2]: 相机旋转向量
    // camera[3:5]: 相机的平移向量
    // camera[6:8]: 相机参数，camera[6]为焦距，camera[7:8]为畸变参数
    // 其中point为三维坐标点
    template<typename T>
    static bool ComputeProjectionAxis(const T* const camera, const T* const point, T* prediction) {
        T p[3];
        // 将point经旋转向量旋转，平移，变到当前相机坐标系
        AngleAxisRotatePoint(camera, point, p);
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // 归一化
        prediction[0] = - p[0] / p[2];
        prediction[1] = - p[1] / p[2];

        // 畸变
        const T& l1 = camera[7];
        const T& l2 = camera[8];

        T r2 = prediction[0] * prediction[0] + prediction[1] * prediction[1];
        T distortion = T(1.0) + r2 * (l1 + l2 * r2);

        const T& focal = camera[6];
        prediction[0] *= focal * distortion;
        prediction[1] *= focal * distortion;

        return true;
    }

    static ceres::CostFunction *Create (const double observe_x, const double observe_y) {
        return (new ceres::AutoDiffCostFunction<CamProjectionError, 2, 9, 3>(new CamProjectionError(observe_x, observe_y)));
    }
};

// ceres BA
void SolveBA(BALProblem& bal_problem);

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printf("USAGE: BA_ceres [BAL_DATA]\n");
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
    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    double *points = bal_problem.mutable_points();
    double *cameras = bal_problem.mutable_cameras();

    const double* observations = bal_problem.observations();
    ceres::Problem problem;
    
    for (int i=0;i<bal_problem.num_observations();i++) {
        // Cost Function
        ceres::CostFunction *costFunction = CamProjectionError::Create(observations[2*i], observations[2*i + 1]);

        // Loss Function
        ceres::LossFunction *lossFunction = new ceres::HuberLoss(1.0);

        // Parameter
        double* camera = cameras + camera_block_size * bal_problem.camera_index()[i];
        double* point = points + point_block_size * bal_problem.point_index()[i];

        problem.AddResidualBlock(costFunction, lossFunction, camera, point);
    }

    cout << "\nbal problem has " << bal_problem.num_cameras() << " cameras and " << bal_problem.num_points() << " points." << endl;
    cout << "Forming " << bal_problem.num_observations() << " observations. " << endl;

    cout << "Solving ceres BA ..." << endl;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

}