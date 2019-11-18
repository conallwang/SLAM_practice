#include <iostream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

struct CURVE_FITTING_COST {
    const double _x, _y;

    CURVE_FITTING_COST(double x, double y): _x(x), _y(y) {}

    template<typename T> 
    bool operator ()(
        const T* const abc,
        T* residual
    ) const {
        residual[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x) + abc[1]*T(_x) + abc[2]);
        return true;
    }
};

// arguments of function
int a = 0;
int b = 0;
int c = 0;

double w_sigma = 1.0;

// 
int N = 100;
double start = 0.0; 
double finish = 1.0;

void printUsage() {
    printf("USAGE: ./curveFitting_ceres a=[SELF_ARG] b=[SELF_ARG] c=[SELF_ARG] [ARGS]\n\n");
    printf("ARGUMENTS: \n");
    printf("    a           args of func exp(ax^2+bx+c)\n");
    printf("    b           args of func exp(ax^2+bx+c)\n");
    printf("    c           args of func exp(ax^2+bx+c)\n");
    return ;
}

void parseArguments(char* arg) {
    char buf[1000];
    int option;
    float foption;

    if (1==sscanf(arg, "a=%f", &foption)) {
        a = foption;
        return ;
    }
    
    if (1==sscanf(arg, "b=%f", &foption)) {
        b = foption;
        return ;
    }

    if (1==sscanf(arg, "c=%f", &foption)) {
        c = foption;
        return ;
    }
}

int main(int argc, char* argv[]) {
    
    if (argc == 1) {
        printUsage();
        return 1;
    }

    for (int i=1;i<argc;i++) parseArguments(argv[i]);

    printf("[STATUS] Generating Data ...\n");

    cv::RNG rng;
    vector<double> x_data, y_data;

    double tmpl = start;
    double step = (finish - start) / (double)N;
    while (tmpl < finish) {
        x_data.push_back(tmpl);
        y_data.push_back(exp(a*tmpl*tmpl + b*tmpl + c) + rng.gaussian(w_sigma * w_sigma));
        tmpl += step;
    }

    printf("[STATUS] Data has been generated !\n\n");

    printf("[STATUS] Constructing Ceres Problem ... \n");
    int d_size = x_data.size();

    double abc[3] = {2.0, -1.0, 5.0};
    ceres::Problem problem;
    for (int i=0;i<d_size;i++) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
                new CURVE_FITTING_COST(x_data[i], y_data[i])
            ),
            nullptr,
            abc
        );
    }
    
    printf("[STATUS] Problem has been constructed !\n\n");

    printf("[STATUS] Configuring Ceres Solver ... \n");
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    printf("[STATUS] Configure Done !\n\n");

    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();

    printf("[STATUS] Start to solve ... \n");
    ceres::Solve(options, &problem, &summary);

    chrono::steady_clock::time_point finish_time = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);

    cout << "[INFO] Solve time cost = " << time_used.count() << endl;
    
    // output results
    cout << summary.BriefReport() << endl;
    cout << "[INFO] Estimated abc: ";
    for (auto &a:abc) cout << a << " ";
    cout << endl;

    return 0;
}