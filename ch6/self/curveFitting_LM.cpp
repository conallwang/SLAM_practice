#include <iostream>
#include <chrono>
using namespace std;

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

// arguments of function
int a = 0;
int b = 0;
int c = 0;

// sigma of RNG::Gaussion
double w_sigma = 1.0;

// arguments of iteration
int N = 100;
double start = 0.0;
double finish = 1.0;
int num_iter = 1000;

double f(double a, double b, double c, double x) {
    return exp(a*x*x + b*x + c);
}

void printUsage() {
    printf("USAGE: ./curveFitting_LM a=[SELF_ARG] b=[SELF_ARG] c=[SELF_ARG] [ARGS]\n\n");
    printf("ARGUMENTS: \n");
    printf("    a           args of function exp(ax^2+bx+c)\n");
    printf("    b           args of function exp(ax^2+bx+c)\n");
    printf("    c           args of function exp(ax^2+bx+c)\n");
    printf("    sigma       args of RNG::Gaussion\n");
    printf("    N           num of data\n");
    printf("    start       start of data\n");
    printf("    finish      finish of data\n");
    printf("    iter        num of iteration\n");
    return ;
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
        return ;
    }

    if (1==sscanf(arg, "c=%f", &foption)) {
        c = foption;
        return ;
    }

    if (1==sscanf(arg, "sigma=%f", &foption)) {
        w_sigma = foption;
        printf("[STATUS] SIGMMA: %f\n", w_sigma);
        return ;
    }
    
    if (1==sscanf(arg, "N=%d", &option)) {
        N = option;
        printf("[STATUS] N: %d\n", N);
        return ;
    }

    if (1==sscanf(arg, "start=%f", &foption)) {
        start = foption;
        printf("[STATUS] START: %f\n", start);
        return ;
    }

    if (1==sscanf(arg, "finish=%f", &foption)) {
        finish = foption;
        printf("[STATUS] FINISH: %f\n", finish);
        return ;
    }

    if (1==sscanf(arg, "iter=%d", &option)) {
        num_iter = option;
        printf("[STATUS] NUM_ITER: %d\n", num_iter);
        return ;
    }

}

int main(int argc, char* argv[]) {

    if (argc == 1) {
        printUsage();
        return 1;
    }

    // Parse Argument
    for (int i=1;i<argc;i++) {
        parseArgument(argv[i]);
    }

    cv::RNG rng;
    vector<double> x_data, y_data;
    
    double tmpl = start;
    double step = (finish - start)/(double)N;
    while (tmpl <= finish) {
        x_data.push_back(tmpl);
        y_data.push_back(exp(a*tmpl*tmpl + b*tmpl + c) + rng.gaussian(w_sigma * w_sigma));
        tmpl += step;
    }

    printf("[STATUS] Generate Data Finish !\n");
    printf("[INFO] x_data size: %d\n", x_data.size());

    // iteration
    Vector3d arg(2.0, -1.0, 5.0);

    double cost = 0;
    double lambda = 0.5;
    Matrix3d I = Matrix3d::Ones();

    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    for (int epoch=0;epoch<num_iter;epoch++) {
        int d_size = x_data.size();
        Matrix3d JJT = Matrix3d::Zero();
        Vector3d Je(0, 0, 0);
        vector<Vector3d, Eigen::aligned_allocator<Vector3d>> J_total;
        cost = 0;

        for (int i=0;i<d_size;i++) {
            double x = x_data[i];
            double y = y_data[i];

            Vector3d J;
            J[0] = -x*x*exp(arg[0]*x*x + arg[1]*x + arg[2]);
            J[1] = -x*exp(arg[0]*x*x + arg[1]*x + arg[2]);
            J[2] = -exp(arg[0]*x*x + arg[1]*x + arg[2]);

            double error = y - exp(arg[0]*x*x + arg[1]*x + arg[2]);
            cost += error * error;

            JJT += J*J.transpose() + lambda * I;
            Je += -J*error;
            J_total.push_back(J);
        }

        Vector3d da = JJT.inverse() * Je;
        Vector3d next_arg = arg + da;
        double real_diff = 0.0;
        double esti_diff = 0.0;
        for (int i=0;i<d_size;i++) {
            real_diff += (f(next_arg[0], next_arg[1], next_arg[2], x_data[i]) - f(arg[0], arg[1], arg[2], x_data[i]));
            esti_diff += J_total[i].transpose() * da;
        }

        double rou = real_diff / esti_diff;
        printf("[INFO] iter = %d, ρ = %f\n", epoch, rou);

        if (rou >= 0.95) {
            break;
        }

        if (rou > 0.5) {
            lambda *= 0.75;
        }
        else {
            lambda *= 1.25;
        }

        arg = next_arg;
    }

    chrono::steady_clock::time_point finish_time = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(finish_time - start_time);

    cout << "time cost = " << time_used.count() << " seconds" << endl;

    printf("Opt algrithm finish !\n");
    cout << "Final Result: " << arg.transpose() << endl;

    return 0;
}