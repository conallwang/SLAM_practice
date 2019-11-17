#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include <fstream>
#include <iomanip>
using namespace std;

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;

// Initialize function, no this func
// need to analyze function
string func = "";

// three arguments ax^2 + bx + c
int a = 0;
int b = 0;
int c = 0;

// where to save generated (x, y) data
string savefile = "";

// Arguments aboud generate data
int N = 100;                // num of data
double start = 0;          // begin of data
double finish = 1;          // end of data
double w_sigmma = 1.0;      // sigmma of guassion error

// iterate
int num_iter = 1000;        // num of loop
double receive_err = 1e-5;
double learning_rate = 1.0;

void printUsage() {
    printf("USAGE: ./curveFitting_GN a=[SELF_ARG] b=[SELF_ARG] c=[SELF_ARG] [savefile=[SAVE_FILE_PATH]]\n\n");
    printf("ARGUMENTS: \n");
    printf("    -h          print user help\n");
    printf("    a           set argument a\n");
    printf("    b           set argument b\n");
    printf("    c           set argument c\n");
    printf("    N           num of data\n");
    printf("    start       begin of data\n");
    printf("    finish      end of data\n");
    printf("    savefile    set save file path\n");
    printf("    iter        num of loop \n");
    printf("    lr          like learning rate in Machine Learning\n");
    return ;
}

void parseArgument(char* arg) {
    char buf[1000];
    int option;
    float foption;

    if (1==sscanf(arg, "func=%s", buf)) {
        printf("[Warning] This function will be added later ...");
        return;
    }

    if (1==sscanf(arg, "a=%f", &foption)) {
        a = foption;
        return;
    }

    if (1==sscanf(arg, "b=%f", &foption)) {
        b = foption;
        return;
    }

    if (1==sscanf(arg, "c=%f", &foption)) {
        c = foption;
        return;
    }

    if (1==sscanf(arg, "N=%d", &option)) {
        N = option;
        printf("[STATUS] N: %d\n", N);
        return;
    }
    
    if (1==sscanf(arg, "start=%f", &foption)) {
        start = foption;
        printf("[STATUS] START: %f\n", start);
        return;
    }

    if (1==sscanf(arg, "finish=%f", &foption)) {
        finish = foption;
        printf("[STATUS] FINISH: %f\n", finish);
        return;
    }

    if (1==sscanf(arg, "sigmma=%f", &foption)) {
        w_sigmma = foption;
        printf("[STATUS] W_SIGMMA: %f\n", w_sigmma);
        return;
    }

    if (1==sscanf(arg, "savefile=%s", buf)) {
        savefile = buf;
        printf("[STATUS] SAVE_FILE: %s\n", savefile.c_str());
        return;
    }

    if (1==sscanf(arg, "iter=%d", &option)) {
        num_iter = option;
        printf("[STATUS] MAX_ITER: %d\n", num_iter);
        return;
    }

    if (1==sscanf(arg, "r_err=%f", &foption)) {
        receive_err = foption;
        printf("[STATUS] RECEIVE_ERROR: %f", foption);
        return;
    }

    if (1==sscanf(arg, "lr=%f", &foption)) {
        learning_rate = foption;
        printf("[STATUS] LEARNING_RATE: %f", learning_rate);
        return;
    }

    if (1==sscanf(arg, "-h")) {
        printUsage();
        return;
    }
}

int main(int argc, char* argv[]) {

    if (argc == 1) {
        printUsage();
        return 1;
    }

    for (int i=1;i<argc;i++) {
        parseArgument(argv[i]);
    }

    double inv_sigmma = 1.0 / w_sigmma;

    printf("[STATUS] FUNCTION: exp(%dx^2+%dx+%d) + w\n", a, b, c);
    printf("\n[STATUS] Generate data ...\n");

    cv::RNG rng;
    vector<double> x_data, y_data;
    double tmpl = start;
    double tmpr = finish;
    double step = (finish - start) / N;
    if (step <= 0) {
        printf("[Warning] please check if your start > finish ? \n");
    }
    while (tmpl < tmpr) {
        x_data.push_back(tmpl);
        y_data.push_back(exp(a*tmpl*tmpl + b*tmpl + c) + rng.gaussian(w_sigmma * w_sigmma));
        tmpl += step;
    }

    printf("[STATUS] Generate Finish!\n");
    if (savefile != "") {
        ofstream ofile(savefile.c_str());

        if (!ofile) {
            printf("[ERROR] Cannot open file: %s\n", savefile.c_str());
            printf("[ERROR] Failure in saving data. Skip\n");
        }
        else {
            int d_size = x_data.size();
            for (int i=0;i<d_size;i++) {
                ofile << setw(8);
                ofile << x_data[i] << "\t" << y_data[i] << endl;
            }
            printf("[STATUS] Data file has been saved in %s\n", savefile.c_str());
        }
    }

    // set initial value of [a, b, c]
    Vector3d arg(2.0, -1.0, 5.0);

    // with batch 
    double cost = 0;
    double last_cost = 0;
    for (int epoch=0;epoch<num_iter;epoch++) {
        int d_size = x_data.size();
        Matrix3d JJT = Matrix3d::Zero();
        Vector3d Je(0, 0, 0);

        cost = 0.0;
        for (int i=0;i<d_size;i++) {
            double x = x_data[i];
            double y = y_data[i];
        
            Vector3d J;
            J[0] = -x*x*exp(arg[0]*x*x + arg[1]*x + arg[2]);
            J[1] = -x*exp(arg[0]*x*x + arg[1]*x + arg[2]);
            J[2] = -exp(arg[0]*x*x + arg[1]*x + arg[2]);
            
            double e = y - exp(arg[0]*x*x + arg[1]*x + arg[2]);
            
            cost += e * e;
            // JJT += inv_sigmma * inv_sigmma * J*J.transpose();
            // Je += -inv_sigmma * inv_sigmma * J*e;

            JJT += inv_sigmma * inv_sigmma * J*J.transpose();
            Je += -inv_sigmma * inv_sigmma * J*e;
        }
        
        Vector3d dx = JJT.inverse() * Je;
        // cout << dx.transpose() << endl;
        arg += learning_rate * dx;
        if (abs(cost - last_cost) <= receive_err) {
            break;
        }
        last_cost = cost;
        printf("[INFO] iter = %d, cost = %f\n", epoch, cost);
    }

    cout << "Opt Finish!" << endl;
    cout << "Final Result: " << arg.transpose() << endl;

    return 0;
}