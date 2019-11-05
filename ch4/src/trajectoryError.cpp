#include <iostream>
#include <cmath>
#include <cstring>
#include <string.h>
#include <vector>
#include <unistd.h>
using namespace std;

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
using namespace Eigen;

// se3
typedef Matrix<double, 6, 1> Vector6d;

string groundtruth = "/home/johnson/SLAM/ch4/trajectory/groundtruth.txt";
string estimated = "/home/johnson/SLAM/ch4/trajectory/estimated.txt";

// Read File, return SUCCESS or FAIL
bool ReadTrajectory(string path, vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> &poses) {
  // vector to save Trajectory Isometry
  // vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
  
  ifstream fin(path);
  if (!fin) {
    return false;
  }

  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Isometry3d tmp(Quaterniond(qw, qx, qy, qz));
    tmp.pretranslate(Vector3d(tx, ty, tz));
    poses.push_back(tmp);
  }

  return true;
}

// Initiate Pangolin
void InitPangolin(pangolin::OpenGlRenderState& s_cam, pangolin::View& d_cam) {
  // Create pangolin window and plot trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  s_cam = pangolin::OpenGlRenderState(
  pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
  pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(s_cam));
}

// type 1: groundtruth 0: estimated
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses_gt, vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses_esti, pangolin::OpenGlRenderState& s_cam, pangolin::View& d_cam) {

  while(!pangolin::ShouldQuit()) {
    // Clear
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    
    int gt_size = poses_gt.size();
    for (size_t i=0;i<gt_size;i++) {
      Vector3d p1 = poses_gt[i].translation();
      Vector3d p2 = poses_gt[(i+gt_size+1)%gt_size].translation();
      
      glBegin(GL_LINES);
      glColor3f(1.0f, 0.0f, 0.0f);
      glVertex3d(p1[0], p1[1], p1[2]);
      glVertex3d(p2[0], p2[1], p2[2]);
      glEnd();
    }

    int esti_size = poses_esti.size();
    for (size_t i=0;i<esti_size;i++) {
      Vector3d p1 = poses_esti[i].translation();
      Vector3d p2 = poses_esti[(i+esti_size+1)%esti_size].translation();
      
      glBegin(GL_LINES);
      glColor3f(0.0f, 0.0f, 1.0f);
      glVertex3d(p1[0], p1[1], p1[2]);
      glVertex3d(p2[0], p2[1], p2[2]);
      glEnd();
    }

    pangolin::FinishFrame();
    usleep(5000);
  }

}

double GetLoss(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses_gt, vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses_esti) {

  double loss = 0.0;
  
  int t_size = poses_gt.size()<poses_esti.size()?poses_gt.size():poses_esti.size();
  for (size_t i=0;i<t_size;i++) {
    Sophus::SE3d SE3_gt(poses_gt[i].matrix());
    Sophus::SE3d SE3_esti(poses_esti[i].matrix());
    
    // using SE3
    // double error = (SE3_gt.inverse() * SE3_esti).log().norm();
    // cout << error << " ";
    // error = ((SE3_gt.inverse() * SE3_esti).log().transpose() * (SE3_gt.inverse() * SE3_esti).log()).sum();
    // cout << sqrt(error) << endl;
    // loss += error * error;
   
    // using se3
    Vector6d se3_gt = SE3_gt.log();
    Vector6d se3_esti = SE3_esti.log();
    
    Vector6d diff = se3_gt - se3_esti;
    loss += (diff.transpose() * diff).sum();
  }

  loss /= t_size;

  return sqrt(loss);

}

int main() {

  // can make a typedef
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses_gt;
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses_esti;

  if (!ReadTrajectory(groundtruth, poses_gt)) {
    cout << "Could not open GROUND TRUTH file: " << groundtruth << endl;
    return 1;
  }
  if (!ReadTrajectory(estimated, poses_esti)) {
    cout << "Could not open ESTIMATED file: " << estimated << endl;
    return 1;
  }

  cout << "Read File Finished!" << endl << endl;

  // int gt_size = poses_gt.size();
  // cout << gt_size << endl;

  cout << "Computing ATE Loss: " << GetLoss(poses_gt, poses_esti) << endl;

  // pangolin::OpenGlRenderState s_cam;
  // pangolin::View d_cam;

  // InitPangolin(s_cam, d_cam);
  // DrawTrajectory(poses_gt, poses_esti, s_cam, d_cam);
  
  return 0;
}

