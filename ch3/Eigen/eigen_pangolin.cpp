#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <string.h>
using namespace Eigen;

string trajectory_file = "/home/johnson/SLAM/ch3/trajectory.txt";

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main() {

  // vector to save trajectory
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> v;
  
  // open trajectory file
  ifstream fin(trajectory_file);
  if (!fin) {
    cout << "Can not open file: " << trajectory_file << endl;
    return 1;
  }

  // read file
  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Isometry3d tmp(Quaterniond(qw, qx, qy, qz));
    tmp.pretranslate(Vector3d(tx, ty, tz));
    v.push_back(tmp);
  }

  // Draw Trajectory
  DrawTrajectory(v);

  return 0;
}

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
  // Create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
  pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
  pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
  			.SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
  			.SetHandler(new pangolin::Handler3D(s_cam));

  while (!pangolin::ShouldQuit()) {
    // clear buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);

    for (size_t i=0;i<poses.size();i++) {
      // Draw axis x, y, z
      Vector3d Ow = poses[i].translation();
      Vector3d xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
      Vector3d yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
      Vector3d zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
      
      glBegin(GL_LINES);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(xw[0], xw[1], xw[2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(yw[0], yw[1], yw[2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3d(Ow[0], Ow[1], Ow[2]);
      glVertex3d(zw[0], zw[1], zw[2]);
      glEnd();
    }

    // 
    int p_size = poses.size();
    for (size_t i=0;i<p_size;i++) {
      Vector3d p1 = poses[i].translation();
      Vector3d p2 = poses[(i+p_size+1)%p_size].translation();

      glBegin(GL_LINES);
      glVertex3d(p1[0], p1[1], p1[2]);
      glVertex3d(p2[0], p2[1], p2[2]);
      glEnd();
    }

    pangolin::FinishFrame();
    usleep(5000);
  }

}
