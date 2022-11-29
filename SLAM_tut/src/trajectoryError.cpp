//
// Created by Lenovo on 2022/10/26.
//

#include <pangolin/pangolin.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/se3.hpp>

using namespace Sophus;
using namespace std;

const string groundtruth_file = "SLAM_tut/utility/ch4_groundtruth.txt";
const string estimated_file = "SLAM_tut/utility/ch4_estimated.txt";

using TrajectoryType = vector<SE3d>;

void DrawTrajectoryType(const TrajectoryType &gt, const TrajectoryType &esti);

TrajectoryType ReadTrajectory(const string &path);

int main(int argc, char **argv)
{
  TrajectoryType groundtruth = ReadTrajectory(groundtruth_file);
  TrajectoryType estimated = ReadTrajectory(estimated_file);
  assert(!groundtruth.empty() && !estimated.empty());
  assert(groundtruth.size() == estimated.size());

  double rmse = 0;
  for (size_t i = 0; i < estimated.size(); i++) {
    auto p1 = estimated[i];
    auto p2 = groundtruth[i];
    double error =(p2.inverse() * p1).log().norm();
    rmse += error * error;
  }
  rmse = rmse / estimated.size();
  rmse = sqrt(rmse);
  cout << "RMSE = " << rmse << endl;
  DrawTrajectoryType(groundtruth, estimated);
  return 0;
}

void DrawTrajectoryType(const TrajectoryType &gt, const TrajectoryType &esti) {
  // create pangolin window and plot the trajectory
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

  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
//    for (size_t i = 0; i < gt.size(); i++) {
//      // 画每个位姿的三个坐标轴
//      Vector3d Ow = gt[i].translation();
//      Vector3d Xw = gt[i] * (0.1 * Vector3d(1, 0, 0));
//      Vector3d Yw = gt[i] * (0.1 * Vector3d(0, 1, 0));
//      Vector3d Zw = gt[i] * (0.1 * Vector3d(0, 0, 1));
//      glBegin(GL_LINES);
//      glColor3f(1.0, 0.0, 0.0);
//      glVertex3d(Ow[0], Ow[1], Ow[2]);
//      glVertex3d(Xw[0], Xw[1], Xw[2]);
//      glColor3f(0.0, 1.0, 0.0);
//      glVertex3d(Ow[0], Ow[1], Ow[2]);
//      glVertex3d(Yw[0], Yw[1], Yw[2]);
//      glColor3f(0.0, 0.0, 1.0);
//      glVertex3d(Ow[0], Ow[1], Ow[2]);
//      glVertex3d(Zw[0], Zw[1], Zw[2]);
//      glEnd();
//    }
    // 画出连线
    for (size_t i = 0; i < gt.size() - 1; i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = gt[i], p2 = gt[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    glLineWidth(4);
//    for (size_t i = 0; i < esti.size(); i++) {
//      // 画每个位姿的三个坐标轴
//      Vector3d Ow = esti[i].translation();
//      Vector3d Xw = esti[i] * (0.1 * Vector3d(1, 0, 0));
//      Vector3d Yw = esti[i] * (0.1 * Vector3d(0, 1, 0));
//      Vector3d Zw = esti[i] * (0.1 * Vector3d(0, 0, 1));
//      glBegin(GL_LINES);
//      glColor3f(1.0, 0.0, 0.0);
//      glVertex3d(Ow[0], Ow[1], Ow[2]);
//      glVertex3d(Xw[0], Xw[1], Xw[2]);
//      glColor3f(0.0, 1.0, 0.0);
//      glVertex3d(Ow[0], Ow[1], Ow[2]);
//      glVertex3d(Yw[0], Yw[1], Yw[2]);
//      glColor3f(0.0, 0.0, 1.0);
//      glVertex3d(Ow[0], Ow[1], Ow[2]);
//      glVertex3d(Zw[0], Zw[1], Zw[2]);
//      glEnd();
//    }
    // 画出连线
    for (size_t i = 0; i < esti.size() - 1; i++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = esti[i], p2 = esti[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::microseconds(5000));
  }
}

TrajectoryType ReadTrajectory(const string &path)
{
  ifstream fin(path);
  TrajectoryType trajectory;
  if (!fin) {
    cerr << "trajectory " << path << " not found" << endl;
    return trajectory;
  }
  while (!fin.eof()) {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    SE3d p1(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    trajectory.push_back(p1);
  }
  return trajectory;
}