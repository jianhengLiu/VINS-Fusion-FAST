/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "keyframe.h"
#include "parameters.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "utility/tic_toc.h"
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <thread>
#include <vector>
#include <visualization_msgs/Marker.h>
#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::CompressedImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;

int R_BUFFER_LENGTH;
int T_BUFFER_LENGTH;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_odometry_rect;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

ros::Publisher pub_point_cloud, pub_margin_cloud;

void new_sequence() {
  printf("new sequence\n");
  sequence++;
  printf("sequence cnt %d \n", sequence);
  if (sequence > 5) {
    ROS_WARN("only support 5 sequences since it's boring to copy code for more "
             "sequences.");
    ROS_BREAK();
  }
  posegraph.posegraph_visualization->reset();
  posegraph.publish();
  m_buf.lock();
  while (!image_buf.empty())
    image_buf.pop();
  while (!point_buf.empty())
    point_buf.pop();
  while (!pose_buf.empty())
    pose_buf.pop();
  while (!odometry_buf.empty())
    odometry_buf.pop();
  m_buf.unlock();
}

void image_callback(const sensor_msgs::CompressedImageConstPtr &image_msg) {
  // ROS_INFO("image_callback!");
  m_buf.lock();
  image_buf.push(image_msg);
  m_buf.unlock();
  // printf(" image time %f \n", image_msg->header.stamp.toSec());

  // detect unstable camera stream
  if (last_image_time == -1)
    last_image_time = image_msg->header.stamp.toSec();
  else if (image_msg->header.stamp.toSec() - last_image_time > 1.0 ||
           image_msg->header.stamp.toSec() < last_image_time) {
    ROS_WARN("image discontinue! detect a new sequence!");
    new_sequence();
  }
  last_image_time = image_msg->header.stamp.toSec();

  // cout<<"image callback, image buff size"<<image_buf.size()<<endl;

  while (image_buf.size() > 1000)
    image_buf.pop();
}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg) {
  // ROS_INFO("point_callback!");
  m_buf.lock();
  point_buf.push(point_msg);
  m_buf.unlock();
  /*
  for (unsigned int i = 0; i < point_msg->points.size(); i++)
  {
      printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i ,
  point_msg->points[i].x, point_msg->points[i].y, point_msg->points[i].z,
                                                   point_msg->channels[i].values[0],
                                                   point_msg->channels[i].values[1]);
  }
  */
  // for visualization
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header = point_msg->header;
  for (unsigned int i = 0; i < point_msg->points.size(); i++) {
    cv::Point3f p_3d;
    p_3d.x = point_msg->points[i].x;
    p_3d.y = point_msg->points[i].y;
    p_3d.z = point_msg->points[i].z;
    Eigen::Vector3d tmp =
        posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) +
        posegraph.t_drift;
    geometry_msgs::Point32 p;
    p.x = tmp(0);
    p.y = tmp(1);
    p.z = tmp(2);
    point_cloud.points.push_back(p);
  }
  pub_point_cloud.publish(point_cloud);
}

// only for visualization
void margin_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg) {
  sensor_msgs::PointCloud point_cloud;
  point_cloud.header = point_msg->header;
  for (unsigned int i = 0; i < point_msg->points.size(); i++) {
    cv::Point3f p_3d;
    p_3d.x = point_msg->points[i].x;
    p_3d.y = point_msg->points[i].y;
    p_3d.z = point_msg->points[i].z;
    Eigen::Vector3d tmp =
        posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) +
        posegraph.t_drift;
    geometry_msgs::Point32 p;
    p.x = tmp(0);
    p.y = tmp(1);
    p.z = tmp(2);
    point_cloud.points.push_back(p);
  }
  pub_margin_cloud.publish(point_cloud);
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
  // ROS_INFO("pose_callback!");
  m_buf.lock();
  pose_buf.push(pose_msg);
  m_buf.unlock();
  /*
  printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n",
  pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                                                     pose_msg->pose.pose.position.z,
                                                     pose_msg->pose.pose.orientation.w,
                                                     pose_msg->pose.pose.orientation.x,
                                                     pose_msg->pose.pose.orientation.y,
                                                     pose_msg->pose.pose.orientation.z);
  */
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
  // ROS_INFO("vio_callback!");
  Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                 pose_msg->pose.pose.position.z);
  Quaterniond vio_q;
  vio_q.w() = pose_msg->pose.pose.orientation.w;
  vio_q.x() = pose_msg->pose.pose.orientation.x;
  vio_q.y() = pose_msg->pose.pose.orientation.y;
  vio_q.z() = pose_msg->pose.pose.orientation.z;

  vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
  vio_q = posegraph.w_r_vio * vio_q;

  vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
  vio_q = posegraph.r_drift * vio_q;

  nav_msgs::Odometry odometry;
  odometry.header = pose_msg->header;
  odometry.header.frame_id = "world";
  odometry.pose.pose.position.x = vio_t.x();
  odometry.pose.pose.position.y = vio_t.y();
  odometry.pose.pose.position.z = vio_t.z();
  odometry.pose.pose.orientation.x = vio_q.x();
  odometry.pose.pose.orientation.y = vio_q.y();
  odometry.pose.pose.orientation.z = vio_q.z();
  odometry.pose.pose.orientation.w = vio_q.w();
  pub_odometry_rect.publish(odometry);

  Vector3d vio_t_cam;
  Quaterniond vio_q_cam;
  vio_t_cam = vio_t + vio_q * tic;
  vio_q_cam = vio_q * qic;

  cameraposevisual.reset();
  cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
  odometry.header.frame_id = "map"; // zxzx
  cameraposevisual.publish_by(pub_camera_pose_visual, odometry.header);
  odometry.header.frame_id = "world";
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
  m_process.lock();
  tic = Vector3d(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y,
                 pose_msg->pose.pose.position.z);
  qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                    pose_msg->pose.pose.orientation.x,
                    pose_msg->pose.pose.orientation.y,
                    pose_msg->pose.pose.orientation.z)
            .toRotationMatrix();
  m_process.unlock();
}

void process() {
  int cnt = 0;

  // cout<<"image buff size"<<image_buf.size()<<endl;
  while (true) {
    sensor_msgs::CompressedImageConstPtr image_msg = NULL;
    sensor_msgs::PointCloudConstPtr point_msg = NULL;
    nav_msgs::Odometry::ConstPtr pose_msg = NULL;

    // find out the messages with same time stamp
    m_buf.lock();
    if (!image_buf.empty() && !point_buf.empty() && !pose_buf.empty()) {
      if (image_buf.front()->header.stamp.toSec() >
          pose_buf.front()->header.stamp.toSec()) {
        pose_buf.pop();
        printf("throw pose at beginning\n");
      } else if (image_buf.front()->header.stamp.toSec() >
                 point_buf.front()->header.stamp.toSec()) {
        point_buf.pop();
        printf("throw point at beginning\n");
      } else if (image_buf.back()->header.stamp.toSec() >=
                     pose_buf.front()->header.stamp.toSec() &&
                 point_buf.back()->header.stamp.toSec() >=
                     pose_buf.front()->header.stamp.toSec()) {
        pose_msg = pose_buf.front();
        pose_buf.pop();
        while (!pose_buf.empty())
          pose_buf.pop();
        while (image_buf.front()->header.stamp.toSec() <
               pose_msg->header.stamp.toSec()) {
          // sensor_msgs::ImageConstPtr ptr_img = image_buf.front();
          image_buf.pop();
          // delete ptr_img;
        }

        image_msg = image_buf.front();
        image_buf.pop();

        while (point_buf.front()->header.stamp.toSec() <
               pose_msg->header.stamp.toSec())
          point_buf.pop();
        point_msg = point_buf.front();
        point_buf.pop();
      }
    }
    m_buf.unlock();

    if (pose_msg != NULL) {
      // printf(" pose time %f \n", pose_msg->header.stamp.toSec());
      // printf(" point time %f \n", point_msg->header.stamp.toSec());
      // printf(" image time %f \n", image_msg->header.stamp.toSec());
      //  skip fisrt few
      if (skip_first_cnt < SKIP_FIRST_CNT) {
        skip_first_cnt++;
        continue;
      }

      if (skip_cnt < SKIP_CNT) {
        skip_cnt++;
        continue;
      } else {
        skip_cnt = 0;
      }

      cv_bridge::CvImageConstPtr ptr;
      ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

      cv::Mat image = ptr->image;
      // build keyframe
      Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                            pose_msg->pose.pose.position.y,
                            pose_msg->pose.pose.position.z);
      Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                               pose_msg->pose.pose.orientation.x,
                               pose_msg->pose.pose.orientation.y,
                               pose_msg->pose.pose.orientation.z)
                       .toRotationMatrix();
      if ((T - last_t).norm() > SKIP_DIS) {
        vector<cv::Point3f> point_3d;
        vector<cv::Point2f> point_2d_uv;
        vector<cv::Point2f> point_2d_normal;
        vector<double> point_id;

        for (unsigned int i = 0; i < point_msg->points.size(); i++) {
          cv::Point3f p_3d;
          p_3d.x = point_msg->points[i].x;
          p_3d.y = point_msg->points[i].y;
          p_3d.z = point_msg->points[i].z;
          point_3d.push_back(p_3d);

          cv::Point2f p_2d_uv, p_2d_normal;
          double p_id;
          p_2d_normal.x = point_msg->channels[i].values[0];
          p_2d_normal.y = point_msg->channels[i].values[1];
          p_2d_uv.x = point_msg->channels[i].values[2];
          p_2d_uv.y = point_msg->channels[i].values[3];
          p_id = point_msg->channels[i].values[4];
          point_2d_normal.push_back(p_2d_normal);
          point_2d_uv.push_back(p_2d_uv);
          point_id.push_back(p_id);

          // printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
        }

        KeyFrame *keyframe = new KeyFrame(pose_msg->header.stamp, frame_index,
                                          T, R, image, point_3d, point_2d_uv,
                                          point_2d_normal, point_id, sequence);
        m_process.lock();
        start_flag = 1;
        posegraph.addKeyFrame(keyframe, 1);
        m_process.unlock();
        frame_index++;
        last_t = T;
      }
    }

    cnt = (cnt + 1) % 20;

    if (cnt == 0) {

      Eigen::Matrix3d w_R_vio_pub;
      posegraph.w_Q_vio_queue.push_back(posegraph.pg_yaw_vio);
      posegraph.w_t_vio_queue.push_back(posegraph.pg_t_vio);
      while (posegraph.w_Q_vio_queue.size() > posegraph.R_buffer_length)
        posegraph.w_Q_vio_queue.pop_front();
      while (posegraph.w_t_vio_queue.size() > posegraph.T_buffer_length)
        posegraph.w_t_vio_queue.pop_front();

      int size_q = posegraph.w_Q_vio_queue.size();
      int size_tt = posegraph.w_t_vio_queue.size();

      double yaw_temp = 0.0;

      for (auto it = posegraph.w_Q_vio_queue.begin();
           it != posegraph.w_Q_vio_queue.end(); it++) {
        yaw_temp = yaw_temp + (*it) / size_q;
      }

      Eigen::Vector3d pg_t_vio_pub = Eigen::Vector3d::Zero();

      for (auto it = posegraph.w_t_vio_queue.begin();
           it != posegraph.w_t_vio_queue.end(); it++) {
        pg_t_vio_pub = pg_t_vio_pub + (*it) / size_tt;
      }

      w_R_vio_pub << cos(yaw_temp), -sin(yaw_temp), 0, sin(yaw_temp),
          cos(yaw_temp), 0, 0, 0, 1;
      Eigen::Quaterniond pg_q_vio_pub(w_R_vio_pub);

      posegraph.pg_T_vio.position.x = pg_t_vio_pub.x();
      posegraph.pg_T_vio.position.y = pg_t_vio_pub.y();
      posegraph.pg_T_vio.position.z = pg_t_vio_pub.z();
      posegraph.pg_T_vio.orientation.w = pg_q_vio_pub.w();
      posegraph.pg_T_vio.orientation.x = pg_q_vio_pub.x();
      posegraph.pg_T_vio.orientation.y = pg_q_vio_pub.y();
      posegraph.pg_T_vio.orientation.z = pg_q_vio_pub.z();

      posegraph.pub_pg_T_vio.publish(posegraph.pg_T_vio);
    }
    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

void command() {
  while (1) {
    char c = getchar();
    if (c == 's') {
      m_process.lock();
      posegraph.savePoseGraph();
      m_process.unlock();
      printf("save pose graph finish\nyou can set 'load_previous_pose_graph' "
             "to 1 in the config file to reuse it next time\n");
      printf("program shutting down...\n");
      ros::shutdown();
    }
    if (c == 'n')
      new_sequence();

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "loop_fusion");
  ros::NodeHandle n("~");
  posegraph.registerPub(n);

  VISUALIZATION_SHIFT_X = 0;
  VISUALIZATION_SHIFT_Y = 0;
  SKIP_CNT = 0;
  SKIP_DIS = 0;

  if (argc != 2) {
    printf("please intput: rosrun loop_fusion loop_fusion_node [config file] \n"
           "for example: rosrun loop_fusion loop_fusion_node "
           "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/"
           "euroc_stereo_imu_config.yaml \n");
    return 0;
  }

  string config_file = argv[1];
  printf("config_file: %s\n", argv[1]);

  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  cameraposevisual.setScale(0.1);
  cameraposevisual.setLineWidth(0.01);

  std::string IMAGE_TOPIC;
  int LOAD_PREVIOUS_POSE_GRAPH;

  ROW = fsSettings["image_height"];
  COL = fsSettings["image_width"];
  std::string pkg_path = ros::package::getPath("loop_fusion");
  string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
  cout << "vocabulary_file" << vocabulary_file << endl;
  posegraph.loadVocabulary(vocabulary_file);

  BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
  cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

  int pn = config_file.find_last_of('/');
  std::string configPath = config_file.substr(0, pn);
  std::string cam0Calib;
  fsSettings["cam0_calib"] >> cam0Calib;
  std::string cam0Path = configPath + "/" + cam0Calib;
  printf("cam calib path: %s\n", cam0Path.c_str());
  m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(
      cam0Path.c_str());

  fsSettings["image0_topic"] >> IMAGE_TOPIC;
  fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
  fsSettings["output_path"] >> VINS_RESULT_PATH;
  fsSettings["save_image"] >> DEBUG_IMAGE;

  fsSettings["pg_R_vio_buffer_length"] >> R_BUFFER_LENGTH;
  fsSettings["pg_t_vio_buffer_length"] >> T_BUFFER_LENGTH;
  posegraph.R_buffer_length = R_BUFFER_LENGTH;
  posegraph.T_buffer_length = T_BUFFER_LENGTH;

  LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
  VINS_RESULT_PATH = VINS_RESULT_PATH + "/vio_loop.csv";
  std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
  fout.close();

  int USE_IMU = fsSettings["imu"];
  posegraph.setIMUFlag(USE_IMU);
  fsSettings.release();

  if (LOAD_PREVIOUS_POSE_GRAPH) {
    printf("load pose graph\n");
    m_process.lock();
    posegraph.loadPoseGraph();
    m_process.unlock();
    printf("load pose graph finish\n");
    load_flag = 1;
  } else {
    printf("no previous pose graph\n");
    load_flag = 1;
  }

  ros::Subscriber sub_vio =
      n.subscribe("/vins_estimator/odometry", 100, vio_callback);
  ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 100, image_callback);
  ros::Subscriber sub_pose =
      n.subscribe("/vins_estimator/keyframe_pose", 100, pose_callback);
  ros::Subscriber sub_extrinsic =
      n.subscribe("/vins_estimator/extrinsic", 100, extrinsic_callback);
  ros::Subscriber sub_point =
      n.subscribe("/vins_estimator/keyframe_point", 100, point_callback);
  ros::Subscriber sub_margin_point =
      n.subscribe("/vins_estimator/margin_cloud", 100, margin_point_callback);

  pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
  pub_camera_pose_visual =
      n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
  pub_point_cloud =
      n.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 1000);
  pub_margin_cloud =
      n.advertise<sensor_msgs::PointCloud>("margin_cloud_loop_rect", 1000);
  pub_odometry_rect = n.advertise<nav_msgs::Odometry>("odometry_rect", 1000);

  std::thread measurement_process;
  std::thread keyboard_command_process;

  measurement_process = std::thread(process);
  keyboard_command_process = std::thread(command);

  ros::spin();

  return 0;
}
