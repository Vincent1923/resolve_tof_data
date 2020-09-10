/*
 * Added by Vincent
 * Thu Sept 3 19:59:50 CST 2020
 */

#include "input_source/local_config.h"
#include "input_source/local_depth_reader.h"
#include "input_source/local_pose_reader.h"
#include "input_source/local_type.h"

#include "frame_utils/camera_model.h"
#include "frame_utils/data_align.h"

#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <iostream>
#include <string>
#include <unistd.h>

// 点云显示
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace input_source;
using namespace frame_utils;

boost::mutex updateModelMutex;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  // Open 3D viewer and add point cloud
  // 创建视窗对象并给标题栏设置一个名称“3D Viewer”并将它设置为boost::shared_ptr智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);

  // 设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为黑色
  viewer->setBackgroundColor(0, 0, 0);

  /*将点云添加到视窗对象中，并定一个唯一的字符串作为ID 号，利用此字符串保证在其他成员中也能标志引用该点云，
    多次调用 addPointCloud() 可以实现多个点云的添加，每调用一次就会创建一个新的ID号，如果想更新一个已经显示的点云，
    先调用removePointCloud()，并提供需要更新的点云 ID 号，也可使用 updatePointCloud() */
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

  // 用于改变显示点云的尺寸，可以利用该方法控制点云在视窗中的显示方法，1设置显示点云大小
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE , 1, "sample cloud");

  // 通过设置照相机参数使得从默认的角度和方向观察点云
  viewer->initCameraParameters();
  viewer->addCoordinateSystem();

  return viewer;
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
  // 下面几行代码在执行一个while循环，每次调用spinOnce都给视窗处理事件的时间，这样允许鼠标键盘等交互操作，此外还有一种spin的重载方法，它只需调用一次。
  std::cout << "viewerRunner: " << std::endl;
  while (!viewer->wasStopped()) {
    std::cout << "viewer->spinOnce(100);: " << std::endl;
    viewer->spinOnce(1000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
  }
}

// auto create_point_cloud(const Eigen::MatrixXf &point_cloud) -> pcl::PointCloud<pcl::PointXYZRGBA>::Ptr {
auto create_point_cloud(const Eigen::MatrixXf &point_cloud, pcl::PointCloud<pcl::PointXYZ> &pcloud) -> pcl::PointCloud<pcl::PointXYZ>::Ptr {
  const auto W = input_source::LocalDepthReader::DataType::width;
  const auto H = input_source::LocalDepthReader::DataType::height;

  // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr map_(new pcl::PointCloud<pcl::PointXYZRGBA>(W, H));
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_(new pcl::PointCloud<pcl::PointXYZ>(W, H));

  for (auto r = 0; r < H; ++r) {
    for (auto c = 0; c < W; ++c) {
      auto &ori_data = point_cloud.col(r * W + c);
      auto &dst_data = map_->at(c, r);

      dst_data.x = ori_data.x();
      dst_data.y = ori_data.y();
      dst_data.z = ori_data.z();

      pcloud.push_back(dst_data);
    }
  }

  return map_;
}

auto create_pose(const input_source::LocalPoseReader::DataType &pose) -> Eigen::Affine3d {
  // 4*4 齐次矩阵变换
  Eigen::Affine3d result;

  auto x = pose.pose.t3_x;
  auto y = pose.pose.t3_y;
  auto yaw = pose.pose.t3_yaw;
  auto sin_yaw = sin(yaw);
  auto cos_yaw = cos(yaw);

  // 由机器人全局位姿创建的变换矩阵
  Eigen::Matrix4d trans;
  trans << cos_yaw,   -sin_yaw,   0,  x,
           sin_yaw,    cos_yaw,   0,  y,
           0, 0,  1,  0,
           0, 0,  0,  1;

  // 相机外参，相机坐标系到机器人坐标系的变换矩阵
  auto ex_para = input_source::LocalConfig::instance().getCameraExPara();

  // 相机坐标系到世界坐标系的变换矩阵
  result = trans * ex_para.cast<double>();

  return result;
}

auto pointCloudToOdom(const Eigen::Matrix<float, 4, Eigen::Dynamic> &point_cloud, const Eigen::Affine3d &pose) -> Eigen::Matrix<float, 4, Eigen::Dynamic> {
  const auto W = input_source::LocalDepthReader::DataType::width;
  const auto H = input_source::LocalDepthReader::DataType::height;

  Eigen::Matrix<float, 4, Eigen::Dynamic> result;
  result.resize(4, W * H);

  Eigen::Matrix4d trans = pose.matrix();

  result = trans.cast<float>() * point_cloud;

  return result;
}

int main_process(const boost::program_options::variables_map &vm) {

  auto calib_file = vm["calib"].as<std::string>();
  auto camera_file = vm["depth"].as<std::string>();
  auto pose_file = vm["pose"].as<std::string>();

  // 读取相机校准文件
  LocalConfig::instance().loadCalibFile(calib_file.data());

  auto camera_reader = LocalDepthReader(camera_file.data());
  auto pose_reader = LocalPoseReader(pose_file.data());

  auto data_align = DataAlign<LocalDepthReader, LocalPoseReader>(camera_reader, pose_reader, true);

  LocalDepthReader::DataType depth_frame;  // 一帧深度图
  LocalPoseReader::DataType pose_frame;    // 一帧机器人位姿
  CameraModel camera_model(LocalConfig::instance());

  // PCL 点云显示
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  viewer = simpleVis(cloud_ptr);

  boost::thread vthread(&viewerRunner, viewer);

  while (!data_align.reachEnd()) {
    std::cout << "while" << std::endl;
    // 线性插值，计算一帧深度图 depth_frame 对应的机器人在世界坐标系下的位姿 pose_frame
    if (!data_align(depth_frame, pose_frame))
      continue;

    // 计算一帧深度图 depth_frame 的点云在相机坐标系下的坐标
    Eigen::Matrix<float, 4, Eigen::Dynamic> point_cloud_mat = camera_model.frameImageToCamera(depth_frame);
    // std::cout << "point_cloud_mat col: " << point_cloud_mat.cols() << std::endl;

    // 计算相机在世界坐标系下的位姿 pose
    Eigen::Affine3d pose = create_pose(pose_frame);

    // 计算一帧点云在世界坐标系下的坐标
    Eigen::Matrix<float, 4, Eigen::Dynamic> point_cloud_world_mat = pointCloudToOdom(point_cloud_mat, pose);
    // std::cout << "point_cloud_world_mat col: " << point_cloud_world_mat.cols() << std::endl;

    // 创建世界坐标系下的 pcl 点云
    pcl::PointCloud<pcl::PointXYZ> pcloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_world = create_point_cloud(point_cloud_world_mat, pcloud1);
    // std::cout << "pcloud1 size(): " << pcloud1.size() << std::endl;

    *cloud_ptr += pcloud1;
    std::cout << "cloud_ptr size(): " << cloud_ptr->size() << std::endl;
    boost::mutex::scoped_lock updateLock(updateModelMutex);
    std::cout << "updateLock: " << std::endl;
    viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr, "sample cloud");
    updateLock.unlock();
    std::cout << "updateLock.unlock(): " << std::endl;
    // boost::this_thread::sleep(boost::posix_time::microseconds (100));

    // break;
    sleep(1);
    std::cout << "sleep(1);" << std::endl;
  }
  vthread.join();

  return 0;
}

int main(int argc, char *argv[]) {

  namespace bpo = boost::program_options;
  namespace bfs = boost::filesystem;

  // 构造选项描述器
  // 选项描述器，其参数为该选项描述器的名字
  bpo::options_description opts_desc("Allowed options");

  // 为选项描述器增加选项
  // 其参数依次为：key，value的类型，该选项的描述
  opts_desc.add_options()
      ("help,h", "produce help message")
      ("calib", bpo::value<std::string>()->default_value("./ToF_dataset/calib_depth_tof.8Z2127.txt"), "calib file")
      ("depth", bpo::value<std::string>()->default_value("./ToF_dataset/camera.116444740651014013.bin"), "depth camera file")
      ("pose", bpo::value<std::string>()->default_value("./ToF_dataset/pose.116444740651014013.bin"), "pose file");

  bpo::positional_options_description p;
  p.add("calib", 1);
  p.add("depth", 1);
  p.add("pose", 1);

  bpo::variables_map vm;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(p).run(), vm);

  bool bad_args = false;
  try {
    bpo::notify(vm);
  } catch (...) {
    bad_args = true;
    // std::cout << "bad args" << std::endl;
  }

  if (vm.count("help") || !vm.count("calib") || !vm.count("depth") || !vm.count("pose") || bad_args) {

    // std::cout << "Usage: " << argv[0] << " [OPTS]" << std::endl;
    // std::cout << opts_desc << std::endl;
    return -1;
  }

  return main_process(vm);
}
