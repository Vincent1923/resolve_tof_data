/*
 * Added by Vincent
 * Fri Sept 4 10:06:30 CST 2020
 */

#include "local_config.h"
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>

namespace input_source {

LocalConfig& LocalConfig::instance() {
    static LocalConfig instance;
    return instance;
}

bool LocalConfig::loadCalibFile(const char *file_calib) {
  if (!file_calib) {
    std::cout << "calib file is null!" << std::endl;
    return false;
  }

  auto in_file = std::ifstream(file_calib);
  if (!in_file.is_open()) {
    std::cout << "can not open calib file!" << std::endl;
    return false;
  }

  // 从 calib file 文件读取深度图的尺寸(width & height)
  in_file >> this->image_size.w >> this->image_size.h;
  if (this->image_size.w != depth_width || this->image_size.h != depth_height) {
    std::cout << "calib file error!" << std::endl;
    return false;
  }

  // 从 calib file 文件读取 ToF 相机的内参数矩阵：
  //   fx 0  cx
  //   0  fy cy
  //   0  0  1
  this->depth_in_para.setIdentity();
  in_file >> this->depth_in_para(0, 0)
          >> this->depth_in_para(1, 1)
          >> this->depth_in_para(0, 2)
          >> this->depth_in_para(1, 2);
  std::cout << "camera param:\n" << this->depth_in_para << std::endl;
  this->inv_depth_in_para = this->depth_in_para.inverse();
  std::cout << "inverse camera param:\n" << this->inv_depth_in_para << std::endl;

  // 从 calib file 文件读取 ToF 相机的外参？
  // tx, ty, tz 表示平移向量。
  // rx, ry, rz 分别表示绕 x, y, z 坐标轴的旋转，即 rool, pitch, yaw。
  float tx, ty, tz, rx, ry, rz;
  in_file >> tx >> ty >> tz
          >> rx >> ry >> rz;

  // 由于相机坐标系和机器人坐标系的不同而产生的旋转变换矩阵 raw_to_norm
  // 相机坐标系：x 向右，y 向下，z 轴指向相机前方。
  // 机器人坐标系：x 指向机器人前方，y 向左，z 轴向上。
  Eigen::Matrix4f raw_to_norm;
  raw_to_norm << 0, 0, 1, 0,
                 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 0, 1;

  Eigen::AngleAxisf r_roll(rx, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf r_pitch(ry, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf r_yaw(rz, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f t(tx, ty, tz);

  // 由外参数的平移向量和旋转矩阵，构造相机到 odom 的变换矩阵 mini_T
  Eigen::Matrix4f mini_T = (t * r_yaw * r_pitch * r_roll).matrix();
  std::cout << "mini_T\n" << mini_T << std::endl;

  // 最后添加由于相机坐标系和机器人坐标系的不同而产生的旋转变换矩阵，
  // 计算最终的相机坐标系相对于 odom 的变换矩阵 depth_ex_para。
  Eigen::Matrix4f odo_to_tof = mini_T * raw_to_norm;
  this->depth_ex_para = odo_to_tof;
  this->inv_depth_ex_para = odo_to_tof.inverse();

  return true;
}

auto LocalConfig::getImageSize() const -> const Vec2u& {
  return this->image_size;
}

auto LocalConfig::getCameraInPara() const -> const Eigen::Matrix3f& {
  return this->depth_in_para;
}

auto LocalConfig::getInvCameraInPara() const -> const Eigen::Matrix3f& {
  return this->inv_depth_in_para;
}

auto LocalConfig::getCameraExPara() const -> const Eigen::Matrix4f& {
  return this->depth_ex_para;
}

auto LocalConfig::getInvCameraExPara() const -> const Eigen::Matrix4f& {
  return this->inv_depth_ex_para;
}

}  // namespace input_source
