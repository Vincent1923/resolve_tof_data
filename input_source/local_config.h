/*
 * Added by Vincent
 * Thu Sept 3 22:00:10 CST 2020
 */

// #pragma once是一个比较常用的C/C++预处理指令，只要在头文件的最开始加入这条预处理指令，就能够保证头文件只被编译一次。
#pragma once
#include "local_type.h"
#include <Eigen/Core>

namespace input_source {

class LocalConfig {
 public:
  static LocalConfig &instance();

  bool loadCalibFile(const char *file_calib);

  auto getImageSize() const -> const Vec2u&;

  auto getCameraInPara() const -> const Eigen::Matrix3f&;

  auto getInvCameraInPara() const -> const Eigen::Matrix3f&;

  auto getCameraExPara() const -> const Eigen::Matrix4f&;

  auto getInvCameraExPara() const -> const Eigen::Matrix4f&;

 private:
  Vec2u image_size;

  Eigen::Matrix3f depth_in_para;      // ToF 相机的内参数矩阵，3 x 3
  Eigen::Matrix3f inv_depth_in_para;  // ToF 相机的内参数矩阵的逆，3 x 3

  Eigen::Matrix4f depth_ex_para;      // ToF 相机的外参数矩阵，即相机坐标系相对于 odom 的变换矩阵，4 x 4
  Eigen::Matrix4f inv_depth_ex_para;  // ToF 相机的外参数矩阵的逆，4 x 4
};

}  // namespace input_source
