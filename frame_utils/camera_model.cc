/*
 * Added by Vincent
 * Tus Sept 8 15:23:09 CST 2020
 */

#include "camera_model.h"
#include "input_source/local_depth_reader.h"
#include "input_source/local_type.h"
#include <input_source/local_config.h>

#include <iostream>

using namespace input_source;

namespace frame_utils {

CameraModel::CameraModel(const input_source::LocalConfig &config) {
  const auto W = LocalDepthReader::DataType::width;
  const auto H = LocalDepthReader::DataType::height;

  this->inv_in_para = config.getInvCameraInPara();  // 3 x 3 相机内参数矩阵的逆
  this->ex_para = config.getCameraExPara();         // 4 x 4 相机外参数矩阵
  std::cout << "inv_in_para\n" << this->inv_in_para << std::endl;
  std::cout << "ex_para\n" << this->ex_para << std::endl;

  this->image_to_camera_map.resize(4, W * H);
  for (int c = 0; c < this->image_to_camera_map.cols(); ++c) {
    this->image_to_camera_map.col(c) << c % W, floor(c / W), 1, 1;
  }
  // 把 3 x 3 的相机内参数矩阵的逆扩展成 4 x 4 的矩阵，用于齐次坐标的计算。
  Eigen::Matrix4f inv_in_para_ext;
  inv_in_para_ext << this->inv_in_para, Eigen::Vector3f::Zero(),
                  Eigen::Matrix<float, 1, 3, Eigen::RowMajor>::Zero(), 1;
  std::cout << "inv_in_para_ext\n" << inv_in_para_ext << std::endl;

  // 计算图像的像素坐标转换到相机坐标系下的齐次坐标，深度 z = 1
  this->image_to_camera_map = inv_in_para_ext * this->image_to_camera_map;
}

auto CameraModel::frameImageToCamera(const input_source::LocalDepthReader::DataType &image) const -> Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> {
  const auto W = LocalDepthReader::DataType::width;
  const auto H = LocalDepthReader::DataType::height;

  auto camera_point_cloud = this->image_to_camera_map;
  for (auto c = 0U; c < W * H; ++c)
    camera_point_cloud.col(c) *= (image.data[c] * 1e-3);

  return camera_point_cloud;
}

}  // namespace frame_utils
