/*
 * Added by Vincent
 * Tus Sept 8 15:19:06 CST 2020
 */

#pragma once
#include "input_source/local_config.h"
#include <input_source/local_depth_reader.h>
#include <input_source/local_type.h>

#include <Eigen/Core>

namespace frame_utils {

class CameraModel {
  Eigen::Matrix3f inv_in_para;
  Eigen::Matrix4f ex_para;

  Eigen::Matrix<float, 4, Eigen::Dynamic> image_to_camera_map;  // 图像的像素坐标转换到相机坐标系下的齐次坐标，深度 z = 1
  Eigen::Matrix<float, 4, Eigen::Dynamic> image_to_odo_map;

public:
    CameraModel(const input_source::LocalConfig &config);

    // auto pointImageToCamera(const input_source::Vec2u &image_pos, input_source::ushort depth) const -> input_source::Vec3f;

    // auto pointImageToOdo(const input_source::Vec2u &image_pos, input_source::ushort depth) const -> input_source::Vec3f;

    // 计算深度图上的每一个像素点在相机坐标系下的3D坐标点云，根据输入的一帧深度图和图像的像素坐标来计算。
    auto frameImageToCamera(const input_source::LocalDepthReader::DataType &image) const -> Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

    // auto frameImageToOdo(const input_source::LocalDepthReader::DataType &image) const -> Eigen::Matrix<input_source::Vec3f, Eigen::Dynamic, Eigen::Dynamic>;
};

}  // namespace frame_utils
