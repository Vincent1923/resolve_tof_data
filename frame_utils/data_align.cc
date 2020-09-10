/*
 * Added by Vincent
 * Mon Sept 7 19:56:00 CST 2020
 */

#include "data_align.h"
#include "input_source/local_depth_reader.h"
#include "input_source/local_pose_reader.h"

#include <Eigen/Core>

#include <cstdio>
#include <iomanip>
#include <iostream>
#include <memory>
#include <fstream>

using namespace input_source;
// using namespace frame_utils;

namespace frame_utils {

// image_frame 为一帧包含时间戳的深度图，pose_frame 为一帧包含时间戳的机器人位姿
template <>
auto DataAlign<LocalDepthReader, LocalPoseReader>::operator()(LocalDepthReader::DataType &image_frame, LocalPoseReader::DataType &pose_frame) -> bool {
  // 获取一帧深度图并放在变量 image_frame 中
  if (!this->base_stream.getImage(image_frame)) {
    return false;
  }

  // aim_time 为进行线性插值的时间，即获取的一帧深度图的时间戳
  auto aim_time = image_frame.time_stamp;

  LocalPoseReader::DataType pre_frame;
  LocalPoseReader::DataType frame;

  // 获取一帧位姿并放在变量 frame 中
  if (!this->ref_stream.peep(frame))
    return false;

  // 若位姿的时间戳比深度图的时间戳大，则直接返回 false
  if (frame.time_stamp > aim_time) {
    // std::cout << "frame.time_stamp > aim_time" << std::endl;
    return false;
  }

  // 以下循环不断获取一帧位姿，直到时间戳 pre_frame < aim_time < frame 为止。
  while (frame.time_stamp < aim_time) {
    pre_frame = frame;
    if (!this->ref_stream.getPose(frame))
      return false;
  }

  auto record = [](const LocalPoseReader::DataType &aim_frame, const LocalPoseReader::DataType &ref_frame) {
      // std::cout << "record" << std::endl;
      auto out_file = std::ofstream("delta_out.txt", std::ios::app);
      static unsigned index = 0;

      auto delta_time = aim_frame.time_stamp - ref_frame.time_stamp;
      auto delta_pose = aim_frame.pose - ref_frame.pose;
      out_file << index++ << "\t" << std::setprecision(5) << delta_time << "\t" << delta_pose.t3_x << "\t" << delta_pose.t3_y << " \t" << (delta_pose.t3_yaw / M_PI * 180.f) << std::endl;
  };

  // 检查时间戳是否符合 pre_frame < aim_time < frame
  bool is_pre = (aim_time - pre_frame.time_stamp) < (frame.time_stamp - aim_time);
  // std::cout << "is_pre: " << is_pre << std::endl;

  // 检查前一帧位姿 frame 的时间戳是否等于深度图像的时间戳 aim_time
  // 若是，则直接把前一帧位姿赋值给 pose_frame
  // 若否，则进行线性插值的计算。
  if (frame.time_stamp == aim_time) {
    pose_frame = frame;
    // record(frame, frame);
  } else {
    // 线性插值，计算与一帧深度图的时间戳 aim_time 相同的一帧机器人位姿 pose_frame
    auto aim_frame = pre_frame.interpolation(&frame, aim_time);
    pose_frame.time_stamp = aim_frame->time_stamp;
    pose_frame.pose = std::static_pointer_cast<LocalPoseReader::DataType>(aim_frame)->pose;
  }

  return true;
}

}  // namespace frame_utils
