/*
 * Added by Vincent
 * Mon Sept 7 19:49:15 CST 2020
 */

#pragma once

#include <input_source/local_depth_reader.h>
#include <input_source/local_pose_reader.h>

namespace frame_utils {

template <class BaseStream, class RefStream>
class DataAlign {
  BaseStream &base_stream;  // 深度图解析器
  RefStream &ref_stream;    // 机器人位姿解析器

//   ImageProc<typename BaseStream::DataType> image_proc;

  bool is_filter;
 public:
  DataAlign<BaseStream, RefStream>(BaseStream &_base_stream, RefStream &_ref_stream, bool filter= false)
      :base_stream(_base_stream), ref_stream(_ref_stream), is_filter(filter) {}

  auto operator()(typename BaseStream::DataType &base_frame, typename RefStream::DataType &ref_frame) -> bool;

  bool reachEnd() {return this->base_stream.reachEnd();}
};

}  // namespace frame_utils
