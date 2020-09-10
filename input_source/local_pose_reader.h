/*
 * Added by Vincent
 * Mon Sept 7 14:22:16 CST 2020
 */

#pragma once

#include "local_type.h"
#include "stream_reader.h"
#include <memory>

namespace input_source {

class LocalPoseReader : public PoseStreamReader<3> {
  struct PrivateData;

 public:
  LocalPoseReader(const char *file_pose);
  ~LocalPoseReader();

  bool reachEnd() const;
        
  bool getPose(DataType &pose_frame);

  bool peep(DataType &pose_frame);
    
 private:
  std::shared_ptr<PrivateData> data_pose{nullptr};  // 保存读取的位姿数据
  bool is_valid{false};  // 检查二进制位姿文件是否正确读取
};

}  // namespace input_source
