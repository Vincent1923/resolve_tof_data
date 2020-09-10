/*
 * Added by Vincent
 * Fri Sept 4 15:03:08 CST 2020
 */

#pragma once
#include "local_type.h"
#include "stream_reader.h"
#include <memory>

namespace input_source {

class LocalDepthReader : public ImageStreamReader<unsigned short, depth_width, depth_height> {
  struct PrivateData;

 public:
  LocalDepthReader(const char *file_imag);
  ~LocalDepthReader();

  // 检查是否已读取完深度图
  bool reachEnd() const;

  // Vec2u getImageSize() const;

  bool getImage(DataType &depth_frame);

 private:
  std::shared_ptr<PrivateData> data_depth{nullptr};  // 保存读取的深度图数据
  bool is_valid{false};  // 检查二进制深度图文件是否正确读取
};

}  // namespace input_source
