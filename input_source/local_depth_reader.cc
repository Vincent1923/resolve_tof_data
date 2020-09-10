/*
 * Added by Vincent
 * Fri Sept 4 15:06:19 CST 2020
 */

#include "local_depth_reader.h"
#include "local_config.h"
#include "local_type.h"

#include <fstream>
#include <iostream>
#include <list>
#include <array>

// #include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace input_source {

struct LocalDepthReader::PrivateData {
  static constexpr auto frame_size = DataType::DataLen * sizeof(DataType::pixel);
  static constexpr auto time_len = sizeof(float);

  std::ifstream in_depth;          // 深度图像的文件流
  unsigned long data_len{0};       // 深度图像文件的长度
  std::list<DataType> frame_list;  // 存放每一帧深度图的队列，每执行一次 readFrame() 函数，就会往队列添加一帧深度图

  // 构造函数
  PrivateData(const char *depth_file) {
    this->in_depth.open(depth_file);

    if (this->in_depth.is_open()) {
      this->in_depth.seekg(0, std::ios::end);
      this->data_len = in_depth.tellg();
      this->in_depth.seekg(0);
    }
  }

  // 读取一帧深度图及时间戳
  bool readFrame() {
    // 检查是否正确打开图像文件
    if (!this->in_depth.is_open())
        return false;

    unsigned pos = this->in_depth.tellg();
    if ((pos + frame_size + time_len) >= this->data_len)
      return false;

    float time;                // 时间戳
    DataType::FrameData data;  // 一帧深度图
    this->in_depth.read((char*)&time, this->time_len);
    this->in_depth.read((char*)data.data(), this->frame_size);

    // 深度图每一个像素数值的最高3位不使用，需要置0
    for (int i = 0; i < DataType::DataLen; i++) {
      data[i] = data[i] << 3;
      data[i] = data[i] >> 3;
      if (data[i] > 5000)
          data[i] = 0;
    }

    this->frame_list.emplace_back(time, std::move(data));

    // 显示一帧深度图
    
    cv::Mat image(depth_height, depth_width, CV_16UC1, data.data());
    cv::Mat show_image;
    cv::convertScaleAbs(image, show_image, 255. / 2000);
    cv::imshow("frame", show_image);
    cv::waitKey(10);
    

    return true;
  }

  void copyTo(unsigned short *depth) {
    if (this->frame_list.empty())
      return;

    auto &frame = this->frame_list.front();
    auto &frame_data = frame.data;

    std::copy(frame_data.begin(), frame_data.end(), depth);
  }
};

LocalDepthReader::LocalDepthReader(const char *file_image) {
  std::cout << "into LocalDepthReader" << std::endl;
  // 新建一个指向类型 PrivateData 的智能指针，并读取二进制深度图文件 file_image 的数据，
  // 最后把新建的指针赋值给成员变量 data_depth
  this->data_depth = std::make_shared<PrivateData>(file_image);
  // 检查二进制深度图文件 file_image 是否正确读取
  this->is_valid = this->data_depth->in_depth.is_open();
}

LocalDepthReader::~LocalDepthReader() {}

bool LocalDepthReader::reachEnd() const {
  return !this->data_depth->readFrame();
}

// 从深度图队列获取一帧深度图，并且如果获取成功，则把队首元素移出队列。
bool LocalDepthReader::getImage(DataType &depth_frame) {
  // std::cout << "LocalDepthReader::getImage" << std::endl;
  // depth 的数据类型为 std::array<Pixel, DataLen>
	auto &depth = depth_frame.data;

	bool got_depth = false;
  if (this->is_valid) {  // 首先检查深度图文件是否正确读取
    if (this->data_depth->frame_list.empty())
      // 检查深度图队列是否为空，若是，则读取一帧深度图
      this->data_depth->readFrame();
    if (!this->data_depth->frame_list.empty()) {
      // 若深度图队列不为空，则把队列的队首元素赋值给 depth_frame，并把队首元素出队列。
      depth_frame.time_stamp = this->data_depth->frame_list.front().time_stamp;
      this->data_depth->copyTo(depth.data());  // depth.data() 返回一个指向 array 第一个元素位置的指针
      got_depth = true;

      this->data_depth->frame_list.pop_front();
    }
  }
  if (!got_depth) memset(depth.data(), 0, depth_frame.DataLen * sizeof(short));

  return got_depth;
}

}  // namespace input_source
