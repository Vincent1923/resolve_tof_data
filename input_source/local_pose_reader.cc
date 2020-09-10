/*
 * Added by Vincent
 * Mon Sept 7 14:26:38 CST 2020
 */

#include "local_pose_reader.h"
#include "local_type.h"
#include <fstream>
#include <list>
#include <memory>
#include <iostream>

using namespace input_source;

struct LocalPoseReader::PrivateData {
  static constexpr auto frame_size = DataType::dof * sizeof(float);
  static constexpr auto time_len = sizeof(float);

  std::ifstream in_pose;   // 位姿的文件流
  unsigned long data_len;  // 位姿文件的长度

  std::list<DataType> frame_list;  // 存放每一帧位姿的队列，没执行一次 readFrame() 函数，就会往队列添加一帧位姿

  // 构造函数
  PrivateData(const char* pose_file) {
    this->in_pose.open(pose_file);

    if (this->in_pose.is_open()) {
      this->in_pose.seekg(0, std::ios::end);
      this->data_len = this->in_pose.tellg();
      this->in_pose.seekg(0);
    }
  }

  // 读取一帧位姿及时间戳
  bool readFrame() {
    if (!this->in_pose.is_open())
      return false;

    unsigned pos = this->in_pose.tellg();
    if ((pos + frame_size + time_len) >= this->data_len)
      return false;

    float time;
    DataType::PoseData data;

    this->in_pose.read((char*)&time, this->time_len);
    this->in_pose.read((char*)data.data, this->frame_size);

    this->frame_list.emplace_back(time, data);
    return true;
  }
};

LocalPoseReader::LocalPoseReader(const char* file_pose) {
  // 新建一个指向类型 PrivateData 的智能指针，并读取二进制位姿文件 file_pose 的数据，
  // 最后把新建的指针赋值给成员变量 data_pose
  this->data_pose = std::make_shared<PrivateData>(file_pose);
  // 检查二进制位姿文件 file_image 是否正确读取
  this->is_valid = this->data_pose->in_pose.is_open();
}

LocalPoseReader::~LocalPoseReader() {}

bool LocalPoseReader::reachEnd() const {
  return !this->data_pose->readFrame();
}

// 从位姿队列获取一帧位姿，并且如果获取成功，则把队首元素移出队列。
bool LocalPoseReader::getPose(DataType &pose_frame) {
  bool got_pose = this->peep(pose_frame);
  if (got_pose)
    this->data_pose->frame_list.pop_front();

  return got_pose;
}

// 从位姿队列获取一帧位姿。
// 若位姿数据 data_pose 中的位姿队列 frame_list 为空，则首先从位姿文件流读取一帧位姿放入位姿队列中，再获取位姿队列的队首元素；
// 若位姿队列不为空，则直接获取位姿队列的队首元素。
bool LocalPoseReader::peep(DataType &pose_frame) {
  bool got_pose = false;
  if (this->is_valid) {  // 首先检查位姿文件是否正确读取
    if (this->data_pose->frame_list.empty())
      // 检查位姿队列是否为空，若是，则读取一帧位姿，并放在位姿队列中
      this->data_pose->readFrame();
    if (!this->data_pose->frame_list.empty()) {
      // 若位姿队列不为空，则把队列的队首元素赋值给 pose_frame
      pose_frame.time_stamp = this->data_pose->frame_list.front().time_stamp;
      pose_frame.pose = this->data_pose->frame_list.front().pose;
      got_pose = true;
    }
  }

  return got_pose;
}
