/*
 * Added by Vincent
 * Sat Sept 5 17:32:58 CST 2020
 */
#pragma once
#include <algorithm>
#include <array>
#include <utility>
#include <memory>
#include <utility>

namespace input_source {

using ushort = unsigned short;
using uchar = unsigned char;

template <class T>
struct Vec2_ {
  union {
    T data[2];
    struct {T x, y;};
    struct {T u, v;};
    struct {T w, h;};
  } ;
};
using Vec2u = Vec2_<unsigned>;
using Vec2f = Vec2_<float>;

template <class T>
struct Vec3_ {
  union {
    T data[3];
    struct {T x, y, z;};
    struct {T r, g, b;};
    struct {T roll, pitch, yaw;};
    struct {T t3_x, t3_y, t3_yaw;};
  };

  T &operator[](unsigned i);
  const T &operator[](unsigned i) const;
  Vec3_<T> operator+(const Vec3_<T> &instance) const;
  Vec3_<T> operator-(const Vec3_<T> &instance) const;
  Vec3_<T> operator*(T scale) const;
  Vec3_<T> operator/(T scale) const;
};
using Vec3u = Vec3_<unsigned>;
using Vec3f = Vec3_<float>;

class StreamData {
 public:
  StreamData() = default;
  StreamData(float _time_stamp)
      :time_stamp(_time_stamp) {}
  virtual ~StreamData() {};

  virtual auto interpolation(const StreamData *next, float aim_time) -> std::shared_ptr<StreamData> = 0;

  float time_stamp;
};

// ImageFrame
static constexpr auto depth_width = 224;
static constexpr auto depth_height = 172;
static constexpr auto frame_len = depth_width * depth_height;

template<class Pixel, unsigned W, unsigned H>
struct ImageFrame : public StreamData {
  static constexpr auto width = W;
  static constexpr auto height = H;

  static constexpr auto DataLen = W * H;
  using pixel = Pixel;
  using FrameData = std::array<Pixel, DataLen>;

  // 3个构造函数
  ImageFrame<Pixel, W, H>() = default;

  ImageFrame<Pixel, W, H>(float _timestamp, const FrameData &_data);

  ImageFrame<Pixel, W, H>(float _time_stamp, const FrameData &&_data);

  // 析构函数
  ~ImageFrame<Pixel, W, H>() {};

  auto interpolation(const StreamData *next, float aim_time) -> std::shared_ptr<StreamData>;

  FrameData data;
};

template<class Pixel, unsigned W, unsigned H>
ImageFrame<Pixel, W, H>::ImageFrame(float _time_stamp, const FrameData &_data)
    :StreamData(_time_stamp), data(_data) {}

template<class Pixel, unsigned W, unsigned H>
ImageFrame<Pixel, W, H>::ImageFrame(float _time_stamp, const FrameData &&_data)
    :StreamData(_time_stamp), data(std::forward<const FrameData>(_data)) {}

// PoseFrame
template <bool is_6_dof>
struct PoseType {
  using TYPE = void*;
};

template <>
struct PoseType<true> {
  using Type = union {
    struct {
      Vec3f t;
      Vec3f r;
    };
    float data[6];
  };
};

template <>
struct PoseType<false> {using  Type = Vec3f;};

template <unsigned Dof>
struct PoseFrame : StreamData {
  static constexpr auto dof = Dof;
  using PoseData = typename PoseType<Dof == 6>::Type;

  PoseFrame<Dof>() = default;

  PoseFrame<Dof>(float _time_stamp, const PoseData &_pose)
      :StreamData(_time_stamp), pose(_pose) {}

  ~PoseFrame<Dof>() {};

  auto interpolation(const StreamData *next, float aim_time) -> std::shared_ptr<StreamData>;

  PoseData pose;
};
using PoseFrame3Dof = PoseFrame<3>;
using PoseFrame6Dof = PoseFrame<6>;

}  // namespace input_source
