/*
 * Added by Vincent
 * Sat Sept 5 16:04:06 CST 2020
 */

#pragma once

#include "local_type.h"

namespace input_source {

class StreamReader {
 public:
  virtual ~StreamReader() {};

  // virtual bool reachEnd() const = 0;
};

template <class Pixel, unsigned W, unsigned H>
class ImageStreamReader : public StreamReader {
 public:
  using DataType = ImageFrame<Pixel, W, H>;

  virtual ~ImageStreamReader<Pixel, W, H>() {};

  // virtual Vec2u getImageSize() const = 0;

  virtual bool getImage(DataType &image_frame) = 0;
};

template <unsigned Dof>
class PoseStreamReader : public StreamReader {
public:

    using DataType = PoseFrame<Dof>;

    virtual ~PoseStreamReader<Dof>() {};

    // virtual bool getPose(DataType &pose_frame) = 0;
};

}  // namespace input_source
