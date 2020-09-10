/*
 * Added by Vincent
 * Sat Sept 5 15:40:02 CST 2020
 */
#include "local_type.h"
#include <cmath>
#include <memory>

namespace input_source {

template class input_source::Vec3_<float>;

    template <class T>
T &Vec3_<T>::operator[](unsigned int i) {

    return this->data[i];
}

    template <class T>
const T &Vec3_<T>::operator[](unsigned int i) const {

    return this->data[i];
}

    template <class T>
Vec3_<T> Vec3_<T>::operator+(const Vec3_<T> &instance) const {

    Vec3_<T> result;
    for (auto i = 0; i < 3; ++i)
        result[i] = (*this)[i] + instance[i];
    return result;
}

    template <class T>
Vec3_<T> Vec3_<T>::operator-(const Vec3_<T> &instance) const {

    Vec3_<T> result;
    for (auto i = 0; i < 3; ++i)
        result[i] = (*this)[i] - instance[i];
    return result;
}

    template <class T>
Vec3_<T> Vec3_<T>::operator/(T scale) const {

    Vec3_<T> result;
    for (auto i = 0; i < 3; ++i)
        result[i] = (*this)[i] / scale;
    return result;
}

    template <class T>
Vec3_<T> Vec3_<T>::operator*(T scale) const {

    Vec3_<T> result;
    for (auto i = 0; i < 3; ++i)
        result[i] = (*this)[i] * scale;
    return result;
}

    template<>
auto ImageFrame<unsigned short, depth_width, depth_height>::interpolation(const StreamData *newer, float aim_time) -> std::shared_ptr<StreamData> {

    using DataType = ImageFrame<unsigned short, depth_width, depth_height>;

    auto older = this;
    auto instance = std::fabs(older->time_stamp - aim_time) < std::fabs(newer->time_stamp - aim_time)? static_cast<const DataType*>(older): static_cast<const DataType*>(newer);

    return std::make_shared<DataType>(instance->time_stamp, instance->data);
}

    template <>
auto PoseFrame<3>::interpolation(const StreamData *newer, float aim_time) -> std::shared_ptr<StreamData> {

    const StreamData *older = this; 
    
    auto is_older = std::fabs(older->time_stamp - aim_time) < std::fabs(newer->time_stamp - aim_time);
    auto time_ratio = (aim_time - older->time_stamp) / (newer->time_stamp - older->time_stamp);

    auto i_older = static_cast<const PoseFrame3Dof*>(older);
    auto i_newer = static_cast<const PoseFrame3Dof*>(newer);

    auto result = i_older->pose + ((i_newer->pose - i_older->pose) * time_ratio);
    return std::make_shared<PoseFrame3Dof>(aim_time, result);
}

    template <>
auto PoseFrame<6>::interpolation(const StreamData *newer, float aim_time) -> std::shared_ptr<StreamData> {

    const StreamData *older = this;
    
    auto is_older = std::fabs(older->time_stamp - aim_time) < std::fabs(newer->time_stamp - aim_time);
    auto time_ratio = (aim_time - older->time_stamp) / (newer->time_stamp - older->time_stamp);
    auto i_older = static_cast<const PoseFrame6Dof*>(older);
    auto i_newer = static_cast<const PoseFrame6Dof*>(newer);

    PoseFrame6Dof::PoseData result;
    result.t = i_older->pose.t + ((i_newer->pose.t - i_older->pose.t) * time_ratio);
    result.r = i_older->pose.r + ((i_newer->pose.r - i_older->pose.r) * time_ratio);

    return std::make_shared<PoseFrame6Dof>(aim_time, result);
}

}  // namespace input_source
