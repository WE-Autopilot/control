/**
 * Abstract class (interface) defining what a controller is.
 * Date: Nov. 10, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_CONTROLLER_HPP
#define AP1_CONTROLLER_HPP

#include <vector>

#include "vectors.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace ap1::control
{
class IController
{
  public:
    ~IController() = default;
    virtual vec3 compute_acceleration(vec3 vel, geometry_msgs::msg::Point target_pos, float target_speed) = 0;
};
} // namespace ap1::control

#endif // AP1_CONTROLLER_HPP
