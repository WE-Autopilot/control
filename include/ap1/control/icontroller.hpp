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
    virtual vec3f compute_acceleration(const vec3f& vel, const vec2f& target_pos, const float target_speed) = 0;
};
} // namespace ap1::control

#endif // AP1_CONTROLLER_HPP
