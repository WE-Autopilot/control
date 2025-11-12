/**
 * PD Controller Implementation
 * Date: Nov. 10, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_PD_CONTROLLER_HPP
#define AP1_PD_CONTROLLER_HPP

#include <cmath>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

#include "ap1/control/icontroller.hpp"

#define EPSILON 1e-6

// THESE SHOULD NOT BE HERE GANG
// very uncouth.
inline float length(geometry_msgs::msg::Point p)
{
    return std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));
}

inline geometry_msgs::msg::Point get_norm(geometry_msgs::msg::Point p)
{
    const float l = length(p);
    geometry_msgs::msg::Point p2;
    p2.x = p.x / l;
    p2.y = p.y / l;
    p2.z = p.z / l;
    return p2;
}

namespace ap1::control
{
class PDController : public IController
{
    float kp_, kd_;

  public:
    PDController(float kp = 2.0, float kd = 0.5) : kp_(kp), kd_(kd){};

    // computes accceleration according to a pid controller
    // currently stores vectors half in the ROS *Point* format and half in
    // std::vectors. This is super not sigma and we should isolate out both with a
    // vec class for all of ap1 (since ros2 points don't have scalar mult, dot, or
    // cross or anything).
    virtual std::vector<float> compute_acceleration(std::vector<float> vel,
                                                    geometry_msgs::msg::Point target_pos,
                                                    float target_speed) override
    {
        // CHECKS
        // if vel contains less than 3 vals throw an error
        if (vel.size() < 3)
        {
            throw std::invalid_argument("vel must have at least three elements (x, y, z).");
        }

        const float distance = length(target_pos);
        geometry_msgs::msg::Point target_dir;

        if (distance > EPSILON)
            target_dir = get_norm(target_pos);
        else
        {
            target_dir.x = 0;
            target_dir.y = 0;
            target_dir.z = 0;
        }

        float target_vel_x = target_dir.x * target_speed;
        float target_vel_y = target_dir.y * target_speed;
        float target_vel_z = target_dir.z * target_speed;

        std::vector<float> acc{kp_ * (target_vel_x - vel.at(0)) - kd_ * vel.at(0),
                               kp_ * (target_vel_y - vel.at(1)) - kd_ * vel.at(1),
                               kp_ * (target_vel_z - vel.at(2)) - kd_ * vel.at(2)};

        return acc;
    };
};
} // namespace ap1::control

#endif // AP1_PD_CONTROLLER_HPP
