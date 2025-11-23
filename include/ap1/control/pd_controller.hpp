/**
 * PD Controller Implementation
 * Date: Nov. 10, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_PD_CONTROLLER_HPP
#define AP1_PD_CONTROLLER_HPP

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

#include "ap1/control/icontroller.hpp"
#include "vectors.hpp"

#define EPSILON 1e-6

namespace ap1::control
{
class PDController : public IController
{
    // Stength of the proportional and derivative terms
    float kp_, kd_;
    // Last velocity vector
    vec3f last_vel_;

  public:
    PDController(float kp = 2.0, float kd = 0.5) : kp_(kp), kd_(kd) {};

    // Computes acceleration based on current velocity, target position, and target speed. Returns
    // acceleration vector.
    virtual vec3f compute_acceleration(const vec3f& vel,
                                       const vec2f& target_pos,
                                       const float target_speed) override
    {
        // Convert target position to vec3f and calculate distance
        const float distance = target_pos.length();

        // targetDir for normalizing target position into unit direction
        vec2f targetDir;
        if (distance > EPSILON)
            targetDir = target_pos.unit_vector();
        else
        {
            targetDir = vec2f(0.0, 0.0);
        }

        // Scale direction vector by target speed for velocity
        vec3f targetVel = targetDir * target_speed;

        std::cout << "target_dir_x (SHOULD BE 1): " << targetDir.x
                  << ", target_dir_y: " << targetDir.y
                  << "\ntarget_vel_x (should be target speed): " << targetVel.x << "\n";

        // Store current velocity for derivative approximation
        last_vel_ = vel;

        // Calculate velocity change (numerical derivative)
        vec3f drv = vel - last_vel_;

        // PD controller: proportional term tracks target velocity, derivative term adds damping
        return kp_ * (targetVel - vel) - kd_ * drv;
    };
};
} // namespace ap1::control

#endif // AP1_PD_CONTROLLER_HPP
