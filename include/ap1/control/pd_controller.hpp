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
# include <vector>

#include "vectors.hpp"

#include "geometry_msgs/msg/point.hpp"

#include "ap1/control/icontroller.hpp"

#define EPSILON 1e-6

namespace ap1::control
{
class PDController : public IController
{
    // Stength of the proportional and derivative terms
    float kp_, kd_;
    //Last velocoty vector
    vec3 last_vel_;

  public:
    PDController(float kp = 2.0, float kd = 0.5) : kp_(kp), kd_(kd) {};

    // Computes acceleration based on current velocity, target position, and target speed. Returns acceleration vector.
    virtual vec3 compute_acceleration(vec3 vel, geometry_msgs::msg::Point target_pos, float target_speed) override
    {
        // Convert target position to vec3 and calculate distance
        vec3 vecTargetPos(target_pos.x, target_pos.y, target_pos.z);
        const float distance = vecTargetPos.length();

        // targetDir for normalizing target position into unit direction
        vec3 targetDir;
        if (distance > EPSILON)
            targetDir = unit_vector(vecTargetPos);
        else
        {
            targetDir = vec3(0.0, 0.0, 0.0);
        }

        // Scale direction vector by target speed for velocity
        vec3 targetVel = targetDir * target_speed;

        std::cout << "target_dir_x (SHOULD BE 1): " << targetDir.x << ", target_dir_y: " << targetDir.y
                  << "\ntarget_vel_x (should be target speed): " << targetVel.x << "\n";

        // Store current velocity for derivative approximation
        last_vel_ = vel;

        // Calculate velocity change (numerical derivative)
        vec3 drv = vel - last_vel_;

        // PD controller: proportional term tracks target velocity, derivative term adds damping
        vec3 acc = kp_* (targetVel - vel) - kd_ * drv;

        return acc;
    };
};
} // namespace ap1::control

#endif // AP1_PD_CONTROLLER_HPP
