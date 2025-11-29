/**
 * Ackermann controller class.
 * This is not a type of icontroller, probably should rename that to be clearer.
 * Created: Nov. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_ACKERMANN_CONTROLLER_HPP
#define AP1_ACKERMANN_CONTROLLER_HPP

#include <string>
#include "vectors.hpp"

namespace ap1::control
{
class AckermannController
{
  public:
    struct Config
    {
        // Added default values for safety if config file fails to load
        double L = 0.33;             // wheelbase in meters
        double a_max = 2.0;          // max longitudinal accleration (ms^-2)
        double delta_max = 0.5;      // max steering angle (~28 deg)
        double throttle_gain = 1.0;  // scales accel to throttle [-1, 1]
    };

    explicit AckermannController(const Config &cfg);

    struct Command
    {
        double throttle; // normalized [-1, 1]
        double steering; // in rads
    };

    Command compute_command(const vec3f& acc, const vec3f& vel);

    static Config load_config(const std::string &path);

  private:
    Config cfg_;
};
} // namespace ap1::control

#endif // AP1_ACKERMANN_CONTROLLER
