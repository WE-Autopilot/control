/**
 * Ackermann controller class.
 * This is not a type of icontroller, probably should rename that to be clearer.
 * Created: Nov. 11, 2025
 * Author(s): Aly Ashour
 */

#ifndef AP1_ACKERMANN_CONTROLLER_HPP
#define AP1_ACKERMANN_CONTROLLER_HPP

#include <string>
#include <vector>

namespace ap1::control
{
class AckermannController
{
  public:
    struct Config
    {
        double L;             // wheelbase in meters
        double a_max;         // max longitudinal accleration (ms^-2)
        double delta_max;     // max steering angle
        double throttle_gain; // scales accel to throttle [-1, 1]
    };

    explicit AckermannController(const Config &cfg);

    struct Command
    {
        double throttle; // normalized [-1, 1]
        double steering; // in rads
    };

    Command compute_command(const std::vector<float> &acc, const std::vector<float> &vel);

    static Config load_config(const std::string &path);

  private:
    Config cfg_;
};
} // namespace ap1::control

#endif // AP1_ACKERMANN_CONTROLLER
