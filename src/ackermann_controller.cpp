/**
 * Ackermann Controller implementation bbgl
 * Created: Nov. 11, 2025
 * Author(s): Aly Ashour
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "ap1/control/ackermann_controller.hpp"

#define EPSILON 1e-3

namespace ap1::control
{

// ideally we should have a vector class with all of these functions defined in it
float mag(const std::vector<float>& v)
{
    if (v.size() < 3)
        return 0;

    return std::sqrt(std::pow(v.at(0), 2) + std::pow(v.at(1), 2) + std::pow(v.at(2), 2));
}

std::vector<float> norm(const std::vector<float>& v)
{
    if (v.size() < 3)
        return {};

    const float m = mag(v);

    if (m == 0)
        return {};

    return {v.at(0) / m, v.at(1) / m, v.at(2) / m};
}

// also this should be moved to a csv/config loader
std::unordered_map<std::string, double> load_csv_config(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open config file: " + path);
    }

    std::unordered_map<std::string, double> config;
    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream ss(line);
        std::string key, value_str;
        if (std::getline(ss, key, ',') && std::getline(ss, value_str))
        {
            try
            {
                double value = std::stod(value_str);
                config[key] = value;
            }
            catch (const std::exception& e)
            {
                throw std::runtime_error("Invalid vlaue for key " + key + ": " + value_str);
            }
        }
    }

    return config;
}

AckermannController::AckermannController(const Config& cfg) : cfg_(cfg)
{
    printf("Created Ackermann Controller with config: a_max=%f, delta_max=%f, L=%f, "
           "throttle_gain=%f\n",
           cfg_.a_max, cfg_.delta_max, cfg_.L, cfg_.throttle_gain);
}

// we should move everything over to double but I already wrote all the message types in float
// and I'm too lazy to switch so we'll do it later
AckermannController::Command AckermannController::compute_command(
    // notice all vectors are literall std::vector<double>'s in the codebase so far
    // this is beyond cooked but will do for now.
    // vec[0] = x, vec[1] = y, vec[2] = z, and anything else is wabisabi
    const std::vector<float>& acc, const std::vector<float>& vel)
{
    Command cmd{};

    double a_long = std::clamp((double)acc.at(0), -cfg_.a_max, cfg_.a_max);
    double a_lat = acc[1];

    double v = mag(vel);
    double kappa = (v > EPSILON) ? a_lat / (v * v) : 0.0;
    double delta = std::atan(cfg_.L * kappa);
    delta = std::clamp(delta, -cfg_.delta_max, cfg_.delta_max);

    double throttle = std::clamp(a_long / cfg_.a_max * cfg_.throttle_gain, -1.0, 1.0);

    cmd.throttle = throttle;
    cmd.steering = delta;

    return cmd;
}

AckermannController::Config AckermannController::load_config(const std::string& path)
{
    Config cfg;

    auto map = load_csv_config(path);

    if (map.count("wheelbase"))
        cfg.L = map["wheelbase"];
    if (map.count("a_max"))
        cfg.a_max = map["a_max"];
    if (map.count("delta_max"))
        cfg.delta_max = map["delta_max"];
    if (map.count("throttle_gain"))
        cfg.throttle_gain = map["throttle_gain"];

    return cfg;
}
} // namespace ap1::control
