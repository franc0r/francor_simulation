#include "PidController.h"

#include <ros/ros.h>

namespace nrbdl {

PidController::PidController(const double k_p, const double k_i, const double k_d)
    : k_p_(k_p),
      k_i_(k_i),
      k_d_(k_d),
      limit_max_u_(std::numeric_limits<double>::max()),
      limit_min_u_(-std::numeric_limits<double>::max())
{
    this->reset();
}

void PidController::reset(void)
{
    // Only reset working variables and keep the controller parameters.
    error_k0_ = 0.0;
    error_k1_ = 0.0;
    error_k2_ = 0.0;

    u_k0_ = 0.0;
    u_k1_ = 0.0;

    target_ = 0.0;
}

void PidController::setLimits(const double minOutput, const double maxOutput)
{
    // Check if the parameters are valid.
    if (minOutput > maxOutput)
    {
        ROS_ERROR("PidController don't accept the limits. The provided min is greater then max.");
        return;
    }

    // Assign.
    limit_min_u_ = minOutput;
    limit_max_u_ = maxOutput;

    // Repsect the new limits on the current output value.
    this->checkLimits();
}

void PidController::setOutputToValue(const double value)
{
    // Set all output values to "value".
    u_k0_ = value;
    u_k1_ = value;

    // Clear all errors, so the history makes no trouble.
    error_k0_ = 0.0;
    error_k1_ = 0.0;
    error_k2_ = 0.0;

    // Check if the output is out of range.
    this->checkLimits();
}

void PidController::process(const double input)
{
    // First calculate the error and copy the old ones.
    error_k2_ = error_k1_;
    error_k1_ = error_k0_;
    error_k0_ = target_ - input;

    // Calculate the new output value according the controller parameters (K_p, K_i and K_d).
    u_k1_ = u_k0_;
    u_k0_ = u_k1_ + (k_p_ + k_i_ + k_d_) * error_k0_ + (-k_p_ - 2.0 * k_d_) * error_k1_ + k_d_ * error_k2_;

    // At least check if the output is out of range.
    this->checkLimits();
}

void PidController::checkLimits(void)
{
    // If the output is out of limit then correct the output to the correspond limit.
    if (u_k0_ > limit_max_u_)
        u_k0_ = limit_max_u_;
    else if (u_k0_ < limit_min_u_)
        u_k0_ = limit_min_u_;
}

} // end namespace nrbdl
