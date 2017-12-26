/*
 * Brief : This controller class represents a PID controller. It works with the equation
 *         U(z) = [ K_p + K_i / (1-z^-1) + K_d (1-z^-1) ] * E(z)
 *
 *                [ (K_p + K_i + K_d) + (-K_p - 2K_d)z^-1 + K_d * z^-2 ]
 *              = [----------------------------------------------------] * E(z)
 *                [                       1-z^-1                       ]
 *
 *  ==>    u[k] = u[k-1] + (K_p + K_i + K_d) * e[k] + (-K_p - 2 * K_d) * e[k-1] + K_d * e[k-2]
 *
 * Author: Christian Merkl (knueppl@gmx.de)
 * Date  : 26.12.2017
 */
#ifndef ___FRANCOR_PID_CONTROLLER_H___
#define ___FRANCOR_PID_CONTROLLER_H___

namespace nrbdl {

class PidController
{
public:
	/**
	 * \brief Constructor. Brings up a PID controller without a output limitation. The target value is zero by
	 *        default. The controller output starts at zero.
	 * \param k_p The P parameter of this controller.
	 * \param k_i The I parameter of this controller.
	 * \param k_d The D parameter of this controller.
	 */
	PidController(const double k_p = 1.0, const double k_i = 0.0, const double k_d = 0.0);
	PidController(const PidController&) = default;
	PidController(PidController&&) = default;
	~PidController(void) = default;

	/**
	 * \brief Resets this controller. The output and target are set to zero. The controller keeps the old settings.
	 */
	void reset(void);
	/**
	 * \brief Set the limits (min and max) of the output.
	 * \param minOutput The minimum value of the output.
	 * \param maxOutput The maximum value of the output.
	 */
	void setLimits(const double minOutput, const double maxOutput);
	/**
	 * \brief Set the controller target value.
	 * \param target The controller tries to reach this value by changing the output value.
	 */
	inline void setTarget(const double target) { target_ = target; }
	/**
	 * \brief Sets the output to the given value. All older values of the output and error will be overwritten,
	 *        so the history hasn't an effect.
	 * \param value The new value of the output.
	 */
	void setOutputToValue(const double value);
	/**
	 * \brief Processes the next time step. This method should be called periodically without any time variation.
	 *        The time between the calls should be constant.
	 * \parma input The new input value for the next time step.
	 */
	void process(const double input);
	/**
	 * \brief Returns the last calculated output value.
	 * \return The current output value. The value is range checked if limits are set.
	 */
	inline double output(void) const { return u_k0_; }

	// Assign operators.
	PidController& operator=(const PidController&) = default;
	PidController& operator=(PidController&&) = default;

private:
	/**
	 * \brief Checks and corrects the output if the output is out of range.
	 */
	void checkLimits(void);

	// Controller parameters P,I,D,limits and target value.
	double k_p_;
	double k_i_;
	double k_d_;

	double limit_max_u_;
	double limit_min_u_;
	double target_;
	// TODO: add a limit for the I component.

	// Working variables of the controller.
	double error_k0_;
	double error_k1_;
	double error_k2_;
	double u_k0_;
	double u_k1_;
};

} // end namespace nrbdl

#endif
