#include "Eigen/Dense"

#include "laser_filter.h"

using namespace Eigen;

/**
 * Constructor. Initializes laser measurement noise and measurement matices and memorizes the state.
 * @param state
 */
LaserFilter::LaserFilter(KalmanFilterState &state, const double v_dot_std, const double yaw_dot_dot_std) :
                         UnscentedKalmanFilter(state, v_dot_std, yaw_dot_dot_std)
{
    R_ = MatrixXd(2, 2);
    R_ <<   0.15 * 0.15, 0,
            0,      0.15 * 0.15;
}

/**
 * Converts state Vector(5) to measurement Vector(2) of x, y
 * @param state Vector(5) of x, y, v, yaw, yaw_rate
 * @param measurement Not initialized vector, that will be Vector(2) with x, y on output
 * @return always true in case of this filter
 */
bool LaserFilter::ConvertStateToMeasurement(const Eigen::VectorXd &state, VectorXd &measurement) const
{
    measurement = VectorXd(2);
    measurement(0) = state(0);
    measurement(1) = state(1);

    return true;
}

/**
 * Converts raw measuremnt vector to Vector(4) cartesian vector.
 * Laser measures only x, y so velocities are set to zero.
 *
 * @param raw_measurement - Vector(2) of x and y
 * @param x - Vector(4)
 */
void LaserFilter::ConvertMeasurementToCartesian(const VectorXd &raw_measurement, VectorXd &x) const
{
    x << raw_measurement(0), raw_measurement(1), 0, 0;
}

/**
 * Converts measurement Vector(2) to state Vector(5) of x, y, v, yaw, yaw_rate
 * @param raw_measurement - Vector(2) of x, y
 * @param x - Initialized Vector(5) which will contain x, y, v, yaw. yaw_rate on output
 */
void LaserFilter::ConvertMeasurementToState(const VectorXd &raw_measurement, VectorXd &x) const
{
    x << raw_measurement(0), raw_measurement(1), 0, 0, 0;
}
