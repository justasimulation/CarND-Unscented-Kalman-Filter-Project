#ifndef EXTENDEDKF_LASER_FILTER_H
#define EXTENDEDKF_LASER_FILTER_H


#include "unscented_kalman_filter.h"

/**
 * Represents a particular filter for processing of laser measurments.
 */
class LaserFilter : public UnscentedKalmanFilter
{
    public:
        /**
         * Constructor. Initializes laser measurement noise and measurement matices and memorizes the state.
         * @param state
         */
        LaserFilter(KalmanFilterState &state, const double v_dot_std, const double yaw_dot_dot_std);

        /**
         * Converts raw measuremnt vector to Vector(4) cartesian vector.
         * Laser measures only x, y so velocities are set to zero.
         *
         * @param raw_measurement - Vector(2) of x and y
         * @param x - Vector(4) of x, y, vx, vy
         */
        void ConvertMeasurementToCartesian(const Eigen::VectorXd &raw_measurement, Eigen::VectorXd &x) const override;


        /**
         * Converts measurement Vector(2) to state Vector(5) of x, y, v, yaw, yaw_rate
         * @param raw_measurement - Vector(2) of x, y
         * @param x - Initialized Vector(5) which will contain x, y, v, yaw. yaw_rate on output
         */
        void ConvertMeasurementToState(const Eigen::VectorXd &raw_measurement, Eigen::VectorXd &x) const override;

        /**
         * Converts state Vector(5) to measurement Vector(2) of x, y
         * @param state Vector(5) of x, y, v, yaw, yaw_rate
         * @param measurement Not initialized vector, that will be Vector(2) with x, y on output
         * @return always true in case of this filter
         */
        bool ConvertStateToMeasurement(const Eigen::VectorXd &state, Eigen::VectorXd &measurement) const override;

};

#endif //EXTENDEDKF_LASER_FILTER_H
