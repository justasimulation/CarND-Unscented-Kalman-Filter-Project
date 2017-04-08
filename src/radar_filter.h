#ifndef EXTENDEDKF_RADAR_FILTER_H
#define EXTENDEDKF_RADAR_FILTER_H

#include "Eigen/Dense"

#include "unscented_kalman_filter.h"

/**
 * Represetns a particular filter for processing radar measurements.
 */
class RadarFilter : public UnscentedKalmanFilter
{
    public:
        /**
         * Constructor. Initializes noise matrix, Jacobian and memorizes given state
         *
         * @param state
         */
        RadarFilter(KalmanFilterState &state, double const v_dot_std, double const yaw_dot_dot_std);

        /**
         * Normalizes measurements[1] values which is yaw(phi) so it is located in the range [-PI, PI]
         * @param vector
         */
        void NormalizeMeasurement(Eigen::VectorXd &vector) const override;


        /**
         * Converts raw measuremnt Vector(3) to cartesian Vector(4).
         * Radar measures only ro, phi and ro_dot so the need to be converted to x, y, vx, vy
         *
         * @param raw_measurement - Vector(3) of ro, phi, ro_dot
         * @param x - Vector(4) of x, y, vx, vy
         */
        void ConvertMeasurementToCartesian(const Eigen::VectorXd &measurement, Eigen::VectorXd &x) const override;

        /**
         * Converts raw measurment Vector(3) to state vector (x, y, v, yaw, yaw_rate)
         * @param measurement - Vector(3)
         * @param x - Initialized Vector(5), on output it will contain x, y, v, yaw, yaw_rate
         */
        void ConvertMeasurementToState(const Eigen::VectorXd &measurement, Eigen::VectorXd &x) const override;

        /**
         * Converts state Vector(5) to measurement Vector(3). Sometimes conversion is not possible,
         * e.g. becuase of devision by zero, in this case false is returned.
         * @param state - Vector(5) with x, y, v, yaw, yaw_rate
         * @param measurement - Not initialized Vector(3) on output it will contain ro, phi, ro_dot
         * @return true if conversion was successfull, false otherwise
         */
        bool ConvertStateToMeasurement(const Eigen::VectorXd &state, Eigen::VectorXd &measurement) const override;

};

#endif //EXTENDEDKF_RADAR_FILTER_H
