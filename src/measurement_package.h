#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

/**
 * Represents a single measurement. Contains the measurement itself and alsow a ground truth vector
 */
class MeasurementPackage
{
    public:
        long long timestamp_;

        enum SensorType
        {
            LASER = 1,
            RADAR = 2,
        }
        sensor_type_;

        //Can be Vector(2) for lasers in which case it contains x and y,
        //or Vector(3) for radars in which case it contains ro, phi, and ro_dot
        Eigen::VectorXd raw_measurements_;

        //Vector(4) contains x, y, vx, vy
        Eigen::VectorXd ground_truth_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
