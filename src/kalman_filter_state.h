#ifndef EXTENDEDKF_KALMAN_FILTER_STATE_H
#define EXTENDEDKF_KALMAN_FILTER_STATE_H

#include "Eigen/Dense"

/**
 * Represents Kalman filter state that can be shared and updated by multiple filters.
 */
class KalmanFilterState
{
    public:
        //state vector
        Eigen::VectorXd x_;

        //state covariance matrix
        Eigen::MatrixXd P_;
};

#endif //EXTENDEDKF_KALMAN_FILTER_STATE_H
