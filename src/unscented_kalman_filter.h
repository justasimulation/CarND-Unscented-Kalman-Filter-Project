#ifndef UNSCENTEDKF_UNSCENTED_KALMAN_FILTER_H
#define UNSCENTEDKF_UNSCENTED_KALMAN_FILTER_H

#include "Eigen/Dense"

#include "kalman_filter_state.h"
#include "measurement_package.h"

class UnscentedKalmanFilter
{
    protected:
        // Measurement noise
        Eigen::MatrixXd R_;

    private:
        // Contains state vector and covariacne matrix
        KalmanFilterState &state_;

        // Predicted sigma points matrix
        Eigen::MatrixXd X_sig_pred_;

        // Process noise standard deviation longitudinal acceleration in m/s^2
        double std_a_;

        // Process noise standard deviation yaw acceleration in rad/s^2
        double std_yawdd_;

        // Weights of sigma points
        Eigen::VectorXd weights_;

        // State vector dimension
        int n_x_;

        // Augmented state vector dimension
        int n_aug_;

        // Number of sigma points
        int n_sigma_;

        // Sigma point spreading parameter
        double lambda_;

    public:
        /**
         * Constructor. Memorizes state object and initializes parameters.
         * @param state
         */
        UnscentedKalmanFilter(KalmanFilterState &state);

        /**
         * Performs prediction step of unscented kalman filter.
         * 1. Generates sigma points
         * 2. Applies process model transformation to them.
         * 3. Calculates mean and covariance of the transformed sigma points
         *
         * Transformed sigma points are saved into X_sig_pred_ so the can be reused by Update step.
         * @param delta_time - time passed from last measurement processing (seconds)
         */
        void Predict(const double delta_time);

        /**
         * Implements update step of unscented kalman filter. Returns NIS value.
         * 1. Transformes calculated in prediction step sigma points into measurement space,
         *    calculates mean and covariance of the points in the measurement space.
         * 2. Updates state vector and covariance.
         * 3. Calculates NIS
         * @param measurement - measurement vector
         * @return NIS value or -1 if update was unsuccessfull (e.g. because of possible division by zero)
         */
        double Update(const Eigen::VectorXd &measurement);

        /**
         * Converts measurment vector to cartesian form i.e. to px, py, vx, vy
         * This method should be implemented by particular filters which knows their measurement format.
         * @param raw_measurement - measurement vector
         * @param x - cartesian state vector
         */
        void virtual ConvertMeasurementToCartesian(const Eigen::VectorXd &raw_measurement,
                                                   Eigen::VectorXd &x) const = 0;

        /**
         * Converts measurement vector to kalman filter state vector, which may be cartision or not.
         * This method should be implemented by particular filters which knows their measurement format.
         * @param raw_measurement - measurement vector
         * @param x - state vector in current format
         */
        void virtual ConvertMeasurementToState(const Eigen::VectorXd &raw_measurement,
                                               Eigen::VectorXd &x) const = 0;

    protected:
        /**
         * Implements angle normalization, so the result angle is inside [-PI, PI] range.
         * @param angle_rad - angle in radians
         * @return angle in range [-PI, PI]
         */
        double NormalizeAngle(const double angle_rad) const;

        /**
         * Virtual method that should be overriden by filters that have angles in their measurements
         * e.g. radar filter. Sometimes angle values can be more that PI, in this case they should be
         * moved into [-PI, PI] range.
         * @param vector - vector in measurement space which angle values should be normalized inside the method
         */
        void virtual NormalizeMeasurement(Eigen::VectorXd &vector) const;

        /**
         * Converts state vector to measurement space, returns false if conversion is not possible
         * e.g. because of divizion by zero.
         * @param state - state vector
         * @param measurement - output measurement vector, it will instantiated inside the method.
         * @return true if conversion was successfull, false otherwise
         */
        bool virtual ConvertStateToMeasurement(const Eigen::VectorXd &state,
                                               Eigen::VectorXd &measurement) const = 0;

    private:
        /**
         * Generates and returns matrix of (n_aug_ * 2 + 1) augmented sigma points.
         * Sigma points are [x_aug, x_aug + sqrt(lambda * P_aug), x_aug - sqrt(lambda * P_aug)]
         * @return Matrix (n_aug_, 2 * n_aug_ + 1). Each column is an agumented sigma point (n_aug_, 1)
         */
        Eigen::MatrixXd GenerateSigmaPoints() const;

        /**
         * Calculates sigma points after applying process model (function) to every sigma point and wright result into
         * matrix X_sig_pred_ (n_x_, n_sigma)
         * Note that incoming sigma points are augmented, but outcoming sigma points are not augmented.
         *
         * Process model equations:
         *
         * in case yaw_rate != 0:
         *
         *                  v/yaw_rate(sin(yaw - yaw_rate * dt) - sin(yaw))         0.5 dt**2 * cos(yaw) * v_rate
         *                  v/yaw_rate(-cos(yaw - ya_rate * dt) + cos(yaw))  +      0.5 dt**2 * sin(yaw) * v_rate
         *x(k+1) = x(k) +                       0                                   dt * yaw_rate_rate
         *                                   yaw_rate * dt                          0.5 dt**2 * yaw_rate_rate
         *                                      0                                   dt * yaw_rate_rate
         *
         * in case yaw_rate == 0:
         *
         *                                     v * cos(yaw) * dt                0.5 * dt**2 * cos(yaw) * v_rate
         *                                     v * sin(yaw) * dt                0.5 * dt**2 * sin(yaw) * v_rate
         *x(k+1) = x(k) +                      0                        +       dt * v_rate
         *                                     yaw_rate * dt                    0.5 * dt**2 * yaw_rate_rate
         *                                     0                                dt * yaw_rate_rate
         * @param X_sig_aug - matrix (n_aug_, n_sigma) where each column is an augmented sigma point
         * @param delta_time - time from the previous measurement in seconds
         */
        void PredictSigmaPoints(const Eigen::MatrixXd &X_sig_aug, const double delta_time);


        /**
         * Updates state vector and covariance matrix based on transformed sigma points.
         * Calculation is done using weights_ instead of simple mean.
         */
        void CalcSigmaPointsMeanCovariance();


        /**
         * Transforms predicted sigma points into measurement space.
         * Conversion may not always be possible so (e.g. when devision by zero occurs),
         * because of that false can be returned meaning that the result is invalid.
         * @param n_z - length of measurement vector
         * @param Z_sig - reference to a matrix that will contain sigma points in measurement space on output
         * @param z_pred - reference to a vector that will contain predicted state vector in measurement space on output
         * @param S - reference ot a matrix that will contain predicted covariances in measurement space on output
         * @return true if conversion was successfull, false otherwise
         */

        bool ConvertPredictedStateToMeasurementSpace(int n_z, Eigen::MatrixXd &Z_sig,
                                                     Eigen::VectorXd &z, Eigen::MatrixXd &S) const;


        /**
         * Updates state vector and covariance matrix.
         * @param n_z - length of measurement vector
         * @param measurement - current measurement vector
         * @param Z_sig - matrix of sigma points trasformed to measurement space (n_z, n_sigma_)
         * @param z_pred - predicted state vector in measurement space
         * @param S - predicted covariance matrix in measurement space
         * @return
         */

        double UpdateState(int n_z, const Eigen::VectorXd &measurement, const Eigen::MatrixXd &Z_sig,
                           const Eigen::VectorXd &z_pred, const Eigen::MatrixXd &S);

};

#endif //UNSCENTEDKF_UNSCENTED_KALMAN_FILTER_H
