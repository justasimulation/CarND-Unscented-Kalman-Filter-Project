#include "unscented_kalman_filter.h"

using namespace Eigen;

///////////////////////////////////////Public methods///////////////////////////////////////////

/**
 * Constructor. Memorizes state object and initializes parameters.
 * @param state
 */
UnscentedKalmanFilter::UnscentedKalmanFilter(KalmanFilterState &state) : state_(state)
{
    // state vector is: x, y, v, yaw, yaw_rate
    n_x_        = 5;

    // augmented state vector is x, y, v, yaw, yaw_rate, v_rate_mean, yaw_rate_rate_mean
    n_aug_      = 7;

    // number of sigma points is a magic number
    n_sigma_    = 2 * n_aug_ + 1;

    // weights are calculated using this nubmer, magic as well
    lambda_     = 3 - n_aug_;

    // initialize weights
    weights_ = VectorXd(n_sigma_);
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    double weight = 0.5 / (n_aug_ + lambda_);
    for(int i = 1; i < n_sigma_; i++)
    {
        weights_(i) = weight;
    }

    // tunable parameters
    std_a_      = 3;
    std_yawdd_  = 0.6;
}

/**
 * Performs prediction step of unscented kalman filter.
 * 1. Generates sigma points
 * 2. Applies process model transformation to them.
 * 3. Calculates mean and covariance of the transformed sigma points
 *
 * Transformed sigma points are saved into X_sig_pred_ so the can be reused by Update step.
 * @param delta_time - time passed from last measurement processing (seconds)
 */
void UnscentedKalmanFilter::Predict(const double delta_time)
{
    //1. generate sigma points
    MatrixXd X_sig_aug = GenerateSigmaPoints();
    //2. apply process model transformation to sigma points and memorize the result in X_sig_pred_
    PredictSigmaPoints(X_sig_aug, delta_time);
    //3. calculate mean and covariance of the transformed sigma points
    CalcSigmaPointsMeanCovariance();
}

/**
 * Implements update step of unscented kalman filter. Returns NIS value.
 * 1. Transformes calculated in prediction step sigma points into measurement space,
 *    calculates mean and covariance of the points in the measurement space.
 * 2. Updates state vector and covariance.
 * 3. Calculates NIS
 * @param measurement - measurement vector
 * @return NIS value or -1 if update was unsuccessfull (e.g. because of possible division by zero)
 */
double UnscentedKalmanFilter::Update(const VectorXd &measurment)
{
    int n_z = measurment.rows();

    VectorXd z_pred;
    MatrixXd S;
    MatrixXd Z_sig;

    //1. Convert predicted sigma points into measurement space
    if(ConvertPredictedStateToMeasurementSpace(n_z, Z_sig, z_pred, S))
    {
        //2. Update state vector and covariance and return NIS
        return UpdateState(n_z, measurment, Z_sig, z_pred, S);
    }
    else // if failed to convert, no update is done, -1 returned as NIS
    {
        return -1;
    }
}

/////////////////////////////////////Protected methods///////////////////////////////////////////////

/**
 * Virtual method that should be overriden by filters that have angles in their measurements
 * e.g. radar filter. Sometimes angle values can be more that PI, in this case they should be
 * moved into [-PI, PI] range.
 * @param vector
 */
void UnscentedKalmanFilter::NormalizeMeasurement(Eigen::VectorXd &vector) const {}

/**
 * Implements angle normalization, so the result angle is inside [-PI, PI] range.
 * @param angle_rad - angle in radians
 * @return angle in range [-PI, PI]
 */
double UnscentedKalmanFilter::NormalizeAngle(const double angle_rad) const
{
    //while (angle_rad> M_PI) angle_rad-=2.*M_PI;
    //while (angle_rad<-M_PI) angle_rad+=2.*M_PI;
    //return angle_rad;

    double angle = angle_rad;
    int times = (int)(angle / (2. * M_PI));
    angle -= times * 2 * M_PI;

    if (angle_rad > M_PI) angle-=2.*M_PI;
    if (angle_rad < -M_PI) angle+=2.*M_PI;

    return angle;
}

////////////////////////////////////////////Private methods////////////////////////////////////////////


/**
 * Generates and returns matrix of (n_aug_ * 2 + 1) augmented sigma points.
 * Sigma points are [x_aug, x_aug + sqrt(lambda * P_aug), x_aug - sqrt(lambda * P_aug)]
 * @return Matrix (n_aug_, 2 * n_aug_ + 1). Each column is an agumented sigma point (n_aug_, 1)
 */
MatrixXd UnscentedKalmanFilter::GenerateSigmaPoints() const
{
    // Create augmented state vector. Use 0 for augmented parameters because we don't know them.
    VectorXd x_aug = VectorXd(n_aug_);
    x_aug.fill(0.);
    x_aug.head(n_x_) = state_.x_;

    // Craete augmented covariance matrix.
    // It is |P_ 0| where Q is |variance(velocity_acceleration),           0                        |
    //       |0  Q|            |              0                   variance(yaw_rate_acceleration)   |
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = state_.P_;
    P_aug(n_x_, n_x_) = std_a_ * std_a_;
    P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    // Find square root of P_aug
    MatrixXd P_aug_sqrt = P_aug.llt().matrixL();

    // Generate sigma points
    MatrixXd X_sig_aug = MatrixXd(n_aug_, n_sigma_);
    X_sig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; i++)
    {
        X_sig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
        X_sig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
    }

    return X_sig_aug;
}

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
void UnscentedKalmanFilter::PredictSigmaPoints(const MatrixXd &X_sig_aug, const double delta_time)
{
    X_sig_pred_ = MatrixXd(n_x_, n_sigma_);

    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        double p_x      = X_sig_aug(0, i);
        double p_y      = X_sig_aug(1, i);
        double v        = X_sig_aug(2, i);
        double yaw      = X_sig_aug(3, i);
        double yawd     = X_sig_aug(4, i);
        double nu_a     = X_sig_aug(5, i);
        double nu_yawdd = X_sig_aug(6, i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001)
        {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_time) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_time));
        }
        else
        {
            px_p = p_x + v * delta_time * cos(yaw);
            py_p = p_y + v * delta_time * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_time;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5 * nu_a * delta_time * delta_time * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_time * delta_time * sin(yaw);
        v_p = v_p + nu_a * delta_time;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_time * delta_time;
        yawd_p = yawd_p + nu_yawdd * delta_time;

        //write predicted sigma point into right column
        X_sig_pred_(0, i) = px_p;
        X_sig_pred_(1, i) = py_p;
        X_sig_pred_(2, i) = v_p;
        X_sig_pred_(3, i) = yaw_p;
        X_sig_pred_(4, i) = yawd_p;
    }
}

/**
 * Updates state vector and covariance matrix based on transformed sigma points.
 * Calculation is done using weights_ instead of simple mean.
 */
void UnscentedKalmanFilter::CalcSigmaPointsMeanCovariance()
{
    //predicted state mean
    state_.x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  //iterate over sigma points
        state_.x_ = state_.x_ + weights_(i) * X_sig_pred_.col(i);
    }

    //predicted state covariance matrix
    state_.P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {  //iterate over sigma points

        // state difference
        VectorXd x_diff = X_sig_pred_.col(i) - state_.x_;
        // angle normalization
        x_diff(3) = NormalizeAngle(x_diff(3));

        state_.P_ = state_.P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

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
bool UnscentedKalmanFilter::ConvertPredictedStateToMeasurementSpace(int n_z, MatrixXd &Z_sig,
                                                                    VectorXd &z_pred, MatrixXd &S) const
{
    //create matrix for sigma points in measurement space
    Z_sig = MatrixXd(n_z, n_sigma_);

    //transform sigma points into measurement space
    for (int i = 0; i < n_sigma_; i++)
    {
        VectorXd mes;

        //try to transform, in case failed the whole transformation is invalid, return false
        if(!ConvertStateToMeasurement(X_sig_pred_.col(i), mes))
        {
            return false;
        }

        Z_sig.col(i) = mes;
    }

    //mean predicted measurement
    z_pred = VectorXd(n_z);
    z_pred.fill(0.0);
    for (int i=0; i < n_sigma_; i++)
    {
        z_pred = z_pred + weights_(i) * Z_sig.col(i);
    }

    //measurement covariance matrix S
    S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < n_sigma_; i++)
    {
        //residual
        VectorXd z_diff = Z_sig.col(i) - z_pred;

        //angle normalization
        NormalizeMeasurement(z_diff);

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    S = S + R_;

    return true;
}

/**
 * Updates state vector and covariance matrix.
 * @param n_z - length of measurement vector
 * @param measurement - current measurement vector
 * @param Z_sig - matrix of sigma points trasformed to measurement space (n_z, n_sigma_)
 * @param z_pred - predicted state vector in measurement space
 * @param S - predicted covariance matrix in measurement space
 * @return
 */
double UnscentedKalmanFilter::UpdateState(int n_z, const VectorXd &measurement, const MatrixXd &Z_sig,
                                          const VectorXd &z_pred, const MatrixXd &S)
{
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sigma_; i++)
    {
        //residual
        VectorXd z_diff = Z_sig.col(i) - z_pred;
        //angle normalization
        NormalizeMeasurement(z_diff);

        // state difference
        VectorXd x_diff = X_sig_pred_.col(i) - state_.x_;
        //angle normalization
        x_diff(3) = NormalizeAngle(x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = measurement - z_pred;

    //angle normalization
    NormalizeMeasurement(z_diff);

    //update state mean and covariance matrix
    state_.x_ = state_.x_ + K * z_diff;
    state_.P_ = state_.P_ - K * S * K.transpose();

    //calculate NIS
    double nis = z_diff.transpose() * S.inverse() * z_diff;

    return nis;
}


