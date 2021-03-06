#include <iostream>

#include "fusion_tracker.h"

using namespace Eigen;

/**
 * Constructor. Initializes values, filters, couples filters with the state.
 * @param v_dot_std - standard deviation of velocity change. Non negative value if set by user,
 *                    or negative value otherwise.
 * @param yaw_dot_dot_std - standard deviation of yaw acceleration. Non negative value if set by user,
 *                          or negative value otherwise.
*/
FusionTracker::FusionTracker(const double v_dot_std, const double yaw_dot_dot_std) :
                            laser_filter_(common_filter_state_, v_dot_std, yaw_dot_dot_std),
                            radar_filter_(common_filter_state_, v_dot_std, yaw_dot_dot_std)
{
    is_initialized_ = false;
    previous_timestamp_ = 0;
    num_measurements_ = 0;
    max_delta_time_ = 0.05;

    cumulative_square_error_ = VectorXd(4);
    cumulative_square_error_ << 0, 0, 0, 0;
}

/**
 * On first call initializes shared filter state with given measurement.
 * On next calls performs predict, update steps on filters chosen by measurement type.
 * @param measurement
 * @param result
 */
void FusionTracker::ProcessMeasurement(const MeasurementPackage &measurement, FusionTrackerResult &result)
{
    //increment number of measurements,
    //we count every measurement even if update cannot be calculated
    num_measurements_++;

    //choose radar or laser filter
    UnscentedKalmanFilter &filter = ChooseFilter(measurement);

    //Initialize defualt NIS value
    double nis = -1;

    //in case this is the first call, initialize the state with the first measurment and a fixed covariance matrix
    if(!is_initialized_)
    {
        previous_timestamp_ = measurement.timestamp_;

        VectorXd state_vector = VectorXd(5);
        filter.ConvertMeasurementToState(measurement.raw_measurements_, state_vector);

        InitializeState(state_vector);

        is_initialized_ = true;
    }
    else
    {
        //calculate time delta
        double dt = (measurement.timestamp_ - previous_timestamp_) / 1000000.0;

        //check that time in ascending order
        if(dt < 0)
        {
            std::cerr << "Incorrect time order. Delta time: " << dt << std::endl;
            exit(EXIT_FAILURE);
        }

        //memorize current time to calculate time delta next time
        previous_timestamp_ = measurement.timestamp_;

        //UKF may be numerically unstable, in this assignment this happens when data-2 is used with radar data only.
        //UKF diverges in this case.
        // Some ways that I heard of to deal with this is (I'm not sure how good they are theoretically):
        // -decompose large time delta into number of small ones and call predict step each small delta time
        // -when cholesky decomposition failes because of negative eigenvalues, the tracking can be restarted
        // -lambda value is negative, it makes things weird, it is possible to tune lambda so it is positive

        // here I use the easiest way to deal with the issue so data-2-radar-only doesn't fail.
        // if delta time is large it is devided into small max_delta_time_ parts and for each max_delta_time_
        // the prediction step is invoked. Prediction is not linear so Predict(dt + dt) != Predict(dt) + Predict(dt),
        // but it kind of works, may be because dt is small, so the motion is almost linear.
        while(dt >= 0)
        {
            double d = std::min(max_delta_time_, dt);
            //perform predict step
            filter.Predict(d);

            dt -= max_delta_time_;
        }

        //perform update step
        nis = filter.Update(measurement.raw_measurements_);
    }

    //convert measurement to cartesian form so it could be passed to report
    VectorXd cartesian_measurement = VectorXd(4);
    filter.ConvertMeasurementToCartesian(measurement.raw_measurements_, cartesian_measurement);

    //accumulate error
    AddSquareError(measurement.ground_truth_);

    //return estimation and cartesian measurement
    result = FusionTrackerResult();
    result.estimation_ = common_filter_state_.x_;
    result.measurement_ = cartesian_measurement;
    result.nis_ = nis;

    //std::cout<<common_filter_state_.P_<<std::endl<<std::endl;
}

/**
 * Initializes shared filter state with given cartesian state vector, which should be retrived from
 * the first measurement. Also covariance state matrix is initialized with fixed values.
 * @param cartesian_x - Vector=(x, y, vx, vy)
 */
void FusionTracker::InitializeState(const VectorXd &cartesian_x)
{
    // 1 and 1000 are some magic numbers,
    // they reflect the fact that we are kind of confident in x and y because we take them
    // from the 1st measurement, but we may know nothing about speed
    common_filter_state_.P_ = MatrixXd(5, 5);
    common_filter_state_.P_ <<  1, 0,    0,     0,  0,
                                0, 1,    0,     0,  0,
                                0,  0,   1,     0,  0,
                                0,  0,   0,     0.1,  0,
                                0,  0,   0,     0,  0.1;

    common_filter_state_.x_ = cartesian_x;

    //As it was said in the hint for the project, starting zero coordinates may be bad for the filter,
    //so make it small values
    if(fabs(cartesian_x(0)) < 1e-5 && fabs(cartesian_x(1)) < 1e-5)
    {
        common_filter_state_.x_(0) = 0.01;
        common_filter_state_.x_(1) = 0.01;
    }
}

/**
 * Returns eigher radar or laser filter based on measurement type.
 * In case measurment type is uknown the program is stopped.
 * @param measurement - measurement package
 * @return eigher radar_filer_ or laser_filer_
 */
UnscentedKalmanFilter& FusionTracker::ChooseFilter(const MeasurementPackage &measurement)
{
    switch(measurement.sensor_type_)
    {
        case MeasurementPackage::LASER:
        {
            return laser_filter_;
        }
        case MeasurementPackage::RADAR:
        {
            return radar_filter_;
        }
        default:
        {
            std::cerr << "Incorrect sensor type: " << measurement.sensor_type_ << std::endl;
            exit(EXIT_FAILURE);
        }
    }
}

/**
 * Calculates square error between given ground truth and current state.
 * Adds this error to the existing value.
 * @param ground_truth
 */
void FusionTracker::AddSquareError(const VectorXd &ground_truth)
{
    VectorXd cartesian = VectorXd(4);
    double px = common_filter_state_.x_(0);
    double py = common_filter_state_.x_(1);
    double v = common_filter_state_.x_(2);
    double yaw = common_filter_state_.x_(3);

    cartesian << px, py, v * cos(yaw), v * sin(yaw);
    VectorXd square_error = (cartesian - ground_truth).array().pow(2);
    cumulative_square_error_ += square_error;
}

VectorXd FusionTracker::GetRMSE() const
{
    //return cumulative_square_error_;
    return (cumulative_square_error_ / num_measurements_).array().sqrt();
}
