#ifndef EXTENDEDKF_FUSION_TRACKER_H
#define EXTENDEDKF_FUSION_TRACKER_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include "kalman_filter_state.h"
#include "laser_filter.h"
#include "radar_filter.h"
#include "fusion_tracker_result.h"

/**
 * Represents fusion tracker. Uses dedicated filters for radar and laser measurements to track an object.
 * Both saser and radar filter are unscented Kalman filters.
 * This class contains common KalmanFilterState that is updated by both filters.
 */
class FusionTracker
{
    private:
        //number of measurements processed so far
        unsigned int num_measurements_;

        //accumulates square error so RMSE could be calculated
        Eigen::VectorXd cumulative_square_error_;

        //when first measurement is read, it should be used to iniialize the state,
        //after that this flag should be put to true
        bool is_initialized_;

        //this one is used to calculate deltat time, which is used in state transition and process covariance
        //matrices
        long previous_timestamp_;

        //maximum delta time between measurmenets, if delta time is greater than that predictions
        //step will be done each max_delta_time_ period, after which update step will be done.
        double max_delta_time_;

        //filter state shared by all the filters
        KalmanFilterState common_filter_state_;

        LaserFilter laser_filter_;
        RadarFilter radar_filter_;

    public:
        /**
         * Constructor. Initializes values, filters, couples filters with the state.
         * @param v_dot_std - standard deviation of velocity change. Non negative value if set by user,
         *                    or negative value otherwise.
         * @param yaw_dot_dot_std - standard deviation of yaw acceleration. Non negative value if set by user,
         *                          or negative value otherwise.
         */
        FusionTracker(const double v_dot_std, const double yaw_dot_dot_std);

        /**
         * On first call initializes shared filter state with given measurement.
         * On next calls performs predict, update steps on filters chosen by measurement type.
         * @param measurement
         * @param result
         */
        void ProcessMeasurement(const MeasurementPackage &measurement, FusionTrackerResult &result);

        /**
         * Calculates RMSE for data processed so far.
         * @return Vector(4) of rmse for x, y, vx, vy
         */
        Eigen::VectorXd GetRMSE() const;


    private:
        /**
         * Calculates square error between given ground truth and current state.
         * Adds this error to the existing value.
         * @param ground_truth
         */
        void AddSquareError(const Eigen::VectorXd &ground_truth);

        /**
         * Returns eigher radar or laser filter based on measurement type.
         * In case measurment type is uknown the program is stopped.
         * @param measurement - measurement package
         * @return eigher radar_filer_ or laser_filer_
         */
        UnscentedKalmanFilter& ChooseFilter(const MeasurementPackage &measurement);

        /**
         * Initializes shared filter state with given cartesian state vector, which should be retrived from
         * the first measurement. Also covariance state matrix is initialized with fixed values.
         * @param cartesian_x - Vector=(x, y, vx, vy)
         */
        void InitializeState(const Eigen::VectorXd &cartesian_x);
};

#endif //EXTENDEDKF_FUSION_TRACKER_H
