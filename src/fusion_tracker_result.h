#ifndef EXTENDEDKF_FUSION_TRACKER_RESULT_H
#define EXTENDEDKF_FUSION_TRACKER_RESULT_H

#include "Eigen/Dense"

/**
 * Represetns result of measurment process by fusion tracker
 */
class FusionTrackerResult
{
    public:
        //Vector(4) of cartesian estimation after processing
        Eigen::VectorXd estimation_;

        //Vector(4) of cartesian measurements
        Eigen::VectorXd measurement_;

        //NIS value
        double nis_;
};

#endif //EXTENDEDKF_FUSION_TRACKER_RESULT_H
