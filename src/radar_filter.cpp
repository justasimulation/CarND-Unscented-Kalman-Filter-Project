#include "Eigen/Dense"

#include "radar_filter.h"

using namespace Eigen;

/**
 * Constructor. Initializes nose matrix and Jacobian and memorizes given state
 *
 * @param state
 */
RadarFilter::RadarFilter(KalmanFilterState &state) : UnscentedKalmanFilter(state)
{
    R_ = MatrixXd(3, 3);
    R_ << 0.3 * 0.3,     0,      0,
            0,      0.03 * 0.03, 0,
            0,      0,      0.3 * 0.3;
}

/**
 * Converts raw measuremnt vector to Vector(3) cartesian Vector(4).
 * Radar measures only ro, phi and ro_dot so the need to be converted to x, y, vx, vy
 *
 * @param raw_measurement - Vector(3) of ro, phi, ro_dot
 * @param x - Vector(4) of x, y, vx, vy
 */
void RadarFilter::ConvertMeasurementToCartesian(const VectorXd &measurement, VectorXd &x) const
{
    double ro = measurement(0);
    double phi = measurement(1);
    double ro_dot = measurement(2);

    double px = ro * cos(phi);
    double py = ro * sin(phi);
    double vx = ro_dot * cos(phi);
    double vy = ro_dot * sin(phi);

    x << px, py, vx, vy;
}

/**
 * Converts state Vector(5) to measurement Vector(3). Sometimes conversion is not possible,
 * e.g. becuase of devision by zero, in this case false is returned.
 * @param state - Vector(5) with x, y, v, yaw, yaw_rate
 * @param measurement - Initialized Vector(3) on output it will contain ro, phi, ro_dot
 * @return true if conversion was successfull, false otherwise
 */
bool RadarFilter::ConvertStateToMeasurement(const Eigen::VectorXd &state, VectorXd &measurement) const
{
    double p_x = state(0);
    double p_y = state(1);
    double v  = state(2);
    double yaw = state(3);

    if(fabs(p_x*p_x + p_y*p_y) < 0.00001)
    {
        return false;
    }


    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    measurement = VectorXd(3);
    measurement(0) = sqrt(p_x*p_x + p_y*p_y);                        //r
    measurement(1) = atan2(p_y,p_x);                                 //phi
    measurement(2) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot

    return true;
}

/**
 * Converts raw measurment Vector(3) to state vector (x, y, v, yaw, yaw_rate)
 * @param measurement - Vector(3)
 * @param x - Initialized Vector(5), on output it will contain x, y, v, yaw, yaw_rate
 */
void RadarFilter::ConvertMeasurementToState(const VectorXd &measurement, VectorXd &x) const
{
    double ro = measurement(0);
    double phi = measurement(1);
    double ro_dot = measurement(2);

    double px = ro * cos(phi);
    double py = ro * sin(phi);
    double vx = ro_dot * cos(phi);
    double vy = ro_dot * sin(phi);

    x << px, py, ro_dot, phi, 0;
}

/**
 * Normalizes measurements[1] values which is yaw(phi) so it is located in the range [-PI, PI]
 * @param vector
 */
void RadarFilter::NormalizeMeasurement(Eigen::VectorXd &vector) const
{
    vector(1) = NormalizeAngle(vector(1));
}

