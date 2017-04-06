#ifndef EXTENDEDKF_STORAGE_H
#define EXTENDEDKF_STORAGE_H

#include <fstream>

#include "measurement_device.h"
#include "measurement_package.h"

/**
 * Incapsulates access to a storage. Currently there is only one type of storage - file storage.
 */
class Storage : public IMeasurementDevice
{
    private:
        //file containing measurements
        std::ifstream measurements_file_;

        //report file where measurements and estimation will be written to
        std::ofstream report_file_;

    public:
        /**
         * Constructor. Initializes input/output files.
         * In case any file cannot be accessed then application stops.
         * @param measurements_file_name - file where measurements come from
         * @param report_file_name - file name where report will be written to
         */
        Storage(const std::string &measurements_file_name, const std::string &report_file_name);

        /**
         * In case there are measurements left in the measurements file, this method reads and returns
         * the next measurement.
         *
         * Supported file format (meas - measurement, gt - ground trurth, v - velocity):
         * L meas_x meas_y timestamp gt_px gt_py gt_vx gt_vy
         * R meas_ro meas_phi meas_ro_dot timestamp gt_px gt_py gt_py
         *
         * @param measurement - reference to a measurement class, in case the next measurement was successfully
         *                      read the new MeasurementPackage class is assigned to the reference.
         * @return true if the new measurement was read, false if there are no more measurements.
         */
        bool GetNextMeasurement(MeasurementPackage &measurement);

        /**
         * Reports estimation measurement and ground truth to file, so they could be read by other scripts.
         *
         * Output format (est - estimation, meas - measurement, gt - ground truth, v - velocity):
         * est_x, est_y, est_vx, est_vy, meas_x, meas_y, gt_x, gt_y, gt_vx, gt_vy
         *
         * @param estimation - cartesian vector of estimated values (x, y, vx, vy)
         * @param measurement - cartesian vector of measured values (x, y)
         * @param ground_truth - cartesian vector of ground truth values (x, y, vx, vy)
         * @param nis - NIS value
         * @param sensor_type - type sensor which measurement is being reported
         */
        void Report(const Eigen::VectorXd &estimation,
                    const Eigen::VectorXd &measurement,
                    const Eigen::VectorXd &ground_truth,
                    const double nis,
                    const MeasurementPackage::SensorType sensor_type);

    private:
        /**
         * Checks that input/output files are open and can be used. If the check fails then application is stopped
         * with error.
         *
         * @param measurements_file_name
         * @param report_file_name
         */
        void CheckFiles(const std::string &measurements_file_name, const std::string &report_file_name) const;
};

#endif //EXTENDEDKF_STORAGE_H
