#include <iostream>
#include <fstream>

#include "storage.h"

using namespace std;
using namespace Eigen;

/**
 * Constructor. Initializes input/output files.
 * In case any file cannot be accessed then application stops.
 * @param measurements_file_name - file where measurements come from
 * @param report_file_name - file name where report will be written to
 */
Storage::Storage(const string &measurements_file_name, const string &report_file_name):
                measurements_file_(measurements_file_name.c_str(), ifstream::in),
                report_file_(report_file_name.c_str(), ofstream::out)
{
    CheckFiles(measurements_file_name, report_file_name);
}

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
bool Storage::GetNextMeasurement(MeasurementPackage &package)
{
    string line;

    //Read next line from the measurements file
    if(getline(measurements_file_, line))
    {
        //create package
        package = MeasurementPackage();

        istringstream iss(line);
        string sensor_type;
        long long timestamp;

        // reads first element from the current line
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0)
        {
            // LASER MEASUREMENT

            // read measurements at this timestamp
            package.sensor_type_ = MeasurementPackage::LASER;
            package.raw_measurements_ = VectorXd(2);
            float x, y;
            iss >> x;
            iss >> y;
            package.raw_measurements_ << x, y;
            iss >> timestamp;
            package.timestamp_ = timestamp;
        }
        else if (sensor_type.compare("R") == 0)
        {
            // RADAR MEASUREMENT

            // read measurements at this timestamp
            package.sensor_type_ = MeasurementPackage::RADAR;
            package.raw_measurements_ = VectorXd(3);
            float ro, phi, ro_dot;
            iss >> ro;
            iss >> phi;
            iss >> ro_dot;
            package.raw_measurements_ << ro, phi, ro_dot;
            iss >> timestamp;
            package.timestamp_ = timestamp;
        }
        else
        {
            cerr << "Uknown sensor type: " << sensor_type << endl;
            exit(EXIT_FAILURE);
        }

        // read ground truth data to compare later
        float x_gt, y_gt, vx_gt, vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        package.ground_truth_ = VectorXd(4);
        package.ground_truth_ << x_gt, y_gt, vx_gt, vy_gt;

        return true;
    }
    else
    {
        //if line could not be read return false
        return false;
    }
}

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
void Storage::Report(const VectorXd &estimation, const VectorXd &measurement, const VectorXd &ground_truth,
                     const double nis, const MeasurementPackage::SensorType sensor_type)
{
    report_file_<<estimation(0)<<"\t";
    report_file_<<estimation(1)<<"\t";
    report_file_<<estimation(2)<<"\t";
    report_file_<<estimation(3)<<"\t";

    report_file_<<measurement(0)<<"\t";
    report_file_<<measurement(1)<<"\t";

    report_file_<<ground_truth(0)<<"\t";
    report_file_<<ground_truth(1)<<"\t";
    report_file_<<ground_truth(2)<<"\t";
    report_file_<<ground_truth(3)<<"\t";

    report_file_<<nis<<"\t";
    report_file_<<sensor_type<<"\n";
}

 /**
 * Checks that input/output files are open and can be used. If the check fails then application is stopped
 * with error.
 * @param measurements_file_name
 * @param report_file_name
 */
void Storage::CheckFiles(const string& measurements_file_name, const string& report_file_name) const
{
    if (!measurements_file_.is_open())
    {
        cerr << "Cannot open input file: " << measurements_file_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!report_file_.is_open())
    {
        cerr << "Cannot open output file: " << report_file_name << endl;
        exit(EXIT_FAILURE);
    }
}