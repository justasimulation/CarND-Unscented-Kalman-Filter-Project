#include "app.h"
#include "storage.h"
#include "fusion_tracker.h"
#include <chrono>

using namespace std;

/**
 * Runs the application cycle.
 * Reads measurements, passes them to tracker and reports results.
 *
 * @param input_file_name - file to read measurements from.
 * @param output_file_name - file to write results to.
 * @param sensor - l for laser measurements only, r for radar measurements only, everything else means using
 *                 both measurements types.
 * @param v_dot_std - standard deviation of velocity change. Non negative value if set by user,
 *                    or negative value otherwise.
 * @param yaw_dot_dot_std - standard deviation of yaw acceleration. Non negative value if set by user,
 *                          or negative value otherwise.
*/
void App::Run(const string &input_file_name, const string &output_file_name,
              const string &sensor, const double v_dot_std, const double yaw_dot_dot_std) const
{
    //cout<<"Application started."<<endl;
    //auto start_time = std::chrono::system_clock::now();

    //Initialize storage with file names
    Storage storage(input_file_name, output_file_name);

    //Currently measurements come from storage so measurement device refers to storage.
    IMeasurementDevice &measurement_device = storage;

    //Initialize tracker
    FusionTracker fusion_tracker = FusionTracker(v_dot_std, yaw_dot_dot_std);

    FusionTrackerResult result;
    MeasurementPackage package;

    unsigned int supported_sensors = sensor == "r" ? MeasurementPackage::RADAR :
                                      (sensor == "l" ? MeasurementPackage::LASER :
                                        MeasurementPackage::RADAR | MeasurementPackage::LASER);

    //The main cycle:

    //While there are measurements left
    while(measurement_device.GetNextMeasurement(package))
    {
        if(package.sensor_type_ & supported_sensors)
        {
            //1. process next measurement by tracker
            fusion_tracker.ProcessMeasurement(package, result);

            //2. report results
            storage.Report(result.estimation_, result.measurement_, package.ground_truth_,
                           result.nis_, package.sensor_type_);
        }
    }

    //chrono::duration<double> proccessing_time = (chrono::system_clock::now() - start_time);
    //cout << "Processing done in " << proccessing_time.count() << " seconds"<<endl;

    cout<<"RMSE"<<endl<<fusion_tracker.GetRMSE()<<endl;
}