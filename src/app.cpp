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
 */
void App::Run(const string &input_file_name, const string &output_file_name) const
{
    //cout<<"Application started."<<endl;
    //auto start_time = std::chrono::system_clock::now();

    //Initialize storage with file names
    Storage storage(input_file_name, output_file_name);

    //Currently measurements come from storage so measurement device refers to storage.
    IMeasurementDevice &measurement_device = storage;

    //Initialize tracker
    FusionTracker fusion_tracker = FusionTracker();

    FusionTrackerResult result;
    MeasurementPackage package;

    //The main cycle:

    //While there are measurements left
    while(measurement_device.GetNextMeasurement(package))
    {
        //1. process next measurement by tracker
        fusion_tracker.ProcessMeasurement(package, result);

        //2. report results
        storage.Report(result.estimation_, result.measurement_, package.ground_truth_,
                       result.nis_, package.sensor_type_);
    }

    //chrono::duration<double> proccessing_time = (chrono::system_clock::now() - start_time);
    //cout << "Processing done in " << proccessing_time.count() << " seconds"<<endl;

    cout<<"RMSE"<<endl<<fusion_tracker.GetRMSE()<<endl;
}