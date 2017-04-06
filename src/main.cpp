//CardND Unscented Kalman Filter Project
//
//usage: UnscentedKF measurements_file_name output_file_name
//
//measurements format:
//L x y timestamp gt_x gt_y gt_vx gt_vy
//R ro phi ro_dot timestamp gt_x gt_y, gt_vx, gt_vy
//
//output format:
//est_x est_y est_v est_yaw est_yaw_rate meas_x meas_y gt_x gt_y gt_vx gt_vy NIS sensor_type
//

#include <iostream>

#include "app.h"

using namespace std;

void check_arguments(int argc, char* argv[]);

/**
 * Checks number of arguments and starts application parameterized with the arguments.
 */
int main(int argc, char* argv[])
{
    //check_arguments(argc, argv);

    string input_file_name = "../data/sample-laser-radar-measurement-data-2.txt";//argv[1];
    string output_file_name = "output.txt";//argv[2];

    //string input_file_name = argv[1];
    //string output_file_name = argv[2];

    App app;
    app.Run(input_file_name, output_file_name);

    return 0;
}

/**
 * Checks that there are two arguments. Should be measurements_file_name, output_file_name
 * Stops execution if number of arguments is incorrect.
 */
void check_arguments(int argc, char* argv[])
{
    string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";

    bool has_valid_args = false;

    // make sure the user has provided input and output files

    if (argc == 1)
    {
        cerr << usage_instructions << endl;
    }
    else if (argc == 2)
    {
        cerr << "Please include an output file.\n" << usage_instructions << endl;
    }
    else if (argc == 3)
    {
        has_valid_args = true;
    }
    else if (argc > 3)
    {
        cerr << "Too many arguments.\n" << usage_instructions << endl;
    }

    if (!has_valid_args)
    {
        exit(EXIT_FAILURE);
    }
}

