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
#include <getopt.h>

#include "app.h"

using namespace std;

void check_arguments(int argc, char* argv[]);
void print_usage(char *exec_name);
void parse_options(int argc, char **argv, string &sensor, double &v_dot, double &y_dot);
void exit_with_message(char* exec_name, string message);

/**
 * Checks number of arguments and starts application parameterized with the arguments.
 */
int main(int argc, char* argv[])
{
    check_arguments(argc, argv);

    //string input_file_name = "../data/sample-laser-radar-measurement-data-2.txt";//argv[1];
    //string output_file_name = "output.txt";//argv[2];

    string input_file_name = argv[1];
    string output_file_name = argv[2];

    string sensor = "";
    double v_dot_std = -1;
    double yaw_dot_dot_std = -1;

    parse_options(argc, argv, sensor, v_dot_std, yaw_dot_dot_std);

    App app;
    app.Run(input_file_name, output_file_name, sensor, v_dot_std, yaw_dot_dot_std);

    return 0;
}

/**
 * Checks that there are at least two arguemnts.
 */
void check_arguments(int argc, char* argv[])
{
    // make sure the user has provided input and output files

    if (argc == 1)
    {
        exit_with_message(argv[0], "No arguments found.");
    }
    else if (argc == 2)
    {
        exit_with_message(argv[0], "Please include an output file.");
    }
 }

/**
 * Parse options. [-s l|r] [-v <non negative double>] [y <non negative double>]
 * s - sensor, v - velocity change standard deviation, y - yaw acceleration standard deviation
 */
void parse_options(int argc, char **argv, string &sensor, double &v_dot_std, double &y_dot_std)
{
    int res;
    while((res = getopt(argc, argv, "s:v:y:")) != EOF)
    {
        switch(res)
        {
            case 's':
                sensor = optarg;
                break;
            case 'v':
                try
                {
                    v_dot_std = std::stod(optarg);
                    if(v_dot_std < 0)
                    {
                        exit_with_message(argv[0], "Velocity change standard deviation should be non negative.");
                    }
                }
                catch (invalid_argument ex)
                {
                    exit_with_message(argv[0], "Velocity change standard deviation should be non negative.");
                }
                break;
            case 'y':
                try
                {
                    y_dot_std = std::stod(optarg);
                    if(y_dot_std < 0)
                    {
                        exit_with_message(argv[0], "Yaw acceleration standard deviation should be non negative.");
                    }
                }
                catch (invalid_argument ex)
                {
                    exit_with_message(argv[0], "Yaw acceleration standard deviation should be non negative.");
                }
                break;
            case '?':
                exit_with_message(argv[0], "Failed to parse arguments.");
            default:
                exit_with_message(argv[0], "Failed to parse arguments.");
        }
    }
}

void exit_with_message(char* exec_name, string message)
{
    cerr<<message;
    print_usage(exec_name);
    exit(EXIT_FAILURE);
}

void print_usage(char *exec_name)
{
    string usage_instructions = "Usage instructions: ";
    usage_instructions += exec_name;
    usage_instructions += " path/to/input.txt output.txt [-s l|r] [-v <non negative double>] [y <non negative double>]";

    cerr<<endl<<usage_instructions<<endl;
}

