#ifndef EXTENDEDKF_APP_H
#define EXTENDEDKF_APP_H

#include <iostream>

/*
 * Application main class.
 */
class App
{
    public:
        /**
         * Starts and runs the application. All the application's logic except arguments check originates from
         * this method. Basic arguments check is done in main.cpp.
         *
         * @param input_file_name -  file to read measurements from.
         * @param output_file_name - file to right results to.
         * @param sensor - l for laser measurements only, r for radar measurements only,
         *                 everything else means using both types of measurements
         * @param v_dot_std - standard deviation of velocity change. Non negative value if set by user,
         *                    or negative value otherwise.
         * @param yaw_dot_dot_std - standard deviation of yaw acceleration. Non negative value if set by user,
         *                          or negative value otherwise.
         */
        void Run(const std::string &input_file_name, const std::string &output_file_name,
                 const std::string &sensor, const double v_dot_std, const double yaw_dot_dot_std) const;
};

#endif //EXTENDEDKF_APP_H
