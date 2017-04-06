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
         */
        void Run(const std::string &input_file_name, const std::string &output_file_name) const;
};

#endif //EXTENDEDKF_APP_H
