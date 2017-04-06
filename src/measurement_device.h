
#ifndef EXTENDEDKF_MEASUREMENT_DEVICE_H
#define EXTENDEDKF_MEASUREMENT_DEVICE_H

#include "measurement_package.h"

/**
 * Representation of a measurement device.
 * Currently measurements come from file so Storage implements this interface.
 */
class IMeasurementDevice
{
    public:
        virtual bool GetNextMeasurement(MeasurementPackage &measurement) = 0;
};

#endif //EXTENDEDKF_MEASUREMENT_DEVICE_H
