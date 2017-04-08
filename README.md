# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)
[data_1_xy]: ./resources/data_1_xy.png
[data_1_nis_laser]: ./resources/data_1_nis_laser.png
[data_1_nis_radar]: ./resources/data_1_nis_radar.png
[data_2_xy]: ./resources/data_2_xy.png
[data_2_nis_laser]: ./resources/data_2_nis_laser.png
[data_2_nis_radar]: ./resources/data_2_nis_radar.png



## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Build Instructions

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`

## Usage Instructions

`./UnscentedKF path/to/input.txt path/to/output.txt [-s l|r] [-v <non negative double>] [-y <non negative double>]`

* s - which sensor's measurements will be processed. `l` - laser only, `r` - radar only. If not set both sensors' measurements will be processed.
* v - velocity change (longitudinal acceleration) standard deviation. Should be non negative double.
* y - yaw acceleration standard deviation. Should be non negative double.

Example:

`./UnscentedKF path/to/input.txt path/to/output.txt -s l -v 3.0 -y 0.9`

## Brief Code Description

* `main.cpp` - entry point. Parses arguments and starts `App` class.
* `app.h/cpp` - contains `App` class which initializes `Storage`, `MeasurementDevice`, `FusionTracker`, connects them together and runs the main cycle

```
    while(measurement_device.GetNextMeasurement(package))
    {
        ...
        fusion_tracker.ProcessMeasurement(package, result);
        storage.Report(...);
    }
```
* `storage.h/cpp` - contains `Storage` class responsible for reading/writing from/to files.
* `measurement_device.h` - interface `IMeasurementDevice` emulating some sort of device which is implemented by `Storage` becuase measurements come from files at the moment.
* `fusion_tracker.h/cpp` -  contains `FusionTracker` class which stores filters and common filter state. The class updates the state using
appropriate filter and calculates RMSE.
* `kalman_filter_state.h` - contains `KalmanFilterState` class which represents common filters state updated by different filters.
 It contains state vector and covariance matrix
* `unscented_kalman_filter.h/cpp` - class `UnscentedKalmanFilter` is an implementation of the filter with the same name. Almost everything needed for the filter is implemented here,
 except measurement noise initialization, measurement vector normalization and vectors conversion which are implemented in dedicated laser and radar filters.
* `radar_filter.h/cpp` - contains `RadarFilter` which is an extension of `UnscentedKalmanFilter`.
* `laser_filter.h/cpp` - contains `LaserFilter` which is an extension of `UnscentedKalmanFilter`.

## On Instability

Nothing of that was covered in the course, but turns out that UKF easily diverges. 
I faced the issue when
 I tried to use only radar measurements on the second data file. 

I found a paper [Stability of linear and non-linear Kalman filters](https://helda.helsinki.fi/bitstream/handle/10138/144334/gradu_toni_karvonen.pdf?sequence=2),
which covers stability of UKF. As I understand for the error to be bounded at each time some values described in the paper should be bounded at each time step as well.
Probably in our case some value becomes too large and everything blows up. Theoretically correct understanding of what went wrong would require much more time,
so I fixed it without completely understanding what I was fixing.

As I learned searching on the web the following may be the cause of instability: 

* large time delta
* negative eigenvalues of augmented covariance matrix, which causes failure of Cholesky decomposition
* negative lambda value - it may not be an issue, but it looks weird, because it brings negative weight into mean calculation

Time delta was really large compared to regular time deltas, so I fixed it by splitting it into several small deltas and calling prediction each time

```
        while(dt >= 0)
        {
            double d = std::min(max_delta_time_, dt);
            filter.Predict(d);

            dt -= max_delta_time_;
        }

```

It doesn't look like it is theoretically equivalent to a single prediction step, 
but at the same time it doesn't seem like it is theoretically forbidden. 

The fix solved the problem and gave decent results so I didn't try to look into eigenvalues or lambda options.

The conclusion from this is that it is probably necessary to control for some values and make
sure they are in right conditions for the whole system not to diverge. 
But for clear understanding of that a depper research of the topic is required.

## Rubric Results

Results meet the requirement. 
But for me it is no clear how to interpret NIS plots.
Seems that they are quite different for different measurement series and for different sensors.

### First Data File

`./UnscentedKF ../data/sample-laser-radar-data-1.txt output.txt`

or

`./UnscentedKF ../data/sample-laser-radar-data-1.txt output.txt -v 1.0 -y 0.9`

var| RMSE
---|----
px | 0.06 
py | 0.06
vx | 0.53
vx | 0.55

![alt_text][data_1_xy]

![alt_text][data_1_nis_radar]

![alt_text][data_1_nis_laser]

---

### Second Data File

`./UnscentedKF ../data/sample-laser-radar-data-1.txt output.txt`

or

`./UnscentedKF ../data/sample-laser-radar-data-2.txt output.txt -v 1.0 -y 0.9`

var| RMSE
---|----
px | 0.16 
py | 0.18
vx | 0.23
vx | 0.26

![alt_text][data_2_xy]

![alt_text][data_2_nis_radar]

![alt_text][data_2_nis_laser]

## Single Sensor Results

I optimized longitudinal and yaw acceleration standard deviations for the rubric criteria,
but for single sensor the results are considerably worse. By tuning longitudinal acceleration
 the results below can be shown. Radar measurmenets of the second dataset do not diverge anymore,
   but show the worst result, deeper investiogation is needed to understand why.

### First Data File Both Sensors

`./UnscentedKF ../data/sample-laser-radar-data-1.txt output.txt -v 3.0 -y 0.9`

var| RMSE
---|----
px | 0.06 
py | 0.07
vx | 0.52
vx | 0.56


### First Data File Laser Only

`./UnscentedKF ../data/sample-laser-radar-data-1.txt output.txt -s l -v 3.0 -y 0.9`

var| RMSE
---|----
px | 0.08 
py | 0.07
vx | 0.63
vx | 0.60

### First Data File Radar Only

`./UnscentedKF ../data/sample-laser-radar-data-1.txt output.txt -s r -v 3.0 -y 0.9`

var| RMSE
---|----
px | 0.11
py | 0.12
vx | 0.59
vx | 0.63

### Second Data File Both Sensors

`./UnscentedKF ../data/sample-laser-radar-data-2.txt output.txt -v 3.0 -y 0.9`

var| RMSE
---|----
px | 0.18
py | 0.18
vx | 0.29
vx | 0.27

### Second Data File Laser Only

`./UnscentedKF ../data/sample-laser-radar-data-2.txt output.txt -s -l -v 3.0 -y 0.9`

var| RMSE
---|----
px | 0.21
py | 0.19
vx | 0.38
vx | 0.30

### Second Data File Radar Only

`./UnscentedKF ../data/sample-laser-radar-data-2.txt output.txt -s -r -v 3.0 -y 0.9`

var| RMSE
---|----
px | 0.20
py | 0.44
vx | 0.99
vx | 0.47


## Reflections

* This is not as easy as it was presented in the course. There is a divergence issue.
* Not clear how to address the divergence properly.
* Not clear how to interpret different NIS plots. They may be quite different for different series of measurements or for different sensors.
* Overall performance seems better on curved parts of trajectory, but sometimes worse on straight lines.
