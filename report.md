# **Traffic Sign Recognition** 

<!-- ## Writeup

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

--- -->

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Build a sensor fusion algorithm (Extended Kalman Filter) for laser and radar sensors, in C++
* Test the algorithm performance in a simulation


[//]: # (Image References)

[image1]: ./Docs/report_files/ekf_flowchart.jpg "EKF Process Flowchart"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/748/view) individually and describe how I addressed each point in my implementation.  

---
### Compiling

#### 1. Your code should compile.

All code is compiled using `cmake` and `make`. Project code is available at [project code](https://github.com/rezarajan/sdc-extended-kalman-filter).

A note on code structure: after compiling, the executable may be run as `./ExtendedKF`. Additional flags may be specified to turn off specific sensor readings:

* `--no-radar` : disables radar measurements
* `--no-laser` : disables laser measurements

Note that only one flag may be specified at execution.

_Example:_ `./ExtendedKF --no-radar` disables radar measurements


---
### Accuracy

#### 2. px, py, vx, vy output coordinates must have an RMSE <= [.11, .11, 0.52, 0.52] when using the file: "obj_pose-laser-radar-synthetic-input.txt" which is the same data file the simulator uses for Dataset 1.

Based on Dataset 1 performance with both the laser and radar measurements, the performance of the EKF is as follows:

|State Variable| RMSE       |
|:------------:|:----------:|
|px            |0.0975253   | 
|py            |0.0853578   | 
|vx            |0.418453    | 
|vy            |0.479022    | 

**Additional Tests**

Additional tests are performed where either the laser or radar measurements are excluded, as a comparative measure to the performance when all sensor measurements are accounted for.

_No Radar_

|State Variable| RMSE       |
|:------------:|:----------:|
|px            |0.1886023   | 
|py            |0.1541028   | 
|vx            |0.739615    | 
|vy            |0.47365     | 
 

_No Laser_

|State Variable| RMSE       |
|:------------:|:----------:|
|px            |0.235262    | 
|py            |0.336079    | 
|vx            |0.625627    | 
|vy            |0.692373    |


It is clear that the performance of the filter improved significantly with the inclusion of both sensor measurements. Though the errors with a single sensor are not terribly imprecise, the room for improvement by inclusion of all measurements is considerable. Note that the RMSE values for measurements with only the laser are much lower than with the radar only.

---

### Follows the Correct Algorithm

#### 3. Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.

The algorithm follows the same control flow shown below:

![image1]

_Source: Udacity Self Driving Engineer Nanodegree_

#### 4. Your Kalman Filter algorithm handles the first measurements appropriately.

The algorithm handles the initialization case by directly converting sensor measurements to state values. The first sensor measurement received is the one taken to initialize the state, regardless of sensor type. In the case of laser measurements, the values for x and y coordinates are taken directly from the reading. In the case of radar measurements, a mapping function is used to convert the distance and angle to x and y coordinates.

#### 5. Your Kalman Filter algorithm first predicts then updates.

The algorithm performs the predict step, followed by the update step. These steps are performed sequentially, only after a measurement is received. Furthermore, measurements are handled asynchrously, i.e. **it does not** wait for both laser and radar measurements to be received before the steps are performed.

#### 6. Your Kalman Filter can handle radar and lidar measurements.

The algorithm contains correct mappings for both lidar and radar measurements. In the case of radar measurements which are non-linear, a Jacobian is calculated and the extended kalman filter formulae are used, to linearize measurements. See [above](#3-your-sensor-fusion-algorithm-follows-the-general-processing-flow-as-taught-in-the-preceding-lessons).

---
### Code Efficiency

#### 7. Your algorithm should avoid unnecessary calculations.

In general, the code runs efficiently, and avoids unnecessary calculations or data structures. The code follows an object-oriented structure, and uses only variables that are necessary to executing the kalman filter alorithm. There are perhaps areas for improvement, but in the case of this project the performance is sufficient.

There are only a few additional sections of code which do not directly relate to the extended kalman filter itself: parsing sensor information from the simulator, logging, and additional checks for any flags defined at runtime.

---

### Results

The results of the extended kalman filter are shown [here](https://youtu.be/pOxVQBDDOpQ).