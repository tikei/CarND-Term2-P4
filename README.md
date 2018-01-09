# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---


## Project Description

Autonomous steering and throttle control using a PID controller written in C++.
This project uses the Udacity Self-Driving car simulator. 

## PID Controller 

# Parameters

The general form of the PID model is:

**PID_output = -Kp x Error - Kd x differential_error - Ki x integral_error**
where:
Kp, Kd and Ki are model parameter values to be derived empirically or through optimisation.


The P, I, D components of the PID algorithm have the following effect:

**P** -> Proportional. Represents the effect of the current error. If the error is large, the PID controller outputs a large value with opposite sign. 

**I** -> Integral. Represents the sum of all previous errors. If the PID output does not sufficiently correct for the errors at each time interval, a large error accumulates and the PID controller outputs a large corrective value.

**D** -> Differential. Takes into account the current rate of change of the error. The lower the rate of change in error, the less correction is needed from the PID controller (lower PID output). Thus the PID controller anticipates based on the current trend in the error how much correction is necessary, avoiding overshooting the target value. 


# Parameter values used:
The values for Kp, Kd and Ki were chosen manually in the following way:

The steering PID controller parameter values were selected using constant throttle.
Initial value of 0.3 was used for Kp. Ki and Kd were set to zero. A Kp value of 0.2 was providing better results and the vehicle was able to make the first turn. 
Then further experimentation proceeded with values for Kd (holding Ki = 0). Once suitable results were obtained by driving around the track, a value for Ki was selected (again through experimenting with different values).

Once the steering PID controller was tuned, the constant throttle was replaced by the throttle PID controller which was tuned manually in the same way as the steering PID.

**Final values selected:** 

**PID controller for steering:**
* Kp value: 0.2
* Kd value: 1.5
* Ki value: 0.003

**PID controller for throttle:**
* Kp value: 0.7
* Kd value: 0.5
* Ki value: 0.0



## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 



## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).




