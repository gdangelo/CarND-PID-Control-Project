# CarND-PID-Control-Project

> PID Controller for Self-Driving Car ND

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

![PID Controller](https://user-images.githubusercontent.com/4352286/37220909-147b8132-2396-11e8-9e0b-544d357cdb75.png)

## Project Basics

The goal of this project is to implement in C++ a PID controller to control the steering angle for driving a car around a virtual track using the [Udacity simulator](https://github.com/udacity/self-driving-car-sim/releases), as well as tuning each PID gain in order to calculate a steering angle that keeps the car on the track.

The simulator provides cross-track error (CTE) via websocket. The PID (proportional-integral-differential) controller give steering commands to drive the car reliably around the simulator track.

**Project Steps**
- Implement a PID Controller for steering the car
- Optimize each PID gain to run the car smoothly around the track

---

## Discussion / Reflection

### Components of a PID controller

Using a PID controller is a great place to start because it is easy to understand, simple to implement, and efficient in a wide area of applications. However, making the controller to perform well in order to steer a car is a tough challenge. Hence, it is very important to understand how a PID works to tune it accordingly.

The PID controller is a feedback controller, which means that it senses/measures the system state and compared it to the desired goal. The difference between the two is the error or, in other words, the delta between where the car is and where we want it to be. In our case, the measured state is the car's position while the desired goal is the middle of the lane. The error is called the cross-track error (CTE). So, the goal of the PID controller is to convert the CTE into a steering command.

To reach its goal, the PID controller uses three components:

- The "P" for proportional means that the PID produces a command proportional to the CTE. In other words, the car steers in proportion to the CTE. This makes sense because the PID is able to steer the car in the correct direction. If the car is on the right of the middle, then it would steer to the left to get closer to the middle. If the car is far from the middle, the PID would produce a higher steering angle. Higher is the CTE, stronger will be the steering command.

- The "I" for integral sums up all CTE. It is used to remove constant CTE. No matter how small the constant CTE is, the summation of that contant CTE will be significant enough to adjust the controller output. In our case, it prevents the car from driving on one side of the lane the whole time by steering the car toward the middle. 

- The "D" for derivate is the change in CTE between the current measurement and the previous one. This means that, if the car is moving in the wrong direction (away from the middle) this will cause the steering angle to get larger (the "D" term is added to the "P" term), but if the car is getting closer to the middle the steering angle will get smoothed out (the "D" term is substracted to the "P" term) leading to a better driving experience. Besides, inside a curve, as the derivative is quickly changing it helps the car correct its steering angle faster.

These three components are summed together to produce the PID controller output. On top of that, a different gain parameter is associated with each component which are called Kp, Ki, and Kd. 

`steering_angle = - (Kp * P_term + Ki * I_term + Kd * D_term)`.

These gains can be tuned to run the car smoothly around the track.

### Finding the right gains

The optimization algorithm called Twiddle has been used to automatically fine tune the PID gains. The car has been run for about 50 iterations using Twiddle, with a constant throttle value (0.3), to finally obtain the corresponding results: **Kp: 0.30351, Ki: 0.00001, Kd: 2.66123**.

I've also done some tests to control throttle using the steering angle value as follow `throttle = (1 - abs(steering_angle)) * 0.5 + 0.3`. Gains (Kp, Ki, and Kd) obtained from running Twiddle again with the throttle didn't converge to a satisfying result. Furthermore, adding two more parameters (Ka, and Kb) to Twiddle to tune, such as `throttle = (1 - abs(steering_angle)) * Ka + Kb`, takes too long to run before having good results.

---

## Overview
Starting to work on this project consists of the following steps:

1. Install uWebSocketIO and all the required [dependencies](#installation-and-dependencies)
2. Clone this repository
3. Build the main program 
    - `mkdir build`
    - `cd build`
    - `cmake ..`
    - `make`
4. Launch `./pid [use_twiddle] [Kp] [Ki] [Kd]`:
    - *use_twiddle* could be set to -1 to do not use Twiddle, or to any double value to set the max distance (~2000 for one lap)
    - *Kp*, *Ki*, and *Kd* could take any double values
5. Launch the Udacity Term 2 simulator
6. Enjoy!

---

## Installation and Dependencies

This project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

### Other Important Dependencies
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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3). 
  
Once all the dependencies have been installed **clone** the project:

```sh
git clone https://github.com/gdangelo/CarND-PID-Control-Project/
```

and follow the steps 3 to 6 of the [Overview section](#overview) in order to build and run the main program.
  
---

## Questions or Feedback

> Contact me anytime for anything about my projects or machine learning in general. I'd be happy to help you :wink:

* Twitter: [@gdangel0](https://twitter.com/gdangel0)
* Linkedin: [Grégory D'Angelo](https://www.linkedin.com/in/gregorydangelo)
* Email: [gregory@gdangelo.fr](mailto:gregory@gdangelo.fr)
