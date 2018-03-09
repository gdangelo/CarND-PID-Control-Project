# CarND-PID-Control-Project

> PID Controller for Self-Driving Car ND

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

![PID Controller](https://user-images.githubusercontent.com/4352286/37220909-147b8132-2396-11e8-9e0b-544d357cdb75.png)

The goal of this project is to use a PID controller to control the steering angle for driving a car in a game simulator. The simulator provides cross-track error (CTE) via websocket. The PID (proportional-integral-differential) controllers give steering commands to drive the car reliably around the simulator track.

> **NOTE**: Optimization algorithm called Twiddle has been used to automatically fine tune the P-I-D parameters. Here are the corresponding results: **P: 0.30351, I: 0.00001, D: 2.66123**.

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

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)  
  
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
