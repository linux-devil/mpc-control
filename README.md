# MPC - Model Predictive Control

For this project we have implemented a global kinematic model which represents the vehicle state model by position, orientation and velocity.
The main goal was to use the mentioned MPC (Model Predictive Controller) to safely drive a car along the track. The key point in this project was to choose the appropriate parameter values that optimize the MPC cost function.
In addition to the variables mentioned before, others parameters were taken into account to define the MPC pipeline.
* The CTE (cross track error) represents the distance of the vehicle from the trajectory.
* EPSI (orientation error) which describes the difference between the vehicle orientation and the trajectory orientation.
Two actuators were also used to solve the MPC
* delta -> the steering angle
* a -> the acceleration value (can be negative or positive)

## Video link - https://youtu.be/Y49KsWKjB1U

# mpc-control
Implementation of Model Predictive Control using Udacity Self Driving Car simulator

## Compile

Code successfully compiles

## Implementation

# Model

Implemented a global kinematic model which represents the vehicle state model by position, orientation and velocity.
Key was to select appropriate parameter values that optimise the MPC cost function.

Following parameters were taken into account to define the MPC pipeline.
* CTE (cross track error) represents the distance of the vehicle from the trajectory.
* EPSI (orientation error) which describes the difference between the vehicle orientation and the trajectory orientation.
Two actuators were also used to solve the MPC:
* delta -> the steering angle
* a -> the acceleration value (can be negative or positive)

# Kinematic model is implemented using the following equations:
  
      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

(x,y) —  position of the vehicle
psi  — orientation of the vehicle
v — velocity
delta and a — actuators like steering angle and aceleration (throttle/brake combined)
Lf — measures the distance between the front of the vehicle and its center of gravity (the larger the vehicle, the slower the turn rate)
cte — cross-track error (the difference between the line and the current vehicle position y in the coordinate space of the vehicle)
epsi —  orientation error. 

The vehicle model is implemented in the FG_eval class.

##Model Predictive Control with Latency

To accomplish the 100ms delay requirement, it was set a value of 20 for N and 0.2s for dt in order to have a better and greater predictive estimation. Also set a max speed value of 65mph to avoid very high speed situations.

# Dependencies

Dependencies

- cmake >= 3.5
  All OSes: click here for installation instructions
- make >= 4.1
  Linux: make is installed by default on most Linux distros
  Mac: install Xcode command line tools to get make
  Windows: Click here for installation instructions
- gcc/g++ >= 5.4
  Linux: gcc / g++ is installed by default on most Linux distros
  Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  Windows: recommend using MinGW
- uWebSockets == 0.14, but the master branch will probably work just fine
  Follow the instructions in the uWebSockets README to get setup for your platform. You can download the zip of the   appropriate version from the releases page. Here's a link to the v0.14 zip.
  If you have MacOS and have Homebrew installed you can just run the ./install-mac.sh script to install this.
- Ipopt
  Mac: brew install ipopt --with-openblas
- CppAD
  Mac: brew install cppad
  Linux sudo apt-get install cppad or equivalent.

# Basic building Instructions
- Clone this repo.
- Make a build directory: mkdir build && cd build
- Compile: cmake .. && make
- Run it: ./mpc.
