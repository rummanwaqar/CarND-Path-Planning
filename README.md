# Autonomous Vehicle Path Planner

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="imgs/output.gif" width="480" alt="Output" />

## Overview
This project implements a path planning algorithms to drive a car on a highway. 
The simulator sends car telemetry information (car's position and velocity) and sensor fusion 
information about the rest of the cars in the highway (Ex. car id, velocity, position). 
It expects a set of points spaced in time at 0.02 seconds representing the car's trajectory. 

## Dependencies
* [Simulator Term 3](https://github.com/udacity/self-driving-car-sim/releases)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* cmake
* gcc/g++
* make
* openssl
* libuv
* zlib

## Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd $_`
3. Compile: `cmake .. && make`

## Run
* To run `./path_planner

---

## Details
Trajectory generation is handled by generating a few points for the vehicle in frenet coordinates. 
These points are then converted to cartesian coordinated, converted to ego vehicle frame 
and smoothed out by generating a spline fitting these points. Denser smooth points are recovered 
from the spline and converted back to world frame. We start each path with a few points from the 
previous cycle to ensure smooth transition. 

At each time step we check for car's ahead of us, and we slow down or change lane if possible if collision 
is imminent. Before changing lanes, we check the lane by predicting future positions of all cars in that lane.