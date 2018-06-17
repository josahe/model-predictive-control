# Model Predictive Control
> A model for predicting and controlling vehicle actuation

## INTRODUCTION
The goal of this project is to implement [Model Predictive Control](https://en.wikipedia.org/wiki/Model_predictive_control) to drive a car around a track in simulation. MPC is a more complex, advanced method for controlling car actuation, such as steering and acceleration, compared to using a PID controller.

The aim is to follow a trajectory of waypoints by modeling and predicting actuation commands that reduce the cost of finding the optimal trajectory. The MPC uses a [kinematic vehicle model](www.me.berkley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf) to predict future vehicle state and actuation commands.

More details can be found in the project writeup.

This project was undertaken as part of the [Udacity Self-Driving Car NanoDegree](https://eu.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).

## RELEVANT LINKS
#### Project writeup
* [writeup.md](writeup.md)

#### Original project repo
* https://github.com/udacity/CarND-Controls-MPC

## RELEVANT FILES
* [main.cpp](./src/main.cpp)
* [MPC.cpp](./src/MPC.cpp)
