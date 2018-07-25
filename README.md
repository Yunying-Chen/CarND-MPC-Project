# CarND-MPC-Project

## Intro
This project implements a Model Predictive Controller(MPC) to control a vehicle in a simulator. The telemetry information is provided and the steering angle and throttle need to be calculated to control the vehicle.

## Dependencies
* make >= 4.1(mac, linux), 3.81(Windows)              
* gcc/g++ >= 5.4       
* uWebSockets
* Eigen
* Ipopt and CppAD

## The Model
In this project the Kinematic model is used. It recieves the state of the vehicle such as position(x,y), orientation angle(psi), velocity(v), cross-track error(cte) and orientation error(epsi). The outputs of the actuator are acceleration(throttle)  and steering angle. With the previous state and timestep, current state of the vehicle is calculated. The equations are as follows:                                                                        
\chi_{t+1}= \chi_{t} + \upsilon_{t} * cos(\psi_{t}) * d_{t}
