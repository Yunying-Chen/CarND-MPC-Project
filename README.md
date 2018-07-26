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
In this project the Kinematic model is used. It recieves the state of the vehicle such as position(x,y), orientation angle(psi), velocity(v), cross-track error(cte) and orientation error(epsi). The outputs of the actuator are acceleration(throttle) and steering angle. With the previous state and timestep, current state of the vehicle is calculated. The equations are as follows:                                                                   
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt                       
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt                                                    
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt                                                                       
v[t+1] = v[t] + a[t] * dt                                                     
cte[t+1] = f(x[t]) - y[t-1] + v[t] * sin(epsi[t]) * dt        
epsi[t+1] = psi[t] - psi_des[t] + (v[t] / Lf  * delta[t] * dt)                

## Timestep Length and Elapsed Duration (N & dt)
The number of points(N) and the time interval(dt) define the prediction horizon. If the prediction horizon is too small, it leads to responsive control and instability. If the prediction horizon is too long, it leads to smooth control and the vehicle will be slow response. After manually trying differnt values of N and dt, such as N=25/dt=0.05, N=20/dt=0.05, N=8/dt=0.1 .etc. When N=10 and dt=0.1, it works the best. 


## Polynomial Fitting and MPC Preprocessing
The wayspoints recieved from the simulator is transformed to vehicle's coordinate system. As a result, the inital position of the vehicle(x,y) and the orientation angle(psi) are zero. The transformation equations are as follows:                      
X_ = (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi)                                 
Y_ = (ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) * cos(-psi)                                                

## Model Predictive Control with Latency
To cope with a 100 millisecond latency, it is one time interval delay. In order to sovle this problem, when defining the values of the actuation at time t, it should take the latency into account and the values of acceleration(throttle) and steering angle should be taken from t-1.


