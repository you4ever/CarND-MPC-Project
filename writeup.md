# Model Predictive Control


---


## Introduction
In the PID control projects, it is observed the control paradigm of responding to previously accumulated and instantaneous errors cannot cope with fast changing road curves especially in higher speed. In this project, a predictive model is included in the control and the actuation values are optimized for predicted errors. In addition, an actuation is applied with a propagation and processing delay. It is known PID controller cannot handle control latency well. But a Model Predictive Controller (MPC) can cope with the latency issue by incorporating the latency into the model. 

A video demo can be found here: https://youtu.be/ZyTTf_IEPEM

## Implementation
The implementation of the MPC consists of the followings steps:

1) Vehicle kinematic model is implemented to predict the vehicle state with control vectors
2) Converting the vehicle state and way points from the simulator to proper coordinates and fit a polynomial to waypoints.
3) Set up the automatic derivative (AD) based solver for finding the best control vectors (steering and throttle) by formulating the task as an optimization problem.
4) Fine tuning the model to find the best result

 
### The Model

#### Converting Measurement to Vehicles's Corrdination
The vehicles state are described by six parameters: 
* ``px``: the x coordinate of vehicle's x position
* ``py``: the y coordinate of vehicles's y position
* ``v`` : the vehicle's speed in m/s
* ``psi``: the vehicle's heading
* `` cte ``: the cross-track error
* ``psie``: heading error

In a given measurement, the vehicle's state``px``, ``py`` and ``psi`` as well as waypoints are given in the global coordinate. For ease of computation, we convert the coordinate system to center around the vehicle's position and align the x-axis with heading at the time of measurement. Hence after converting the coordinate system, we have ``px = 0``, ``py = 0`` and ``psi = 0``. The following equations are used to calculate the new coordinate after the conversion:
```$xslt
   x_new = (x - px) * cos(-psi） - (y - py) * sin(-psi);
   y_new = (x - px) * sin(-psi） + (y - py) * cos(-psi);
```
The actual conversion is performed in main.app: Line 107-17. In addition, the speed is measured in mph, in order to align with the vehicle kinematic model the speed is converted to m/s by ``v *= 0.44704``.

#### Kinematic Model and Constraints
The vehicle model are described by a set of discrete time differentiate equations:
```
   x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
   y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
   psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
   v_[t] = v[t-1] + a[t-1] * dt
   cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
   epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```
This model is converted to constraints of the optimization problem in order to enabling the predictive model (c.f. MPC.cpp: Line 106-11). By setting ``delta`` and ``a`` to zeros, a degenerated model with zero control vectors is shown in below:
```
   x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
   y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
   psi_[t] = psi[t-1] 
   v_[t] = v[t-1]
   cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
   epsi[t] = psi[t] - psides[t-1]
```
This zero-input model is used to predict the vehicle state to adapt the control latency (c.f. main.cpp: Line 131-41).

### Timestep Length and Elapsed Duration (N and dt)

The choice of N and dt depends mainly on two factors: control latency and predictive horizon. 

The predictive horizon given by N*dt is chosen to achieve the balance between predictive model accuracy and environment complexity. Heuristically, in order to cope with complex road condition with many turns, the control vectors should be chosen with longer predictive horizon. On the other hand, too long predictive horizon will give rise to wrong actuation due to the the inaccuracy of vehicle's model used in MPC. In my experiment, the predictive horizon is chosen to 3 seconds. 

Given predictive horizon ``N*dt = 3 sec``, the next parameter to choose is ``dt``. A common misconception is the smaller ``dt`` is the better. In realistic scenarios, due the the control latency the too small ``dt`` will give rise to wrong control input. Due to the "predictive" nature, the MPC will be misled by too small ``dt`` because it wrongly assumes the controller can apply the future controls faster than it actually can. So the `dt` shall be chosen so that it's slightly larger than the control latency. After some experiments, `dt = 0.20 sec` is chosen. 

Summarize the above, in ``MPC.cpp`` we have:
```$xslt
size_t N = 15;
double dt = 0.20;
```

### Polynomial Fitting and MPC Preprocessing
    
As discussed before, the coordinate system is converted to be centered around the vehicles position and algined with vehicles's heading at the time of measurement, namely, vehicle's perspective. So the coordinates of the waypoints are first converted to vehicle's perspective. The equations used in given in above. 

Next, the waypoints are fitted to a degree-3 polynomial. Experiments indicate degree-2 polynomial is sufficient for the lake track used in Term 2's simulator. The complexity of lake track is relatively low comparing to scenes we've seen in Term 1's second track and other driving conditions we've encountered in Term 1 lane finding projects. As a result, degree-2 polynomial is sufficient we because in this track we do not expect to have "double turns" within 6 waypoints ahead. Degree-3 polynomial will be able to handle such "double turns" and works well in our around lake track -- hence I make such choice for some extent of future proof. The waypoints after converting to vehicles perspective is fitted to the degree-3 polynomial in ``main.cpp : Line 118``.  

Before using the measured vehicle's state to solving the optimization problem, it's updated to reflect the control latency. The equation used for updating the vehicle's state is given in above with zero control vectors and ``dt = latency = 0.1 sec``. The updated vehicle's state reflects the predicted the vehicle state at the time of control vectors will be applied. The cross-track error (``cte``) and heading error (``epsi``) is also updated with new vehicles position (c.f. ``main.cpp: Line 133-142``). 

### Model Predictive Control with Latency

The MPC is implemented in ``MPC.cpp``. Thanks to the pre-processing step of updating the vehicle's state before solving the optimization problem, the MPC can be designed transparent to latency model except for the ``dt`` parameter tuning as mentioned above. The predictive model is incorporated into the problem formulation as constraints and is implemented in ``MPC.cpp: Line 70 - 112``. The constraints' upper bounds and lower bounds are set to zeros enforce the equality in the model (see ``MPC.cpp: Line 184-189``) 

Finally, the model is complete with setting up the cost function. The cost function consists of three components:

First and foremost is the cost of CTE and heading error. Deviation from center and heading given by the tangent line of waypoints are penalized. 

Second part is the cost on magnitude of control vectors. Intuitively, large control vectors shall be avoided unless necessary to maintain the vehicle in safe areas. For example, unnecessary sudden brake is dangerous to all users of the road.

Lastly, the delta between consecutive controls should not be too large for the sake of both safety and comfort of passengers. 

Detailed cost functions are given in ``MPC.cpp: Line 53-68``. The weights for each penalty needs some manual twiddling. However, experiments shows the MPC has a large tolerance range of the different weight choices as long as the first two penalties on CTE and heading error are kept the largest so that car is always kept in safety. In my experiments, I first fixed the first weights to 8000 and 1000 respectively and further tuned the penalty for large delta in order to avoid oscillating behavior. Finally the weights on throttle is set to 100 to avoid unnecessary sudden acceleration and braking. 

## Simualtion result

After tuning the weights of cost functions, the MPC is able to drive the car safely and smoothly by tracking the waypoints. The reference speed is set to 90 mph and highest speed observed in the simulation is around 77mph. The model is able to safely achieve top speed up to 101 mph by setting a high very high reference speed (e.g. 100 m/s). 
