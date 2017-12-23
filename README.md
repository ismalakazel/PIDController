# CarND Controls PID
*Self-driving Car Nanodegree at Udacity.*

**Goals:**

- Learn about PID Controls
- Implement PID Controls in C++
- Learn C++

**Tools:**
- [CarND-PID-Control-Project](https://github.com/udacity/CarND-PID-Control-Project)
- [Xcode](https://developer.apple.com/support/xcode/)


## Description of the PID coefficients in this implementation

**P**: The proportional coefficient is used to best keep the vehicle in the center line by steering it in relation to the cross track error. When updating the steering angle with `-Kp * p_error`, as seen in line 43 of `PID.cpp`, the vehicle stays in the center line as much as possible until oscilation takes in and overshooting becomes a problem.

**D**: The differential coefficient is used to solve the overshooting problem due to high oscilation, as state above. By updating the `TotalError` function in `PID.ccp` with `-Kp * p_error - Kd * d_error` the vehicle will oscilate less and won't overshoot. This can be understood as a counter steer to avoid overshooting of the proportional error.

**I**: The integral coefficient is a way to account for biases, such as a wheels being misaligned to one side. In this implementation the integral part has no effect since no biases are present in the simulator. The integral is included in the `TotalError` function in `PID.ccp` as `-Kp * p_error - Kd * d_error - Ki * i_error`


## Choice of hyperparameters

**P**: 0.2
#
**I**: 0.0
#
**D**: 3.0
#

These were manually tuned according to trial and error. I first started making sure the car started a straight drive by setting **P** to 0.2. Then to avoid overshooting I set the **D** parameter to 3.0. The **I** parameter is set to 0.0 because we are not accounting for any biases. With these values can drive the whole lane best keeping itself in the center line. 
