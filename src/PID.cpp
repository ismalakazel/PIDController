#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    // CTE at delta T-1
    this->previous_cte = 0;
    
    // Proportional controller
    this->p_error = 0;
    
    // Integral controller
    this->i_error = 0;
    
    // Differential controller
    this->d_error = 0;
    
    // Coefficients
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::UpdateError(double cte) {
    
    // Steer in proportional to the cross track error
    p_error = cte;
    
    // Update sum of all cross track errors to compensate for biases
    i_error += cte;
    
    // Counter steer to avoid overshooting of the proportional error
    d_error = cte - previous_cte;
    
    // Track delta T cross track error
    previous_cte = cte;
}

double PID::TotalError() {
    return -Kp * p_error - Kd * d_error - Ki * i_error;
}

