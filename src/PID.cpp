#include "PID.h"
#include <vector>
#include <numeric>
#include <cmath>
#include <iostream>
using std::vector;
using std::abs;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this -> Kp = Kp_;
  this -> Ki = Ki_;
  this -> Kd = Kd_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  best_error = 9999.9;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

}

void PID::Twiddle() {
  vector<double> p = {Kp, Ki, Kd};
  vector<double> dp = {1.0, 1.0, 1.0};
  double sum = std::accumulate(dp.begin(), dp.end(), 0); 
  double error;
  while (sum > 0.2){
    for (unsigned int i = 0; i < p.size(); i++){
      p[i] += dp[i];
      this -> Kp = p[0];
      this -> Ki = p[1];
      this -> Kd = p[2];
      error = TotalError();
      
      if (abs(error) < best_error){
        best_error = abs(error);
        dp[i] *= 1.1;
      } else {
        p[i] -= 2 * dp[i];
        this -> Kp = p[0];
        this -> Ki = p[1];
        this -> Kd = p[2];
        error = TotalError();
        if(abs(error) < best_error){
          best_error = abs(error);
          dp[i] *= 1.1;
        } else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }     
      }
    }
  }
  std::cout << Kp << " " << Ki << " " << Kd << std::endl;
  

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total = -Kp * p_error - Ki * i_error - Kd * d_error;
  
  return total;  // TODO: Add your total error calc here!
}