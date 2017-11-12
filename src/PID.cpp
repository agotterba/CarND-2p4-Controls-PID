#include "PID.h"
#include <math.h> 

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  kp_ = Kp;
  ki_ = Ki;
  kd_ = Kd;
  current_cte_ = 0.0;
  prev_cte_ = 0.0;
  diff_cte_ = 0.0;
  int_cte_ = 0.0;
  max_int_ = 20.0; //random guess
}

void PID::UpdateError(double cte) {
  prev_cte_ = current_cte_;
  current_cte_ = cte;
  diff_cte_ = current_cte_ - prev_cte_;
  if(prev_cte_ * current_cte_ < 0){ //error has changed from positive to negative
    int_cte_ = 0; //set integral to 0, as suggested in George Gillard paper
  }else if (fabs(int_cte_) < max_int_ || int_cte_ * current_cte_ < 0){
    int_cte_ += current_cte_;
  }
  
}

double PID::TotalError() {
  return -1 * (kp_ * current_cte_ + kd_ * diff_cte_ * ki_ * int_cte_);
}

