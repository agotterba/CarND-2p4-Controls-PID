#ifndef TWIDDLE_H
#define TWIDDLE_H

class TWIDDLE {
  //indicies for PID calculation
  double kp_;
  double ki_;
  double kd_;
  double current_cte_;
  double prev_cte_;
  double diff_cte_;
  double int_cte_;
  double max_int_;

public:

  
  /*
  * Constructor
  */
  TWIDDLE();

  /*
  * Destructor.
  */
  virtual ~TWIDDLE();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
