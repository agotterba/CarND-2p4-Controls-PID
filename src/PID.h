#ifndef PID_H
#define PID_H

// class PID {
// public:
//   /*
//   * Errors
//   */
//   double p_error;
//   double i_error;
//   double d_error;

//   /*
//   * Coefficients
//   */ 
//   double Kp;
//   double Ki;
//   double Kd;
class PID {
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
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  //reset integral, prev_cte
  void reset();

  //update params, but don't reset integral, prev_cte
  void tweak(double Kp,double Ki, double Kd);
    
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
