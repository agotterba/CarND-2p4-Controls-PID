#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"

struct pid_setting {
  double kp;
  double kd;
  double ki;
  double dp;
  double dd;
  double di;
};


  

class TWIDDLE {
  //PID CONTROL
  PID pid_steering;
  PID pid_throttle;
  pid_setting steering;
  pid_setting throttle;
  pid_setting init_steering; //remember init values for normalization of dparam_sum

  //TARGET SPEED
  double target_speed;
  double speed_error;

  //INTERNAL STATE
  bool reset;
  int step_count;
  int step_limit;
  int param_ring; //0 for kp, 1 for kd, 2 for ki
  int posneg_ring; //0 for positive, 1 for negative
  bool init_run;
  bool improved_this_epoch;
  double run_cum_error;
  double best_error;
  double max_single_error;
  int epoch;

  //Updating parameters
  double fact_increase;
  double fact_decrease;
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
  * Initialize TWIDDLE.
  */
  void init(double init_steer_kp, double init_steer_kd, double init_steer_ki,
            double init_speed_kp, double init_speed_kd, double init_speed_ki,
            double target_speed);

  /*
  * Update the PIDs, and the twiddle fields
  */
  void update(double cte, double speed);

  /*
  * return the steering angle to use
  */
  double findSteering();

  //return the throttle to use
  double findThrottle();

  //get speed_error
  double get_speed_error();

  //check if simulation should be reset
  bool checkReset();

  void apply_reset();

  //function to update cumulative error; can change function as desired
  void update_run_cum_error(double cte);

  //
  bool eval_posneg(double &p_param, double &d_param, bool improved);
};

#endif /* TWIDDLE_H */
