#include "TWIDDLE.h"
#include "PID.h"
#include <math.h> 
#include <iostream>

using namespace std;

/*
* TODO: Complete the TWIDDLE class.
*/

TWIDDLE::TWIDDLE() {}

TWIDDLE::~TWIDDLE() {}

void TWIDDLE::init(double init_steer_kp, double init_steer_kd, double init_steer_ki,
                   double init_steer_cp, double init_steer_cd, double init_steer_ci,
                   double init_speed_kp, double init_speed_kd, double init_speed_ki,
                   double init_target_speed){
  //PID CONTROLERS
  steering.kp = init_steer_kp;
  steering.kd = init_steer_kd;
  steering.ki = init_steer_ki; 
  st33ring.kp = init_steer_cp;
  st33ring.kd = init_steer_cd;
  st33ring.ki = init_steer_ci; 
  //steering.dp = steering.kp / 100.0; //rely on 'no improvements this epoch' to determine when to raise speed
  //steering.dd = steering.kd / 100.0;
  //steering.di = steering.ki / 100.0;
  steering.dp = 3e-4; 
  steering.dd = 1.3e-9;
  steering.di = 6.5e-9;
  st33ring.dp = 8.0e-9; 
  st33ring.dd = 5.0e-12;
  st33ring.di = 9e-10;
  init_steering = steering;
  init_st33ring = st33ring;
  throttle.kp = init_speed_kp;
  throttle.kd = init_speed_kd;
  throttle.ki = init_speed_ki;
  throttle.dp = 1e-2; 
  throttle.dd = 5e-6;
  throttle.di = 1e-2;
  pid_steering.Init("tSteering",steering.kp,steering.kd,steering.ki);
  pid_st33ring.Init("tSt33ring",st33ring.kp,st33ring.kd,st33ring.ki);
  pid_throttle.Init("tThrottle",throttle.kp,throttle.kd,throttle.ki);

  //TARGET SPEED
  target_speed = init_target_speed;
  speed_error = 0;

  //INTERNAL STATE
  reset = false;
  step_count = 0;
  step_limit = 3260; //one lap at 15mph
  param_ring = 0; //0 kp, 1 for kd, 2 for ki, 3 for cp, 4 for cd, 5 for ci
  posneg_ring = 0; //0 for positive, 1 for negative
  chew_run = true;
  init_run = true;
  improved_this_epoch = false;
  run_cum_error = 0.0;
  best_error = 1e6; // if the error really was this high, the car would be off the road
  max_single_error = 0.0;
  fact_increase = 1.5;
  fact_decrease = 0.5;
  epoch = 0;
}

void TWIDDLE::update(double cte, double speed) {
  update_run_cum_error(cte);
  pid_steering.UpdateError(cte);
  pid_st33ring.UpdateError(cte*cte*cte);
  speed_error = speed - target_speed;
  pid_throttle.UpdateError(speed_error);
  if (cte > max_single_error){
    max_single_error = cte;
  }
  step_count += 1;
  if (step_count >= step_limit){
    reset = true;
  }
}

double TWIDDLE::findSteering() {
  return pid_steering.TotalError() + pid_st33ring.TotalError();
}

double TWIDDLE::findThrottle() {
  return pid_throttle.TotalError();
}

double TWIDDLE::get_speed_error() {
  return speed_error;
}
bool TWIDDLE::checkReset() {
  if(reset){
    //cout <<"checkReset found true; applying reset\n";
    //return true;
    apply_reset();
    return false; //won't restart sim for now; keep advancing to see new track segment
    //pid_steering.reset(); //only apply these when simulation will be reset
    //pid_throttle.reset();
    //return true;
  }
  return false;
}

void TWIDDLE::apply_reset(){
  if (chew_run){
    cout <<"finished chew run\n";
    cout << "              chewing error " << run_cum_error << "\n";
    chew_run = false;
  }else if(init_run){
    cout <<"finished init run\n";
    cout << "        found initial error " << run_cum_error << "\n";
    best_error = run_cum_error;
    init_run = false;
    //we know we're starting with steering, kp, positive
    steering.kp += steering.dp;
  }else{
    bool improved = false;
    bool inc_param_ring = false;
    bool increase_speed = false;
    cout << "        found new_error " << run_cum_error << ", vs. best_error " << best_error << "\n";
    if (run_cum_error < best_error){
      best_error = run_cum_error;
      improved = true;
      improved_this_epoch = true;
    }// run_cum_error < best_error
    switch(param_ring){
    case 0 ://was testing kp
      inc_param_ring = eval_posneg("kp",steering.kp, steering.dp, improved);
      break;
    case 1 : //was testing kd
      inc_param_ring = eval_posneg("kd",steering.kd, steering.dd, improved);
      break;
    case 2 : //was testing ki
      inc_param_ring = eval_posneg("ki",steering.ki, steering.di, improved);
      break;
    case 3 ://was testing cp
      inc_param_ring = eval_posneg("cp",st33ring.kp, st33ring.dp, improved);
      break;
    case 4 : //was testing cd
      inc_param_ring = eval_posneg("cd",st33ring.kd, st33ring.dd, improved);
      break;
    case 5 : //was testing ci
      inc_param_ring = eval_posneg("ci",st33ring.ki, st33ring.di, improved);
      break;
    }//switch param_ring (1st)
    if (inc_param_ring){
      param_ring = param_ring + 1;
      param_ring = param_ring % 6;
      //param_ring = param_ring % 3; //after 11/26/2017 bug, retrain with only main pid controller before adding 2nd
      if (param_ring == 0){ //finished epoch; check in on how we're doing 
        double dparam_sum =      (steering.dp / (init_steering.kp > 1e-7 ? init_steering.kp : 1e-7))
                               + (steering.dd / (init_steering.kd > 1e-7 ? init_steering.kd : 1e-7))
                               + (steering.di / (init_steering.ki > 1e-7 ? init_steering.ki : 1e-7))
          //                     + (st33ring.dp / (init_st33ring.kp > 1e-7 ? init_st33ring.kp : 1e-7))
          //                     + (st33ring.dd / (init_st33ring.kd > 1e-7 ? init_st33ring.kd : 1e-7))
          //                     + (st33ring.di / (init_st33ring.ki > 1e-7 ? init_st33ring.ki : 1e-7))
          ;
        cout << "\n\n";
        cout << "FINISHED EPOCH " << epoch << "\n";
        cout << "                              speed is " << target_speed << "\n";
        cout << "                              k params " << steering.kp << "," << steering.kd << "," << steering.ki << "\n";
        cout << "                              d params " << steering.dp << "," << steering.dd << "," << steering.di << "\n";
        cout << "                              k p4r4ms " << st33ring.kp << "," << st33ring.kd << "," << st33ring.ki << "\n";
        cout << "                              d p4r4ms " << st33ring.dp << "," << st33ring.dd << "," << st33ring.di << "\n";
        cout << "  best_error, new_error, max_error are " << best_error << ", " <<run_cum_error << ", " << max_single_error << "\n";
        cout << "                         dparam_sum is " << dparam_sum << "\n";
        epoch++;
        best_error = 1e6; //reset best_error
        init_run = true;

        if (dparam_sum < 1e-2 && !improved_this_epoch && max_single_error < 2.5){ //if dparam_sum is good and max_single_error was from old data, will fix after next epoch
          increase_speed = true;
        }
        max_single_error = 0;
        improved_this_epoch = false;
      }
      if (increase_speed){
        target_speed = target_speed + 1.0;
        cout << "  increasing speed to " << target_speed << " !!!!!\n";
        //best_error = 1e6; //reset best_error; changed to do this every epoch, so it doesn't get pegged to 1 fluke of a good run
        //init_run = true;
        init_steering.kp = steering.kp; //reset init params so dparam_sum normalization is more realistic
        init_steering.kd = steering.kd;
        init_steering.ki = steering.ki;
        init_st33ring.kp = st33ring.kp; //reset init params so dparam_sum normalization is more realistic
        init_st33ring.kd = st33ring.kd;
        init_st33ring.ki = st33ring.ki;
      }else if (init_run == false){
        switch(param_ring){
        case 0 ://now testing kp
          steering.kp += steering.dp;
          cout <<"    trying first side of kp; dp is "<< steering.dp << ", kp is now "<< steering.kp << "\n";
          break;
        case 1 : //now testing kd
          steering.kd += steering.dd;
          cout <<"    trying first side of kd; dd is "<< steering.dd << ", kd is now "<< steering.kd << "\n";
          break;
        case 2 : //now testing ki
          steering.ki += steering.di;
          cout <<"    trying first side of ki; di is "<< steering.di << ", ki is now "<< steering.ki << "\n";
          break;
        case 3 ://now testing cc
          st33ring.kp += st33ring.dp;
          cout <<"    trying first side of cp; dp is "<< st33ring.dp << ", cp is now "<< st33ring.kp << "\n";
          break;
        case 4 : //now testing cd
          st33ring.kd += st33ring.dd;
          cout <<"    trying first side of cd; dd is "<< st33ring.dd << ", cd is now "<< st33ring.kd << "\n";
          break;
        case 5 : //now testing ci
          st33ring.ki += st33ring.di;
          cout <<"    trying first side of ci; di is "<< st33ring.di << ", ci is now "<< st33ring.ki << "\n";
          break;
        }//switch param_ring (2nd)
      }//if else increase_speed
    }//if inc_param_ring
  }//if/else init_run
  cout << "\n";
  cout << "    starting new step with k params " << steering.kp << "," << steering.kd << "," << steering.ki << " speed is " << target_speed << "\n";
  cout << "                           d params " << steering.dp << "," << steering.dd << "," << steering.di << "\n";
  cout << "                           k p4r4ms " << st33ring.kp << "," << st33ring.kd << "," << st33ring.ki << " speed is " << target_speed << "\n";
  cout << "                           d p4r4ms " << st33ring.dp << "," << st33ring.dd << "," << st33ring.di << "\n";
  cout << "        param_ring, posneg_ring are " << param_ring << ", " << posneg_ring <<"\n";
  //  cout << "                           speed is " << target_speed << "\n";

  pid_steering.tweak(steering.kp,steering.kd,steering.ki);
  pid_st33ring.tweak(st33ring.kp,st33ring.kd,st33ring.ki);
  run_cum_error = 0.0;
  reset = false;
  step_count = 0;
  return;
}//void apply_reset

bool TWIDDLE::eval_posneg(string kname, double &k_param, double &d_param, bool improved){
  bool inc_param_ring = false;
  bool inc_posneg_ring = false;

  switch(posneg_ring){
  case 0://finished test of making more positive
    if(improved){ //got better;change to next parameter
      inc_param_ring = true;
      d_param *= fact_increase;
      cout <<"    keeping k_param "<<kname<<"; increasing d_param to "<< d_param << "\n";
    }else{ //was positive and didn't help; try negative
      k_param = k_param - (2*d_param);
      inc_posneg_ring = true;
      cout <<"    trying other side of k_param "<<kname<<"; d_param is "<< d_param << ", k_param is now "<< k_param << "\n";
    }
    break;
  case 1: //finished test of making more negative
    inc_param_ring = true;//no matter what, moving to next param
    inc_posneg_ring = true;
    if(improved){ //got better; lock this change in
      d_param *=  fact_increase; 
      cout <<"    keeping k_param "<<kname<<"; increase d_param to "<< d_param << "\n";
    }else{
      k_param += d_param;
      d_param *= fact_decrease;
      cout <<"   resetting k_param "<<kname<<" to " << k_param <<"; decreasing d_param to "<< d_param << "\n";
    }
    break;
  }
  if (inc_posneg_ring){
    posneg_ring = posneg_ring + 1;
    posneg_ring = posneg_ring % 2;
  }
  return inc_param_ring;
}
  
void TWIDDLE::update_run_cum_error(double cte){
  //can change update function as desired here
  //run_cum_error += (cte * cte);
  //penalize larger errors even more severely
  run_cum_error += (cte * cte * cte * cte);
}
