#include "PID.h"
#include <math.h>
#include <time.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(std::string Name, double Kp, double Ki, double Kd) {
  //cout<<"initializing pid with kp "<<Kp<<"\n";
  name_ = Name;
  kp_ = Kp;
  ki_ = Ki;
  kd_ = Kd;
  //current_cte_ = 0.0;
  //prev_cte_ = 0.0;
  //delta_cte_ = 0.0;
  int_cte_ = 0.0;
  //max_int_ = 20.0; //random guess
  max_int_ = 1000.0; //random guess; want to practically disable for now
  //prev_time_ = 0.0;
  nom_dt_    = ((double)1500) / CLOCKS_PER_SEC; //average dt value from before I made it time-based instead of iteration based
  first_iteration_ = true;
  //cout<<"finished init\n";
}

void PID::reset(){
  current_cte_ = 0.0;
  prev_cte_ = 0.0;
  delta_cte_ = 0.0;
  int_cte_ = 0.0;
  delta_t_ = 0.0;
  prev_time_ = clock();
  current_time_ = clock();
  first_iteration_ = true;
}

void PID::tweak(double Kp, double Ki, double Kd){
  kp_ = Kp;
  ki_ = Ki;
  kd_ = Kd;
}

void PID::UpdateError(double cte) {
  if (first_iteration_){
    //cout<<"first UpdateError call with cte "<<cte<<"\n";
    current_time_ = clock();
    current_cte_ = cte;
  }
  //cout<<"UpdateError call with cte "<<cte<<"\n";
  prev_time_ = current_time_;
  prev_cte_ = current_cte_;
  current_time_ = clock();
  current_cte_ = cte;
  delta_t_ = ((double)(current_time_ - prev_time_)) / CLOCKS_PER_SEC;
  //cout <<"delta_t_ is "<<delta_t_<<"\n";
  //cout <<"clocks_per_sec is "<< CLOCKS_PER_SEC<<"\n";
  //cout <<"nom_dt is "<<nom_dt_<<"\n";
  delta_cte_ = (current_cte_ - prev_cte_) * (delta_t_ > 1e-6 ? (nom_dt_ / delta_t_) : 1); //if delta_t_ is twice as large as nominal, slope is half what iteration would imply
  //if(prev_cte_ * current_cte_ < 0){ //error has changed from positive to negative
    //int_cte_ = 0; //set integral to 0, as suggested in George Gillard paper
  //}else if{
  //I see that sometimes, the car recovers part way through a turn,
  //then loses the recovery when it hits the midpoint.  Trying to unwind integral more gradually
  if (int_cte_ * current_cte_ < 0){
    //int_cte_ *= 0.8;
    //int_cte_ *= 0.90;
    int_cte_ *= 0.95;
  }
  //int_cte_ += current_cte_ * (delta_t_ / nom_dt_); //basic integral, without checking against max
   if (fabs(int_cte_) < max_int_ || int_cte_ * current_cte_ < 0){ //only add if int_cte is less than max, of if current_cte_ will decrease magnitude of int_cte
     int_cte_ += current_cte_ * (delta_t_ / nom_dt_); //if delta_t_ is twice as large as nominal, integral should grow twice as much
   }
  first_iteration_ = false;
  //cout<<name_<<" UpdateError with current_cte,delta_cte,int_cte "<<current_cte_<<","<<delta_cte_<<","<<int_cte_<<"\n";
}

double PID::TotalError() {
  
  double te = -1 * (
                      (kp_ * current_cte_)
                    + (kd_ * delta_cte_  )
                    + (ki_ * int_cte_    )
                   );

  //cout<<"Finished TotalError:"<<te<<"\n";
  return te;
}

