#ifndef PID_H
#define PID_H
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
//  double p_error;
//  double i_error;
//  double d_error;
  std::vector<double> errors;
  std::vector<double> old_steer;

  /*
  * Coefficients
  */ 
//  double Kp;
//  double Ki;
//  double Kd;
  std::vector<double> params;
  std::vector<double> dparams;

  // cte history
  std::vector<double> cte_hist;

  // next direction and improvement
  double cte_old;
  double steer;
  bool is_initialized;
  int index;
  int next_par;
  int iteration;
  double err;
  double best_err;
  int steps;
  double record;


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
  void Init(double init_Kp, double init_Ki, double init_Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  // Twiddle
  void Twiddle(double cte);

};

#endif /* PID_H */
