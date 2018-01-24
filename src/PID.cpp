#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double init_Kp, double init_Ki, double init_Kd) {
	// Initialize the parameters
	params = { init_Kp, init_Ki, init_Kd };
	dparams = { 0.01, 0.001, 1.0};
	
	// Initialize the errors
	errors = { 0,0,0 };
	best_err = 100;

	// initialize next direction
	index = 0;
	next_par = 0;
	is_initialized = false;
	iteration = 1;
	steps = 0;
	record = 0;
}

void PID::UpdateError(double cte) {

	if (!is_initialized) {
		// Steering command for first iteration
		steer = 0;
		cte_old = cte;
		cte_hist.push_back(cte);
		is_initialized = true;
	}
	else {
		errors[0] = cte;
		errors[1] = 0.9 * errors[1] + cte;
	//	errors[1] += cte;
		errors[2] = cte - cte_old;

		// Reset the integral
		if (fabs(cte) < 0.01) { errors[1] = cte; }

		// Twiddle Update
		if ((steps > 300) && (steps < 3000)) {
			// uncomment if twiddle optimization is done
		//	Twiddle(cte);
		}
		steps += 1;



		steer = TotalError();

		// Output
		/* 
		cout << "Iteration: " << iteration << "| Step: "<< steps<< "\n";
		cout << "Individual Errors: " << errors[0] << " " << errors[1] << " " << errors[2] << "\n";
		cout << "D Parameters: " << dparams[0] << " " << dparams[1] << " " << dparams[2] << "\n";
		cout << "Parameters: " << params[0] << " " << params[1] << " " << params[2] << "\n\n";
		*/

		cte_old = cte;

		// limit the steering
		if (steer > 1) {
			steer = 1;
		}
		else if (steer < -1) {
			steer = -1;
		}
	}
}


void PID::Twiddle(double cte) {
	int i = index;
	err = pow(cte, 2);

	// Output
	/*
	cout << "Current error: " << err << " ||| Best error: " << best_err << "\n";
	cout << "Current index: " << index << "\n";
	*/
	while (fabs(dparams[0] + dparams[1] + dparams[2]) > pow(10, -5)) {
		if (iteration == 1) {
			params[i] += dparams[i];
			iteration = 2;
		}
		else if (iteration == 2) {
			if (err < best_err) {
				best_err = err;
				dparams[i] *= 1.1;
				next_par = 1;
			}
			else {
				params[i] -= 2 * dparams[i];
				iteration = 3;
			}
		}
		else if (iteration == 3) {
			if (err < best_err) {
				best_err = err;
				dparams[i] *= 1.1;
			}
			else {
				params[i] += dparams[i];
				dparams[i] *= 0.9;
				iteration = 1;
			}
			next_par = 1;
		}
	}

	// Get next parameter
	if (next_par == 1) {
		index += 1;
		index = index % 3;
		next_par = 0;
	}
}


double PID::TotalError() {
	return  -params[0] * errors[0] - params[2] * errors[2] - params[1] * errors[1];
}

