#include "Twiddle.h"
#include <iostream>

Twiddle::Twiddle(){

}


Twiddle::~Twiddle(){
}

void Twiddle::set_p(vector<double> params) {
    p = params;
    pid.Init(params[0], params[1], params[2]);
}

void Twiddle::set_dp(vector<double> dparams) {
    dp = dparams;
}

void Twiddle::tweak(int param_index, bool tweak_up) {
    if (tweak_up) {
        p[param_index] += dp[param_index];
    }
    else {
        p[param_index] -= 2.0 * dp[param_index];
    }
    // update the controller
    pid.Init(p[0], p[1], p[2]);
}

double Twiddle::control_out(double cte) {
    pid.UpdateError(cte);
    return pid.TotalError();
}

void Twiddle::adjust_dp(int param_index, bool adjust_up){
    if (adjust_up) {
        dp[param_index] *= dp_coeff_h;
    }
    else
    {
        dp[param_index] *= dp_coeff_l;
    }   
}

void Twiddle::print_params() {
    std::cout << "Current parameters: Kp = " << p[0] << " Ki = " << p[1] << " Kd = " << p[2] << std::endl;
    std::cout << "Current twiddle values: dKp = " << dp[0] << " dKi = " << dp[1] << " dKd = " << dp[2] << std::endl;
}