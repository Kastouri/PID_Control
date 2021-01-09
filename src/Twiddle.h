#include <vector>
#include <uWS/uWS.h>
#include "PID.h"
using std::vector;

#ifndef TWIDDLE_H
#define TWIDDLE_H
class Twiddle {
public:
    /**
     * Constructor
     */
    Twiddle();
    /**
     * Destructor
     */
    virtual ~Twiddle();

    /**
     * Controller to tweak
     */
    PID pid;

    /**
     * Controller Parameter vector
     */

    vector<double> p;
    
    /**
     * twiddle Parameter vector
     */
    vector<double> dp;

    /**
     * Sets the parameters of the controller
     * @param params The vector of parameter
     */
    void set_p (vector<double> params);
    /**
     * Sets the parameters of the twiddle vector
     * @param params The vector of parameter
     */
    void set_dp (vector<double> dparams);

    /**
     * Calculates the controllers output
     * @param cte The current of the cross track error
     * @param dt time since last call
     */
    double control_out(double cte);

    /**
     * Tweaks the parameters of the controller using the dp values up or down
     * @param pram_index The index of the parameter to be twaeked
     * @param tweak_up Chooses between p[i] += dp[i] and p[i] -= 2 * dp[i]
     */
    void tweak(int pram_index, bool tweak_up);

    /**
     * Adjusts the dp values
     * @param pram_index The index of the parameter to be twaeked
     * @param adjust_up Choses were the value dp will be incremented or decreased
     */
    void adjust_dp(int pram_index, bool adjust_up); 
    
    /**
     * Prints the parametes of the controller
     */
    void print_params();

private:
    int tweaked_param_i = 0;
    const double dp_coeff_h = 1.1;
    const double dp_coeff_l = 0.9;
};
#endif  // TWIDDLE_H