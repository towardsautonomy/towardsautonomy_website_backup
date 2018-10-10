#include <stdio.h>
#include <math.h>

#define CORRECT_VAL	72

double est;				/**< Current Estimation */
double mes_val;			/**< Measured Value */
double kg;				/**< Kalman Gain */
double err_est;			/**< Estimation Error */
double err_mes;			/**< Possible Measurement Error */

double calcKalmanGain(double e_est, double e_mes) {
	return (e_est / (e_est + e_mes));
}

double updateEstimate(double est_p, double kg, double mes) {
	return (est_p + kg*(mes - est_p));
}

double updateErrEstimate(double kg, double e_est) {
	return ((1 - kg)*e_est);
}

int main(int argc, char *argv[]) {
	err_mes = 4;
	est = 68;
	err_est = 2;
	
    for ( ; ; ) {
        printf("Enter the new measured value (%f +/- %f): ", (double)CORRECT_VAL, err_mes);
		scanf("%lf", &mes_val);
		kg = calcKalmanGain(err_est, err_mes);
		est = updateEstimate(est, kg, mes_val);
		err_est = updateErrEstimate(kg, err_est);
		printf("\tKalman Gain = %f\n\tEstimated Value = %f\n\n", kg, est);
	}
 
    return 0;
}