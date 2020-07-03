#include <stdio.h>
#include <iostream>
#include "kalman_filter.h"

using std::cout;
using std::endl;

int main(){
    kalman_filter kalmanFilter;
    kalmanFilter.predict(1, 10);
    cout << "State Matrix Prediction is: " << endl;
    cout << kalmanFilter.state_matrix << endl << endl;

    cout << "Covariance Prediction is: " << endl;
    cout << kalmanFilter.covariance_matrix << endl << endl;

    cout << "Process Noise is: " << endl;
    cout << kalmanFilter.process_noise_matrix << endl << endl;
    
    kalmanFilter.update(10);
    cout << "System Uncertainty is: " << endl;
    cout << kalmanFilter.system_uncertainty_matrix << endl << endl;
    
    cout << "Kalman Gain is: " << endl;
    cout << kalmanFilter.kalman_gain_matrix << endl << endl;

    cout << "Residual is: " << endl;
    cout << kalmanFilter.residual_matrix << endl << endl;

    cout << "Measurement value is: " << endl;
    cout << kalmanFilter.measurement_value_matrix << endl << endl;

    cout << "Measurement function is: " << endl;
    cout << kalmanFilter.measurement_function_matrix << endl << endl;

    cout << "Identity Matrix is: " << endl;
    cout << kalmanFilter.identity_matrix << endl << endl;

    cout << "KH is: " << endl;
    cout << kalmanFilter.KH << endl << endl;

    cout << "State Matrix Update is: " << endl;
    cout << kalmanFilter.state_matrix << endl << endl;

    cout << "Covariance Update is: " << endl;
    cout << kalmanFilter.covariance_matrix << endl << endl << endl << endl;
}