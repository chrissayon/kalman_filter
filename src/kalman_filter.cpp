#include <stdio.h>
#include "kalman_filter.h"

kalman_filter::kalman_filter(){
    time = 0;

    state_matrix.resize(2, 1);
    state_matrix(0, 0) = 0;
    state_matrix(1, 0) = 0;
                                            /* 
                                            [distance]
                                            [velocity] 
                                            */
    
    state_transition_matrix.resize(2, 2);
    state_transition_matrix(0, 0) = 1;
    state_transition_matrix(0, 1) = time;
    state_transition_matrix(1, 0) = 0;
    state_transition_matrix(1, 1) = 1;
                                            /*
                                            [ 1 t ]
                                            [ 0 1 ]
                                            */
    
    control_function_matrix.resize(2, 1);
    control_function_matrix(0, 0) = (0.5)*(time*time);
    control_function_matrix(1, 0) = time;
                                                    /*
                                                    [ 1/2t^2 ]          
                                                    [ t ]
                                                    */
    
    control_matrix.resize(1, 1);
    control_matrix(0, 0) = 0;                     
                                            /* 
                                            [a]
                                            */
    
    covariance_matrix.resize(2, 2);
    covariance_matrix(0, 0) = 2;  
    covariance_matrix(0, 1) = 2;  
    covariance_matrix(1, 0) = 2;  
    covariance_matrix(1, 1) = 2;  
                                            /*
                                            [ variance_pos covariance_pos_vel ]
                                            [ covartiance_vel_pos variance_vel ]
                                            */
    
    process_noise_matrix.resize(2, 2);
    process_noise_matrix(0, 0) = (time*time*time*time)/4;             
    process_noise_matrix(0, 1) = (time*time*time)/2;
    process_noise_matrix(1, 0) = (time*time*time)/2;
    process_noise_matrix(1, 1) = time;
    // process_noise_matrix(0, 0) = 0  ;          
    // process_noise_matrix(0, 1) = 0;
    // process_noise_matrix(1, 0) = 0;
    // process_noise_matrix(1, 1) = 1;
                                            /*
                                            [ (T**4)/4 (T**3)/2 ]
                                            [ (T**3)/2 T ]
                                            */

    measurement_value_matrix.resize(1, 1);
    measurement_value_matrix(0, 0) = 0;           
                                            /*
                                            [ altitude ]
                                            */
    
    measurement_function_matrix.resize(1, 2);
    measurement_function_matrix(0, 0) = 1;   
    measurement_function_matrix(0, 1) = 0;      
                                            /*
                                            [ 1 0 ]
                                            */ 
    
    measurement_noise_matrix.resize(1, 1);
    measurement_noise_matrix(0, 0) = 4;           
                                            /*
                                            [ temperature_noise+pressure_noise ]
                                            */

    // Equation related variables
    system_uncertainty_matrix.resize(1, 1);
    system_uncertainty_matrix(0, 0) = 0;  
                                            /*
                                            [ s ]
                                            */
    
    kalman_gain_matrix.resize(2, 1);
    kalman_gain_matrix(0, 0) = 0;
    kalman_gain_matrix(1, 0) = 0;

                                            /*
                                            [ k ]
                                            [ k ]
                                            */
    
    residual_matrix.resize(1, 1);
    residual_matrix(0, 0) = 0;
                                            /*
                                            [ y ]
                                            */
    
    identity_matrix_set = false;
};


bool kalman_filter::predict(double update_time, double update_acceleration) {

    state_transition_matrix(0, 1) = update_time;
    control_function_matrix(0, 0) = (0.5)*(update_time*update_time);
    control_function_matrix(1, 0) = update_time;
    control_matrix(0, 0) = update_acceleration;

    process_noise_matrix(0, 0) = (update_time*update_time*update_time*update_time)/4;             
    process_noise_matrix(0, 1) = (update_time*update_time*update_time)/2;
    process_noise_matrix(1, 0) = (update_time*update_time*update_time)/2;
    process_noise_matrix(1, 1) = update_time;
    
    state_matrix = (state_transition_matrix * state_matrix) + (control_function_matrix * control_matrix); // x = Fx + Bu
    covariance_matrix = (state_transition_matrix * covariance_matrix * state_transition_matrix.transpose()) + process_noise_matrix; // P = FPF + Q

    return true;
}


bool kalman_filter::update(double update_measurement) {

    measurement_value_matrix(0, 0) = update_measurement; 
    
    system_uncertainty_matrix = (measurement_function_matrix * covariance_matrix * measurement_function_matrix.transpose()) + measurement_noise_matrix;
    system_uncertainty_matrix(0, 0) = 1/system_uncertainty_matrix(0, 0);
    
    kalman_gain_matrix = (covariance_matrix * measurement_function_matrix.transpose()) * (system_uncertainty_matrix);
    
    residual_matrix = measurement_value_matrix - (measurement_function_matrix * state_matrix);
    
    state_matrix = state_matrix + (kalman_gain_matrix * residual_matrix);
    
    KH = kalman_gain_matrix * measurement_function_matrix;
    
    if(identity_matrix_set == false){
       
        identity_matrix_set = true;
        identity_matrix.resize(KH.rows(), KH.cols());

        for(int i = 0; i < KH.rows(); i++){

            for(int j = 0; j < KH.cols(); j++){
                
                if(i == j){
                    identity_matrix(i, j) = 1;
                } else {
                    identity_matrix(i, j) = 0;
                }

            }

        }
    }

    KH = identity_matrix - KH;
    
    // covariance_matrix = KH*covariance_matrix*KH.transpose() + kalman_gain_matrix*measurement_noise_matrix*kalman_gain_matrix.transpose();
    covariance_matrix = KH*covariance_matrix;

    return true;
}