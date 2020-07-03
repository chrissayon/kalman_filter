// Kalman Filter
// The Kalman Filter is used for approximating variables by feeding in information from other
// that are directly correlation as some type of equation.

// Kalman Filter follows the equations:
// Predict Stage
// x_k = F(x_(k-1)) + Bu            - Predict Future State
// P = F(P_(k-1))(F^Transpose) + Q  - Predict Future Covariance

// Update Stage
// S = HP(H_T) + R                  - Estimate System Uncertainty (Associated measurement noise)
// K = PH/S                         - Calculate Kalman Gain
// y = z - H(x_k)                   - Calculate Residual (Difference between predicted and measurement values)
// x_final = (x_k) + Ky             - Update State
// P_final = (I-KH)P                - Update Covariance

#include <Eigen>

using Eigen::MatrixXd;
using Eigen::Transpose;

class kalman_filter {
    private:
        double time;
        double acceleration;
        bool identity_matrix_set;
             

    public:
        // User initialize variables
        int state_dimensions[1];                // Specify x - State Dimension, always M x 1
        MatrixXd state_matrix;              
                                                /* 
                                                [distance]
                                                [velocity] 
                                                */
        
        int state_transition_dimensions[1];     // Specify F - State Transition Dimension
        MatrixXd state_transition_matrix;      
                                                /*
                                                [ 1 t ]
                                                [ 0 1 ]
                                                */
        
        int control_function_dimensions[1];     // Specify B - Control Function Dimension
        MatrixXd control_function_matrix;   
                                                /*
                                                [ 1/2t^2 ]          
                                                [ t ]
                                                */

        int control_dimensions[1];              // Specify u - Control Dimension
        MatrixXd control_matrix;                  
                                                /* 
                                                [a]
                                                */

        int covariance_dimensions[1];           // Specify P - Covariance Dimension       
        MatrixXd covariance_matrix;         
                                                /*
                                                [ variance_pos covariance_pos_vel ]
                                                [ covartiance_vel_pos variance_vel ]
                                                */

        int process_noise_dimensions[1];        // Specify Q - Process Noise Dimension
        MatrixXd process_noise_matrix;      
                                                /*
                                                [ (T**4)/4 (T**3)/2 ]
                                                [ (T**3)/2 T ]
                                                */
         
        int measurement_value_dimensions[1];    // Specify z - Measurement Value Dimension
        MatrixXd measurement_value_matrix;        
                                                /*
                                                [ altitude ]
                                                */
        
        int measurement_function_dimensions[1];     // Specify H - Measurement Function Dimension
        MatrixXd measurement_function_matrix;   
                                                    /*
                                                    [ 1 0 ]
                                                    */ 
        
        int measurement_noise_dimensions[1];    // Specify R - Measurement Noise Dimension          
        MatrixXd measurement_noise_matrix;         
                                                /*
                                                [ temperature_noise+pressure_noise ]
                                                */

        // Equation related variables
        int system_uncertainty_dimensions[1]; // Specify S - System Uncertainty Dimension
        MatrixXd system_uncertainty_matrix;    
                                                /*
                                                [ s ]
                                                */

        int kalman_gain_dimensions[1];        // Specify K - Kalman Gain Dimension
        MatrixXd kalman_gain_matrix;      
                                                /*
                                                [ k ]
                                                [ k ]
                                                */

        int residual_dimensions[1];           // Specify y - Residual Dimension
        MatrixXd  residual_matrix;        
                                                /*
                                                [ y ]
                                                [ y ]
                                                */

        MatrixXd  KH; //Created so that the dimensions of the identity matrix for the equation are known
        MatrixXd  identity_matrix; //Created so that the dimensions of the identity matrix for the equation are known


        kalman_filter();
        bool predict(double update_time, double update_acceleration);
};