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

class kalman_filter {
    public:
        void test();
};