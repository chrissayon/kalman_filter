# Kalman Filter Library
Kalman Filter library in C++

The Kalman Filter is a control system used for approximating states which are essentially variables you are keeping track of, e.g. distance, velocity, etc.

Kalman Filter is split between two stages:
- Prediction stage: The control system will predict what the output is based on the equation in the control system
- Update stage: The control system will correct any error that has occured in the prediction stage by correcting the values that were calculated with sensor data

#### Predict Stage
```
x_k = F(x_(k-1)) + Bu            - Predict Future State
P = F(P_(k-1))(F^Transpose) + Q  - Predict Future Covariance
```
#### Update Stage
```
S = HP(H_T) + R                  - Estimate System Uncertainty (Associated measurement noise)
K = PH/S                         - Calculate Kalman Gain
y = z - H(x_k)                   - Calculate Residual (Difference between predicted and measurement values)
x_final = (x_k) + Ky             - Update State
P_final = (I-KH)P                - Update Covariance
```
