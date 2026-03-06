#ifndef KALMAN_H
#define KALMAN_H

#include "ari_math/ari_matrix.h"



template <int StateDim, int InputDim, int MeasDim>
class Kalman
{
private:
    /* --- System Matrices --- */
    
    // State transition matrix (A): Relates the previous state to the current state
    ari_matrix<StateDim, StateDim> A;
    
    // Control input matrix (B): Relates the optional control input to the state
    ari_matrix<StateDim, InputDim> B;
    
    // Observation/Measurement matrix (H): Relates the state to the measurement
    ari_matrix<MeasDim, StateDim> H;

    /* --- State and Covariance Matrices --- */

    // State vector (x): The estimated current state of the system
    ari_matrix<StateDim, 1> x;
    
    // State estimate covariance matrix (P): Represents the uncertainty of the state estimate
    ari_matrix<StateDim, StateDim> P;
    
    // Process noise covariance matrix (Q): Represents uncertainty in the system model
    ari_matrix<StateDim, StateDim> Q;
    
    // Measurement noise covariance matrix (R): Represents uncertainty in the sensor readings
    ari_matrix<MeasDim, MeasDim> R;
    
    // Identity matrix (I): Used for covariance update calculations
    ari_matrix<StateDim, StateDim> I;

    /* --- Intermediate Matrices for Calculations --- */

    // Kalman Gain (K): Determines how much the measurement should correct the state estimate
    ari_matrix<StateDim, MeasDim> K;
    
    // Innovation covariance (S): Represents the uncertainty of the measurement prediction
    ari_matrix<MeasDim, MeasDim> S;

public:
    /**
     * @brief Constructor to initialize the Kalman Filter with system parameters.
     * * @param sys_A State transition matrix
     * @param sys_B Control input matrix
     * @param obs_H Observation/Measurement matrix
     * @param cov_Q Process noise covariance matrix
     * @param cov_R Measurement noise covariance matrix
     * @param init_P Initial state estimate covariance matrix
     */
    Kalman(const ari_matrix<StateDim, StateDim>& sys_A,
           const ari_matrix<StateDim, InputDim>& sys_B,
           const ari_matrix<MeasDim, StateDim>& obs_H,
           const ari_matrix<StateDim, StateDim>& cov_Q,
           const ari_matrix<MeasDim, MeasDim>& cov_R,
           const ari_matrix<StateDim, StateDim>& init_P) 
    {
        // Assign provided matrices to internal variables
        A = sys_A;
        B = sys_B;
        H = obs_H;
        Q = cov_Q;
        R = cov_R;
        P = init_P;

        /* Set P to identity matrix scaled by 0.1f */
        P.setIdentity();
        P = P * 0.1f;

        /* Set R matrix to identity matrix scaled by 0.05f */
        R.setIdentity();
        R = R * 0.05f;  // Measurement (accelerometer) noise

        // Initialize state vector to zero
        x.setZero(); 
        
        // Setup the Identity matrix
        I.setIdentity(); 
    }

    void predict(ari_matrix<InputDim,1> u)
    {
        // Predict the current state
        x = A * x + B * u;

        // Predict error covariance
        P = A * P * A.transpose() + Q;

    }

    void update(ari_matrix<MeasDim,1> z)
    {
        /* Update innovation matrix S = HPH' + R */
        S = H * P * H.transpose() + R;

        /* Update Kalman gain matrix K = PH'(HPH'+R)⁻¹ */
        K = P * H.transpose() * S.inverse(S);

        /* Update the state according to the measurements */
        x = x + K * (z - H * x);

        /* Update covariance matrix */
        P = (I - K * H) * P;

    }

    /* Helper functions to get state variables */
    float getState(int r, int k){return x(r,k);}
    
};







#endif