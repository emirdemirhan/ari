#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include "ari_math/ari_matrix.h"

class Kalman
{
private:

    /* Sytem matrix A (6,6) */
    ari_Matrix6f A;

    /* Control input matrix (6,3) which is compatible with measurement (3,1) */
    ari_matrix<6,3> B;

    /* State matrix (6,1)*/
    ari_Vector6f x; 

    /* Error covariance matrix (6,6) */
    ari_Matrix6f P;

    /* Process covariance matrix (6,6) */
    ari_Matrix6f Q;

    /* Measurement covariance matrix (2,2) */
    ari_matrix<2,2> R;

    /* Identity matrix (6,6) */
    ari_Matrix6f I;

    /* Previous state (6,1) */
    ari_Vector6f x_prev;

    /* Kalman gain (6,2) */
    ari_matrix<6,2> K;

    /* Innovation covariance matrix (2,2) */
    ari_matrix<2,2> S;

    /* Observation matrix (2,6) */
    ari_matrix <2,6> H;


    float _dt;
    
public:

    Kalman(float dt) : _dt(dt)
    { 
            /* System matrix A is considered as 
        \theta_(k+1) = \theta_k + (w_theta - bias_theta) * dt   */
        A = {
            1.0f, 0.0f, 0.0f, -_dt, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f, -_dt, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f, 0.0f, -_dt,
            0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
        };

    /* Control input matrix (6,3) which is compatible with measurement (3,1) */
        B = { _dt,0.0f,0,
                        0,_dt,0,
                        0,0,_dt,
                        0,0,0,
                        0,0,0,
                        0,0,0};
        /* Set P matrix */
        P.setIdentity();
        P = P * 0.1f;

        /* Set Q matrix */
        Q.setIdentity();

        // Angle variances
        Q(0,0) = 0.01f;
        Q(1,1) = 0.01f;
        Q(2,2) = 0.01f;

        // Bias variances
        Q(3,3) = 0.001f;
        Q(4,4) = 0.001f;
        Q(5,5) = 0.001f;

        /* Set R matrix */
        R.setIdentity();
        R = R * 0.05f;  // Measurement (accelerometer) noise

        /* Set Identity matrix */
        I.setIdentity();

        /* Initialize innovation covariance matrix S = HPH' + R */
        S = H * P * H.transpose() + R;

        /* Initialize Kalman gain matrix K = PH'(HPH'+R)⁻¹ */
        K = P * H.transpose() * S.inverse(S);



    }

    void predict(ari_Vector3f u)
    {
        // Predict the current state
        x = A * x_prev + B * u;

        // Predict error covariance
        P = A * P * A.transpose() + Q;

    }

    void update(ari_matrix<2,1> z)
    {
        /* Update the state according to the measurements */
        x = x + K * (z - H * x);

        /* Update innovation matrix S = HPH' + R */
        S = H * P * H.transpose() + R;

        /* Update previous state as current state */
        x_prev = x;

    }

    /* Helper functions to get euler angles */
    float getRoll(void){return x(1,0);}
    float getPitch(void){return x(1,0);}
    float getYaw(void){return x(1,0);}

    
};







#endif