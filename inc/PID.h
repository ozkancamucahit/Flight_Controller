/*
 * PID.h
 *
 * Created: 10/01/2021 13:16:06
 *  Author: mmuca
 */ 


#ifndef PID_H_
#define PID_H_

typedef struct PID
{
    /*Gains*/
    float Kp, Ki, Kd;

    /* D low pass filter timeconstant */
    float tau;

    /* Output limits */
    float limMin, limMax;

    /* Integrator limits */
    float limMinInt, limMaxInt;

    /* Sample time in secs */
    float T;

    /* COntroller Memory */
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;

    /* Controller output */
    float out;

}PIDController_t;

void PIDController_Init( PIDController_t* pPID );
float PIDController_Update( PIDController_t* pPID, float setPoint, 
float measurement);

#endif /* PID_H_ */