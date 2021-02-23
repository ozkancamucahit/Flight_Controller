/*
 * PID.c
 *
 * Created: 10/01/2021 13:16:23
 *  Author: mmuca
 */ 

#include "PID.h"


/**
 *  \brief Reset or init
 *  \param pPID pointer to PID Struct
 */
void PIDController_Init( PIDController_t* pPID )
{
    pPID->differentiator = 0.0f;
    pPID->integrator     = 0.0f;
    pPID->prevError      = 0.0f;
    pPID->prevMeasurement= 0.0f;
    pPID->out            = 0.0f;
}


/**
 * \brief update params and recalculate geins
 * 
 * \param pPID  pointer to PID Struct
 * \param setPoint Desired point
 * \param measurement current point
 * \return out adjustment
 */
float PIDController_Update( PIDController_t* pPID, float setPoint, 
float measurement)
{

    /*
	* Error signal
	*/
    float error = setPoint - measurement;


	/*
	* Proportional
	*/
    float proportional = pPID->Kp * error;


	/*
	* Integral
	*/
    pPID->integrator = pPID->integrator + 0.5f * pPID->Ki * pPID->T * (error + pPID->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pPID->integrator > pPID->limMaxInt) {

        pPID->integrator = pPID->limMaxInt;

    } else if (pPID->integrator < pPID->limMinInt) {

        pPID->integrator = pPID->limMinInt;

    }


	/*
	* Derivative (band-limited differentiator)
	*/
		
    pPID->differentiator = -(2.0f * pPID->Kd * (measurement - pPID->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pPID->tau - pPID->T) * pPID->differentiator)
                        / (2.0f * pPID->tau + pPID->T);


	/*
	* Compute output and apply limits
	*/
    pPID->out = proportional + pPID->integrator + pPID->differentiator;

    if (pPID->out > pPID->limMax) {

        pPID->out = pPID->limMax;

    } else if (pPID->out < pPID->limMin) {

        pPID->out = pPID->limMin;

    }

	/* Store error and measurement for later use */
    pPID->prevError       = error;
    pPID->prevMeasurement = measurement;

	/* Return controller output */
    return pPID->out;

}














