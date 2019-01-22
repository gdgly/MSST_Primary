/*
 * DAB_Primary.h
 *
 *  Created on: Oct 19, 2018
 *      Author: y3437
 */
#include "F28x_Project.h"

#ifndef MSST_PWM_H_
#define MSST_PWM_H_

extern void PwmInit();

extern void Pwm_EN();
extern void Pwm_DIS();
extern void Rectifier_EN();
extern void Rectifier_DIS();
extern void RectDuty_SET(float duty);

extern void DabPri_EN();
extern void DabPri_DIS();
extern void Dab_UpdateRegister();

extern void DabFreq_INC();
extern void DabFreq_DEC();
extern void DabPhs_INC();
extern void DabPhs_DEC();
extern void DabPhs_SET(int16 arg);

#endif /* MSST_PWM_H_ */
