/*
 * MSST_Primary_Control.c
 *
 *  Created on: Dec 17, 2018
 *      Author: y3437
 */

#include "F28x_Project.h"
#include "MSST_PWM.h"
#include "Syncopation_Data.h"
#include "MSST_GlobalFunctions.h"

#define TS 2e-5


Uint16 rect_state = 0;

void Rectifier_Start()
{
    rect_state = 1;
//    Rectifier_EN();
}

//Uint16 log_index = 0;
int16 log_length = 8000;

void Logging_length(int16 arg_1)
{
    log_length = arg_1;
}


float Vdc_ref = 200;
float Iac_ref = 0;
float Iac_mag = 0;
float Vac_ref = 0;

// I_loop_variables
float i_kp = 40;
float i_kr = 100;
float i_sogi_x1 = 0;
float i_sogi_x2 = 0;
float i_dc_offset_inte = 0;
float i_sogi_error;
float i_omega_h1_T;
// End of I_loop_variables

float current_loop(float Iac_ref, float Iac, float Freq);
float voltage_sogi(float Vdc, float Freq);
float voltage_loop(float Vdc_ref, float Vdc_filtered);

Uint16 sample_count = 0;

float RectifierControl(float Vac_theta, float freq, float Vac, float Iac, float Vdc, float Vdc_filter)
{
    float duty = 0.5;

//    if(rect_state == 1)
//    {
//        Iac_mag = voltage_loop(Vdc_ref, Vdc_filter);
//
//        if(Iac_mag > 20)
//            Iac_mag = 20;
//
//
//        Iac_ref = Iac_mag * __cospuf32(Vac_theta);
//
//        Vac_ref = current_loop(Iac_ref, Iac, freq);
//
//        duty = Vac_ref / Vdc;
//    }
//    if((getDataLogState() == 0) && (sample_count > 5))
//    {
//        sample_count = 0;
////        DataLog_Logging(log_index,Vdc,Vdc_filter,Iac_mag,Iac_ref);
//        log_index++;
//    }
//    else
//    {
//        sample_count++;
//    }
//
//    if(log_index >= log_length)
//    {
//        Rectifier_Stop();
//        log_index = 0;
////        DataLog_StartToSend(log_length);
//    }
    return duty;
}

//
//


void current_loop_pr(float arg_2, float arg_3)
{
    i_kp = arg_2;
    i_kr = arg_3;
}

#pragma CODE_SECTION(current_loop, ".TI.ramfunc");
float current_loop(float Iac_ref, float Iac, float Freq)
{
    i_omega_h1_T = Freq * TS;

    float i_sogi_h1_x1_n;
    float i_sogi_h1_x2_n;

    i_sogi_error = -Iac_ref + Iac;

    i_sogi_h1_x1_n = __cospuf32(i_omega_h1_T) * i_sogi_x1 - __sinpuf32(i_omega_h1_T) * i_sogi_x2 + TS * 377 * i_sogi_error;
    i_sogi_h1_x2_n = __sinpuf32(i_omega_h1_T) * i_sogi_x1 + __cospuf32(i_omega_h1_T) * i_sogi_x2;

    i_sogi_x1 = i_sogi_h1_x1_n;
    i_sogi_x2 = i_sogi_h1_x2_n;

    float Vac_ref = i_kp * i_sogi_error + i_kr * i_sogi_x1;

    return Vac_ref;
}

float vdc_h1_x1 = 0;
float vdc_h1_x2 = 0;
float vdc_h2_x1 = 0;
float vdc_h2_x2 = 0;
float vdc_h0_x = 160;
float vdc_sogi_k = 1.414;
float vdc_h0_k = 100;

void v_sogi_k_p(float arg_2, float arg_3)
{
    vdc_sogi_k = arg_2;
    vdc_h0_k = arg_3;
}

#pragma CODE_SECTION(voltage_sogi, ".TI.ramfunc");
float voltage_sogi(float Vdc, float Freq)
{
    float v_omega_T = Freq * TS;
    float v_2omega_T = 2 * Freq * TS;

    float vdc_h1_x1_n;
    float vdc_h1_x2_n;
    float vdc_h2_x1_n;
    float vdc_h2_x2_n;
    float vdc_h0_x_n;

    float v_sogi_error = vdc_sogi_k * (Vdc - vdc_h1_x1 - vdc_h2_x1 - vdc_h0_x);

    vdc_h0_x_n = vdc_h0_x + vdc_h0_k * v_sogi_error * TS;

    vdc_h1_x1_n = __cospuf32(v_omega_T) * vdc_h1_x1 - __sinpuf32(v_omega_T) * vdc_h1_x2 + TS * 377 * v_sogi_error;
    vdc_h1_x2_n = __sinpuf32(v_omega_T) * vdc_h1_x1 + __cospuf32(v_omega_T) * vdc_h1_x2;

    vdc_h2_x1_n = __cospuf32(v_2omega_T) * vdc_h2_x1 - __sinpuf32(v_2omega_T) * vdc_h2_x2 + TS * 377 * 2 * v_sogi_error;
    vdc_h2_x2_n = __sinpuf32(v_2omega_T) * vdc_h2_x1 + __cospuf32(v_2omega_T) * vdc_h2_x2;


    vdc_h1_x1 = vdc_h1_x1_n;
    vdc_h1_x2 = vdc_h1_x2_n;
    vdc_h2_x1 = vdc_h2_x1_n;
    vdc_h2_x2 = vdc_h2_x2_n;
    vdc_h0_x = vdc_h0_x_n;

    return vdc_h0_x;
}

float v_loop_inte = 0;
//float v_loop_kp = 0.04;
//float v_loop_ki = 4;
float v_loop_error = 0;

float v_loop_kp = 0.05;
float v_loop_ki = 0.5;

void v_loop_pi(float arg_2, float arg_3)
{
    v_loop_kp = arg_2;
    v_loop_ki = arg_3;
}

#pragma CODE_SECTION(voltage_loop, ".TI.ramfunc");
float voltage_loop(float Vdc_ref, float Vdc_filtered)
{
    v_loop_error = Vdc_ref - Vdc_filtered;

    v_loop_inte += TS * v_loop_error;

    float I_ac_ref = v_loop_kp * v_loop_error + v_loop_ki * v_loop_inte;

    return I_ac_ref;
}


void Rectifier_Stop()
{
    Rectifier_DIS();
    rect_state = 0;

    i_sogi_x1 = 0;
    i_sogi_x2 = 0;
    i_dc_offset_inte = 0;

    vdc_h1_x1 = 0;
    vdc_h1_x2 = 0;
    vdc_h2_x1 = 0;
    vdc_h2_x2 = 0;
    vdc_h0_x = 160;

    v_loop_inte = 0;

    Iac_ref = 0;
    Iac_mag = 0;
    Vac_ref = 0;
}

//float *probe;
//#pragma CODE_SECTION(DataLogWr, ".TI.ramfunc");
//void DataLogWr()
//{
//    if(DataLog_state == 0)
//    {
//        DataLog[DataLog_index] = omega_h1;
//        DataLog_index++;
//        DataLog[DataLog_index] = V_ac_amp;
//        DataLog_index++;
//        DataLog[DataLog_index] = Iac_amp;
//        DataLog_index++;
//        DataLog[DataLog_index] = V_dc_filtered;
//        DataLog_index++;
//        if(DataLog_index >= 20000)
//        {
//            DataLog_index = 0;
//            DataLog_state = 2;
////            control_state = 0;
//        }
//    }
//}
//
//#pragma CODE_SECTION(ResetStateVariables, ".TI.ramfunc");
//void ResetStateVariables()
//{
//    i_sogi_x1 = 0;
//    i_sogi_x2 = 0;
//    i_dc_offset_inte = 0;
//    v_loop_inte = 0;
//}
//
//void V_dc_ref_set(float arg_2)
//{
//    if((arg_2 > 200) && (arg_2 < 850))
//        V_dc_ref = arg_2;
//}
//
//void V_dc_ref_inc()
//{
//    if(V_dc_ref < 850)
//        V_dc_ref += 1;
//}
//
//void V_dc_ref_dec()
//{
//    if(V_dc_ref > 200)
//        V_dc_ref -= 1;
//}
//
//void Q_ref_set(float arg_2)
//{
//    if((arg_2 > -5) && (arg_2 < 5))
//        I_react = arg_2;
//}
//
//void I_offset(float arg_2)
//{
////    EPwm9Regs.CMPA.bit.CMPA = (Uint16)arg_2;
//}
//
//void I_loop_PR(float arg_2,float arg_3)
//{
////    i_kp = arg_2;
////    i_ki = arg_3;
//}
