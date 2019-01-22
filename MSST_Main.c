/*
 * main.c
 */

#include <MSST_PWM.h>
#include "MSST_GlobalFunctions.h"
#include "F28x_Project.h"
#include "Syncopation_SCI.h"

#include "Syncopation_Data.h"
#include "MSST_PWM.h"

void deadloop();
void CpuTimerInit();
void CpuTimerIsr();

#define CPU_INT_MSEC 20
//char test_c=0;

void main(void) {
	InitSysCtrl();

	EALLOW;
	ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;
	ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;
	EDIS;

    MSSTGpioConfig();
    CPU_LED_BIT = 0;

    DINT;
    InitPieCtrl();
    InterruptInit();

    SCI_Config();
    AdcInit();
    PwmInit();
    Pwm_DIS();

    DabPri_EN();

    CpuTimerInit();
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  // Enable the PIE block
    IER = M_INT1 | M_INT9;

    EINT;
	deadloop();
}

//extern float omega_h1;
extern float V_dc_filtered;
//extern float V_ac_amp;
extern float Iac_amp;
//extern float V_dc_ref;
//extern float I_react;

extern float Vac_theta;
extern float Vac;

extern Uint16 dab_prd;
extern int16 dab_phs;
extern Uint16 dab_state;

extern Uint16 V_DC;
extern Uint16 I_DC_1;
extern Uint16 I_DC_2;

extern Uint16 TEMP_1;
extern Uint16 TEMP_2;
extern Uint16 TEMP_3;
extern Uint16 TEMP_4;

extern Uint16 log_state;
extern Uint16 log_index;
extern Uint16 log_limit;

extern float Dab_Idc;
extern float Vdc;
extern float Idc_filter_slow;
extern float Vdc_filter_slow;

Uint16 log_send_count = 0;

#pragma CODE_SECTION(deadloop, ".TI.ramfunc");
void deadloop()
{
    while(1)
    {
//        if(log_state == 2)
//        {
//            DataLog_SendSample(log_index);
//            log_send_count++;
//            log_index++;
//            if(log_index>=log_limit)
//                log_index = 0;
//            if(log_send_count >= log_limit)
//            {
//                log_send_count = 0;
//                log_index = 0;
//                log_state = 0;
//            }
//        }

        SCI_UpdatePacketFloat(0, Dab_Idc);
        SCI_UpdatePacketFloat(1, Vdc);
        SCI_UpdatePacketFloat(2, Idc_filter_slow);
        SCI_UpdatePacketFloat(3, Vdc_filter_slow);

        SCI_UpdatePacketInt16(0, dab_prd);
        SCI_UpdatePacketInt16(1, V_DC);
        SCI_UpdatePacketInt16(2, I_DC_2);
        SCI_SendPacket();

        DELAY_US(4000);
    }
}

void CpuTimerInit()
{
    CpuTimer1Regs.TCR.all = 0x4010;
    CpuTimer1Regs.PRD.all = 200000 * CPU_INT_MSEC;
    EALLOW;
    PieVectTable.TIMER1_INT = &CpuTimerIsr;
    EDIS;

    DELAY_US(1);
//    CpuTimer1Regs.TCR.all = 0x4000;
}

#pragma CODE_SECTION(CpuTimerIsr, ".TI.ramfunc");
interrupt void CpuTimerIsr()
{
    CPU_LED_TOGGLE = 1;

    CpuTimer1Regs.TCR.bit.TIF = 1;
}
