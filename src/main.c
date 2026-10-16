/*==================================================================================================
* Project : HMC (S32K144 motor controller)
* Platform : CORTEXM
* Peripheral : S32K14X
*
* Combined firmware: 3-phase complementary center-aligned PWM (FTM0, dead time)
*                    + 2-channel ADC0 acquisition triggered by PDB0, moved by eDMA ch0.
*
* Built from the two working projects:
*   PWM_CENTER_ALGN_3P/src/main.c
*   ADC/src/main.c
==================================================================================================*/

/**
*   @file main.c
*
*   @addtogroup main_module main module documentation
*   @{
*/

/* Including necessary configuration files. */
#include "Mcal.h"
#include "Clock_Ip.h"
#include "Port_Ci_Port_Ip.h"
#include "Gpio_Dio_Ip.h"
#include "Ftm_Pwm_Ip.h"
#include "Adc_Ip.h"
#include "Pdb_Adc_Ip.h"
#include "Dma_Ip.h"
#include "Dma_Ip_Cfg.h"
#include "IntCtrl_Ip.h"
#include "IntCtrl_Ip_Cfg.h"
#include "S32K144_DMAMUX.h"
#include "stdint.h"

/* ---------------------------------------------------------------- PWM */
#define FTM_INSTANCE        0U
#define PWM_PERIOD_TICKS    4800U   /* center-aligned, FTM0 on system clock (48 MHz) */
#define PWM_DUTY_TICKS      2400U   /* 50 % */
#define PWM_CH_U            0U      /* ch0 main, ch1 complement (PTC0/PTC1) */
#define PWM_CH_V            2U      /* ch2 main, ch3 complement (PTC2/PTC3) */
#define PWM_CH_W            4U      /* ch4 main, ch5 complement (PTB4/PTB5) */

/* Gate driver control pins, set high before any PWM pulse */
#define PIN_EN_PORT         IP_PTE
#define PIN_EN              2U      /* PTE2 */
#define PIN_FAULT_PORT      IP_PTE
#define PIN_FAULT           6U      /* PTE6 */

/* ---------------------------------------------------------------- ADC */
#define ADC_NUM_CH          2U
#define DMAMUX_SRC_ADC0     42U     /* EDMA_REQ_ADC0 in S32K144_features.h */
#define DMAMUX_CH_ADC       0U      /* must match DMA_IP_HW_CH_0 in the mex */
#define V_REF               5000U   /* mV */
#define ADC_RESOLUTION      4095U
#define ADC_INSTANCE        0U
#define PDB_INSTANCE        0U

/* Exit codes (nonzero stops the idle loop) */
#define ERR_INTCTRL         2
#define ERR_DMA             3

volatile int exit_code = 0;

/* ADC results, updated by AdcDma_Callback */
volatile uint32_t raw_adc_ch0 = 0U;
volatile uint32_t raw_adc_ch1 = 0U;
volatile uint32_t voltage_ch0 = 0U;   /* mV */
volatile uint32_t voltage_ch1 = 0U;   /* mV */
volatile uint32_t dma_done_count = 0U;

/* DMA destination: eDMA copies ADC0->R[0] and ADC0->R[1] here (2 bytes each) */
static volatile uint16_t adc_dma_buf[ADC_NUM_CH];

/* Installed as IntCallback of DMA_LOGIC_CH_0 in the mex; runs on major loop complete */
void AdcDma_Callback(void)
{
    dma_done_count++;
    raw_adc_ch0 = adc_dma_buf[0];
    raw_adc_ch1 = adc_dma_buf[1];
    voltage_ch0 = (raw_adc_ch0 * V_REF) / ADC_RESOLUTION;
    voltage_ch1 = (raw_adc_ch1 * V_REF) / ADC_RESOLUTION;
}

/*!
  \brief The main function for the project.
  \details Startup sequence:
 * - startup asm routine
 * - main()
*/
int main(void)
{
    /* Clocks and pins */
    Clock_Ip_Init(&Clock_Ip_aClockConfig[0U]);
    Port_Ci_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals,
        g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

    /* Interrupt controller first: installs Dma0_Ch0_IRQHandler in the RAM vector table, enables DMA0_IRQn */
    if (IntCtrl_Ip_Init(&IntCtrlConfig_0) != INTCTRL_IP_STATUS_SUCCESS)
    {
        exit_code = ERR_INTCTRL;
    }

    /* Applies the mex TCD (offsets/sizes/loops), major interrupt and callback */
    if (Dma_Ip_Init(&Dma_Ip_xDmaInitPB) != DMA_IP_STATUS_SUCCESS)
    {
        exit_code = ERR_DMA;
    }

    /* Route the ADC0 DMA request (slot 42) to eDMA channel 0 */
    IP_DMAMUX->CHCFG[DMAMUX_CH_ADC] = 0U;
    IP_DMAMUX->CHCFG[DMAMUX_CH_ADC] = DMAMUX_CHCFG_ENBL(1U) | DMAMUX_CHCFG_SOURCE(DMAMUX_SRC_ADC0);

    Adc_Ip_Init(ADC_INSTANCE, &AdcHwUnit_0);   /* DmaEnable=TRUE sets ADC_SC2[DMAEN] */
    Adc_Ip_DoCalibration(ADC_INSTANCE);

    /* Address fields cannot be set in the mex: source = ADC0->R[0], destination = adc_dma_buf */
    {
        Dma_Ip_LogicChannelTransferListType list[2];
        list[0].Param = DMA_IP_CH_SET_SOURCE_ADDRESS;
        list[0].Value = Adc_Ip_GetDataAddress(ADC_INSTANCE, 0U);
        list[1].Param = DMA_IP_CH_SET_DESTINATION_ADDRESS;
        list[1].Value = (uint32_t)&adc_dma_buf[0];
        (void)Dma_Ip_SetLogicChannelTransferList(DMA_LOGIC_CH_0, list, 2U);
        (void)Dma_Ip_SetLogicChannelCommand(DMA_LOGIC_CH_0, DMA_IP_CH_SET_HARDWARE_REQUEST);
    }

    /* PE2 (EN) and PE6 (FAULT) high before any PWM pulses */
    Gpio_Dio_Ip_WritePin(PIN_EN_PORT, PIN_EN, 1U);
    Gpio_Dio_Ip_WritePin(PIN_FAULT_PORT, PIN_FAULT, 1U);

    /* 3-phase complementary PWM with dead time */
    Ftm_Pwm_Ip_Init(FTM_INSTANCE, &Ftm_Pwm_Ip_UserCfg0);
    Ftm_Pwm_Ip_UpdatePwmPeriod(FTM_INSTANCE, PWM_PERIOD_TICKS, TRUE);
    Ftm_Pwm_Ip_UpdatePwmChannel(FTM_INSTANCE, PWM_CH_U, PWM_DUTY_TICKS, 0U, TRUE);
    Ftm_Pwm_Ip_UpdatePwmChannel(FTM_INSTANCE, PWM_CH_V, PWM_DUTY_TICKS, 0U, TRUE);
    Ftm_Pwm_Ip_UpdatePwmChannel(FTM_INSTANCE, PWM_CH_W, PWM_DUTY_TICKS, 0U, TRUE);

    /* Start ADC sampling: PDB (software trigger, continuous, 4800 ticks at 48 MHz = same 100 us as the PWM) */
    Pdb_Adc_Ip_Init(PDB_INSTANCE, &PdbHwUnit_0);
    Pdb_Adc_Ip_SwTrigger(PDB_INSTANCE);

    for(;;)
    {
        /* Results arrive via DMA + AdcDma_Callback; watch adc_dma_buf / dma_done_count in the debugger */
        if (exit_code != 0)
        {
            break;
        }
    }
    return exit_code;
}

/** @} */