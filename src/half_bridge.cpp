/*
 * Copyright (c) 2016 Martin Jäger / Libre Solar
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "half_bridge.h"

#include <zephyr.h>

#include <stdio.h>

#include "mcu.h"

#ifdef CONFIG_SOC_SERIES_STM32F3X
#include "hrtim.h"
#endif

#ifndef UNIT_TEST
#include <soc.h>
#include <pinmux/stm32/pinmux_stm32.h>

#define DT_DRV_COMPAT st_stm32_timers
#endif

#if DT_NODE_EXISTS(DT_PATH(dcdc))

static uint16_t tim_ccr_min;        // capture/compare register min/max
static uint16_t tim_ccr_max;
#ifndef CONFIG_SOC_SERIES_STM32F3X
static uint16_t tim_dt_clocks = 0;
#else
static hrtim_t hrtim = 0;
static uint16_t dead_time_ns = 0;
static uint16_t hrtim_period = 0;
static uint16_t hrtim_cmp = 0;
static bool hrtim_out_enabled = 0;
#endif

static uint16_t clamp_ccr(uint16_t ccr_target)
{
    // protection against wrong settings which could destroy the hardware
    if (ccr_target <= tim_ccr_min) {
        return tim_ccr_min;
    }
    else if (ccr_target >= tim_ccr_max) {
        return tim_ccr_max;
    }
    else {
        return ccr_target;
    }
}

#ifndef UNIT_TEST

#ifndef CONFIG_SOC_SERIES_STM32F3X
// Get address of used timer from board dts
#define PWM_TIMER_ADDR      DT_REG_ADDR(DT_PHANDLE(DT_PATH(dcdc), timer))
#else
#define PWM_TIMER_ADDR      HRTIM1_BASE
#endif

#if PWM_TIMER_ADDR == TIM3_BASE

static const struct soc_gpio_pinctrl tim_pinctrl[] = ST_STM32_DT_PINCTRL(timers3, 0);

static void tim_init_registers(int freq_kHz)
{
    stm32_dt_pinctrl_configure(tim_pinctrl, ARRAY_SIZE(tim_pinctrl), PWM_TIMER_ADDR);

    // Enable TIM3 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // No prescaler --> timer frequency == SystemClock
    TIM3->PSC = 0;

    // Capture/Compare Mode Register 1
    // OCxM = 110: Select PWM mode 1 on OCx
    // OCxPE = 1:  Enable preload register on OCx (reset value)
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;

    // Capture/Compare Enable Register
    // CCxP: Active high polarity on OCx (default = 0)
    TIM3->CCER &= ~(TIM_CCER_CC3P); // PB0 / TIM3_CH3: high-side
    TIM3->CCER |= TIM_CCER_CC4P;    // PB1 / TIM3_CH4: low-side

    // Control Register 1
    // TIM_CR1_CMS = 01: Select center-aligned mode 1
    // TIM_CR1_CEN =  1: Counter enable
    TIM3->CR1 |= TIM_CR1_CMS_0 | TIM_CR1_CEN;

    // Force update generation (UG = 1)
    TIM3->EGR |= TIM_EGR_UG;

    // Auto Reload Register
    // center-aligned mode counts up and down in each period, so we need half the clocks as
    // resolution for the given frequency.
    TIM3->ARR = SystemCoreClock / (freq_kHz * 1000) / 2;;
}

void half_bridge_start()
{
    if (TIM3->CCR3 != 0U) {
        // Capture/Compare Enable Register
        // CCxE = 1: Enable the output on OCx
        // CCxP = 0: Active high polarity on OCx (default)
        TIM3->CCER |= TIM_CCER_CC3E;
        TIM3->CCER |= TIM_CCER_CC4E;
    }
}

void half_bridge_stop()
{
    TIM3->CCER &= ~(TIM_CCER_CC3E);
    TIM3->CCER &= ~(TIM_CCER_CC4E);
}

uint16_t half_bridge_get_arr()
{
    return TIM3->ARR;
}

uint16_t half_bridge_get_ccr()
{
    return TIM3->CCR3;
}

void half_bridge_set_ccr(uint16_t ccr)
{
    uint16_t ccr_clamped = clamp_ccr(ccr);

    TIM3->CCR3 = ccr_clamped;                   // high-side
    TIM3->CCR4 = ccr_clamped + tim_dt_clocks;   // low-side
}

bool half_bridge_enabled()
{
    return TIM3->CCER & TIM_CCER_CC3E;
}

#elif PWM_TIMER_ADDR == TIM1_BASE

static const struct soc_gpio_pinctrl tim_pinctrl[] = ST_STM32_DT_PINCTRL(timers1, 0);

static void tim_init_registers(int freq_kHz)
{
    stm32_dt_pinctrl_configure(tim_pinctrl, ARRAY_SIZE(tim_pinctrl), PWM_TIMER_ADDR);

    // Enable TIM1 clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // No prescaler --> timer frequency == SystemClock
    TIM1->PSC = 0;

    // Capture/Compare Mode Register 1
    // OC1M = 110: Select PWM mode 1 on OC1
    // OC1PE = 1:  Enable preload register on OC1 (reset value)
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;

    // Capture/Compare Enable Register
    // CC1E = 1: Enable the output on OC1
    // CC1P = 0: Active high polarity on OC1 (default)
    // CC1NE = 1: Enable the output on OC1N
    // CC1NP = 0: Active high polarity on OC1N (default)
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;

    // Control Register 1
    // TIM_CR1_CMS = 00: Select edge-aligned mode
    // TIM_CR1_CEN =  1: Counter enable
    TIM1->CR1 |= TIM_CR1_CEN;

#ifdef CONFIG_SOC_SERIES_STM32G4X
    // Boards with STM32G4 MCU use ADC2 for synchronized ADC sampling. We use OC6
    // for triggering, but but don't configure any pin for PWM mode
    TIM1->CCMR3 |= TIM_CCMR3_OC6M_2 | TIM_CCMR3_OC6M_1 | TIM_CCMR3_OC6PE;
    TIM1->CCER |= TIM_CCER_CC6E;
    // Trigger ADC via TIM1_TRGO2 on OC6ref falling edge signal event
    TIM1->CR2 |= TIM_CR2_MMS2_3 | TIM_CR2_MMS2_2 |  TIM_CR2_MMS2_0;
#endif

    // Force update generation (UG = 1)
    TIM1->EGR |= TIM_EGR_UG;

    // Auto Reload Register
    // edge-aligned mode possible with TIM1 to increase resolution (no division by 2 necessary)
    TIM1->ARR = SystemCoreClock / (freq_kHz * 1000);

    // Break and Dead-Time Register
    // DTG[7:0]: Dead-time generator setup
    TIM1->BDTR |= (tim_dt_clocks & (uint32_t)0x7F); // ensure that only the last 7 bits are changed

    // Lock Break and Dead-Time Register
    // TODO: does not work properly... maybe HW bug?
    TIM1->BDTR |= TIM_BDTR_LOCK_1 | TIM_BDTR_LOCK_0;
}

void half_bridge_start()
{
    if (TIM1->CCR1 != 0U) {
        // Break and Dead-Time Register
        // MOE  = 1: Main output enable
        TIM1->BDTR |= TIM_BDTR_MOE;
    }
}

void half_bridge_stop()
{
    // Break and Dead-Time Register
    // MOE  = 1: Main output enable
    TIM1->BDTR &= ~(TIM_BDTR_MOE);
}

uint16_t half_bridge_get_arr()
{
    return TIM1->ARR;
}

uint16_t half_bridge_get_ccr()
{
    return TIM1->CCR1;
}

void half_bridge_set_ccr(uint16_t ccr)
{
    TIM1->CCR1 = clamp_ccr(ccr);

#ifdef CONFIG_SOC_SERIES_STM32G4X
    // Trigger ADC for current measurement in the middle of the cycle.
    TIM1->CCR6 = TIM1->CCR1 / 2;
#endif
}

bool half_bridge_enabled()
{
    return TIM1->BDTR & TIM_BDTR_MOE;
}

#elif PWM_TIMER_ADDR == HRTIM1_BASE

//~ static const struct soc_gpio_pinctrl tim_pinctrl[] = ST_STM32_DT_PINCTRL(timers1, 0);

static void tim_init_registers(int freq_kHz)
{
    //~ stm32_dt_pinctrl_configure(tim_pinctrl, ARRAY_SIZE(tim_pinctrl), PWM_TIMER_ADDR);
    /**
     * HRTIM have a master timer and 5 slave timing units (tu) with two
     * outputs each. Pinout of f334r8 and f072rb are the same. PWM_HS is
     * on PA8 (TIMA OUT1) and PWM_LS is on PB13 (TIMC OUT2).
     */
    uint32_t freq = freq_kHz * 1000;
    uint16_t dead_time_ns = 0;
    hrtim_cen_t cen = (hrtim_cen_t)(TACEN | TCCEN);

    /* Initializes the master timer */
    hrtim_period = hrtim_init_master(hrtim, &freq);

    /* Initializes TIMA, set the dead time */
    hrtim_init_tu(hrtim, TIMA, &freq);
    hrtim_pwm_dt(hrtim, TIMA, dead_time_ns);

    /* Initializes TIMC, set the dead time */
    hrtim_init_tu(hrtim, TIMC, &freq);
    hrtim_pwm_dt(hrtim, TIMC, dead_time_ns);

    /* Enable counters */
    hrtim_cnt_en(hrtim, cen);

    /* setup complementary outputs on PA8 and PB13 */
    hrtim_set_cb_set(hrtim, TIMA, OUT1, MSTPER);
    hrtim_rst_cb_set(hrtim, TIMA, OUT1, MSTCMP1);
    hrtim_set_cb_set(hrtim, TIMC, OUT2, MSTCMP1);
    hrtim_rst_cb_set(hrtim, TIMC, OUT2, MSTPER);

    /* reset on master timer period event */
    hrtim_rst_evt_en(hrtim, TIMA, RST_MSTPER);
    hrtim_rst_evt_en(hrtim, TIMC, RST_MSTPER);
}

void half_bridge_start()
{
    if (hrtim_cmp >= tim_ccr_min && hrtim_cmp <= tim_ccr_max) {
        /* As it is unusual to use outputs from two timing units for a
         * single complementary output, we can not use hrtim_out_en()
         * which is able to handle two outputs but unable to handle two
         * timing units at once. */
        uint32_t oenr;

        oenr = (OUT1 << (TIMA * 2));
        oenr |= (OUT2 << (TIMC * 2));

        HRTIM1->sCommonRegs.OENR |= oenr;

        hrtim_out_enabled = 1;
    }
}

void half_bridge_stop()
{
    /* As it is unusual to use outputs from two timing units for a
     * single complementary output, we can not use hrtim_out_dis()
     * which is able to handle two outputs but unable to handle two
     * timing units at once. */
    uint32_t odisr;

    odisr = (OUT1 << (TIMA * 2));
    odisr |= (OUT2 << (TIMC * 2));

    HRTIM1->sCommonRegs.ODISR |= odisr;

    hrtim_out_enabled = 0;
}

uint16_t half_bridge_get_arr()
{
    return hrtim_period;
}

uint16_t half_bridge_get_ccr()
{
    return hrtim_cmp;
}

void half_bridge_set_ccr(uint16_t ccr)
{
    hrtim_cmp = clamp_ccr(ccr);
    hrtim_cmp_set(hrtim, MSTR, MCMP1R, hrtim_cmp);
}

bool half_bridge_enabled()
{
    return hrtim_out_enabled;
}

#endif // TIM1

#else // UNIT_TEST

const uint32_t SystemCoreClock = 24000000;

// dummy registers
uint32_t tim_ccr = 0;
uint32_t tim_arr = 0;
bool pwm_enabled = false;

static void tim_init_registers(int freq_kHz)
{
    // assuming edge-aligned PWM like with TIM1
    tim_arr = SystemCoreClock / (freq_kHz * 1000);
}

void half_bridge_start()
{
    pwm_enabled = true;
}

void half_bridge_stop()
{
    pwm_enabled = false;
}

uint16_t half_bridge_get_arr()
{
    return tim_arr;
}

uint16_t half_bridge_get_ccr()
{
    return tim_ccr;
}

void half_bridge_set_ccr(uint16_t ccr)
{
    tim_ccr = clamp_ccr(ccr);          // high-side
}

bool half_bridge_enabled()
{
    return pwm_enabled;
}

#endif /* UNIT_TEST */

#ifndef CONFIG_SOC_SERIES_STM32F3X
static void tim_calculate_dt_clocks(int deadtime_ns)
{
    // (clocks per ms * deadtime in ns) / 1000 == (clocks per ms * deadtime in ms)
    // although the C operator precedence does the "right thing" to allow deadtime_ns < 1000 to
    // be handled nicely, parentheses make this more explicit
    tim_dt_clocks = ((SystemCoreClock / (1000000)) * deadtime_ns) / 1000;
}
#endif

void half_bridge_init(int freq_kHz, int deadtime_ns, float min_duty, float max_duty)
{
#ifndef CONFIG_SOC_SERIES_STM32F3X
    tim_calculate_dt_clocks(deadtime_ns);
#else
    dead_time_ns = deadtime_ns;
#endif

    tim_init_registers(freq_kHz);

    tim_ccr_min = min_duty * half_bridge_get_arr();
    tim_ccr_max = max_duty * half_bridge_get_arr();

    half_bridge_set_duty_cycle(max_duty);      // init with allowed value
}

void half_bridge_set_duty_cycle(float duty)
{
    if (duty >= 0.0 && duty <= 1.0) {
        half_bridge_set_ccr(half_bridge_get_arr() * duty);
    }
}

float half_bridge_get_duty_cycle()
{
    return (float)(half_bridge_get_ccr()) / half_bridge_get_arr();
}

#endif // DT_NODE_EXISTS(DT_PATH(dcdc))
