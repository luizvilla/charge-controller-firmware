/*
 * Copyright (c) 2020 LAAS-CNRS
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
/**
 * @defgroup    owntech_modules_leg OwnTech's leg module
 * @ingroup     owntech_modules
 * @brief       OwnTech PWM management layer by inverter leg
 *
 * @{
 * @file
 * @brief   PWM management layer by inverter leg interface definitions
 * @date    2020
 * @author  Hugues Larrive <hugues.larrive@laas.fr>
 */

#ifndef LEG_H
#define LEG_H

#if MODULE_OT_PWM   /* RIOT */
#include "../ot_pwm/ot_pwm.h"
#endif

#if MODULE_HRTIM    /* RIOT */
#include "../hrtim/hrtim.h"
#endif

#ifdef  __ZEPHYR__
#include "hrtim.h"
#else   /* RIOT */
#include "periph/gpio.h"
#endif

#define LEG_DEFAULT_DT  (100U)
#define LEG_RES         (23040U)
#define LEG_FREQ        KHZ(200U)
//~ #define MIN_DC          (LEG_RES / 10)
//~ #define MAX_DC          (LEG_RES - (LEG_RES / 10))
/*
#define CCMR_COMP       (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | \
                         TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | \
                         TIM_CCMR1_OC2M_2)
*/
/**
 * @brief   Inverter leg configuration data structure
 */
typedef struct {
    hrtim_t pwm;            /**< PWM device */
    uint8_t pos_chan;       /**< PWM device channel for the positive sw */
    uint8_t neg_chan;       /**< PWM device channel for the negative sw */
    HRTIM_TypeDef *dev;     /**< Low level timer used */
    uint16_t dead_time;     /**< Dead time */
    uint16_t duty_cycle;    /**< Duty cycle */
} leg_conf_t;

/**
 * @brief   Initializes all the configured devices
 */
void leg_init(void);

/**
 * @brief   Set the PWM duty cycle for a given leg device
 *
 * @param[in]   leg_dev         leg device from 1 to 10
 * @param[in]   duty_cycle      duty cycle to set from 0.0 to 1
 * @param[in]   phase_shift     phase shift from 0.0 to 1 period
 */
void leg_set(uint8_t leg_dev, float duty_cycle, float phase_shift);

/**
 * @brief   Set the PWM duty cycle for the high side switch only
 *          allowing current only from high to low.
 *
 * @param[in]   leg_dev     leg device from 1 to 10
 * @param[in]   duty_cycle  duty cycle to set
 */
//~ void buck_set(uint8_t leg_dev, uint16_t duty_cycle);

/**
 * @brief   Stop the leg (its 2 outputs goes low)
 *
 * @param[in]   leg_dev      leg device from 1 to 10
 */
void leg_stop(uint8_t leg_dev);

uint8_t leg_numof(void);

#endif /* LEG_H */
/** @} */
