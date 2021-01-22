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
 * @file
 * @brief   PWM management layer by inverter leg
 * @date    2020
 * @author  Hugues Larrive <hugues.larrive@laas.fr>
 */

#include "leg.h"
#include "leg_conf.h"

static uint16_t period, min_dc, max_dc;

void leg_init(void)
{
    uint32_t freq = LEG_FREQ;
    period = hrtim_init(0, &freq, LEG_DEFAULT_DT);
    min_dc = period / 10;
    max_dc = period - min_dc;
}

void leg_set(uint8_t leg_dev, float dc, float pha)
{
    leg_dev--;

    uint16_t duty_cycle = dc * period;
    uint16_t phase_shift = period * pha / 360;

    if (duty_cycle < min_dc) {
        duty_cycle = min_dc;
    }
    if (duty_cycle > max_dc) {
        duty_cycle = max_dc;
    }

    hrtim_pwm_set(leg_config[leg_dev].pwm, leg_config[leg_dev].pos_chan,
                    duty_cycle, phase_shift);
    /* save the duty-cycle */
    leg_config[leg_dev].duty_cycle = duty_cycle;
}

void leg_stop(uint8_t leg_dev)
{
    leg_dev--;
    hrtim_pwm_set(leg_config[leg_dev].pwm, leg_config[leg_dev].pos_chan, 0, 0);
}

uint8_t leg_numof(void)
{
    return LEG_NUMOF;
}
