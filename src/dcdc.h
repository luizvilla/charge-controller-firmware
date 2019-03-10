/* LibreSolar charge controller firmware
 * Copyright (c) 2016-2019 Martin Jäger (www.libre.solar)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DCDC_H
#define DCDC_H

/** @file
 *
 * @brief DC/DC buck/boost control functions
 */

#include <stdbool.h>
#include "battery.h"
#include "power_port.h"

/** DC/DC basic operation mode
 *
 * Defines which type of device is connected to the high side and low side ports
 */
enum dcdc_control_mode
{
    MODE_MPPT_BUCK,     ///< solar panel at high side port, battery / load at low side port (typical MPPT)
    MODE_MPPT_BOOST,    ///< battery at high side port, solar panel at low side (e.g. e-bike charging)
    MODE_NANOGRID       ///< accept input power (if available and need for charging) or provide output power
                        ///< (if no other power source on the grid and battery charged) on the high side port
                        ///< and dis/charge battery on the low side port, battery voltage must be lower than
                        ///< nano grid voltage.
};

/** DC/DC type
 *
 * Contains all data belonging to the DC/DC sub-component of the PCB, incl.
 * actual measurements and calibration parameters.
 */
typedef struct {
    dcdc_control_mode mode;

    // actual measurements
    float ls_current;           ///< Low-side (inductor) current
    float temp_mosfets;

    // current state
    float power;                ///< Power at low-side (calculated by dcdc controller)
    int pwm_delta;              ///< Direction of PWM change for MPPT
    int off_timestamp;          ///< Time when DC/DC was switched off last time

    // maximum allowed values
    float ls_current_max;       ///< Maximum low-side (inductor) current
    float ls_current_min;       ///< Minimum low-side current (if lower, charger is switched off)
    float hs_voltage_max;
    float ls_voltage_max;

    // calibration parameters
    //float offset_voltage_start;  // V  charging switched on if Vsolar > Vbat + offset
    //float offset_voltage_stop;   // V  charging switched off if Vsolar < Vbat + offset
    int restart_interval;       ///< Restart interval (s): When should we retry to start charging after low solar power cut-off?
} dcdc_t;


/** Initialize DC/DC and DC/DC port structs
 *
 * See http://libre.solar/docs/dcdc_control for detailed information
 *
 * @param dcdc DC/DC type description
 */
void dcdc_init(dcdc_t *dcdc);

/** Main control function for the DC/DC converter
 *
 * @param dcdc DC/DC type description
 * @param high_side High-side power port (e.g. solar input for typical MPPT charge controller application)
 * @param low_side  Low-side power port (e.g. battery output for typical MPPT charge controller application)
 */
void dcdc_control(dcdc_t *dcdc, power_port_t *high_side, power_port_t *low_side);

#endif /* DCDC_H */
