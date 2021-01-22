/*
 * Copyright (c) 2021 LAAS-CNRS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief       Test for owntech's leg module integration in zephyr
 *
 * @author      Hugues Larrive <hugues.larrive@laas.fr>
 */

#include <zephyr.h>
#include <stdio.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "leg.h"        // PWM management layer by inverter leg interface definitions


void main(void)
{
	printf("\
CONFIG_BOARD: %s\\n", CONFIG_BOARD);

    puts("\n\
 _______________________________________________________________\n\
|                                                               |\n\
|      Test for owntech's leg module integration in zephyr      |\n\
|_______________________________________________________________|\n\
\n\
\n\
See src/leg_conf.h for legs configuration\n\
TODO: fix leg_conf_t data structure for hrtim\n\
\n");



    puts("\
//  void leg_init(void);\n\
    leg_init();\n");
    leg_init();

    puts("\
//  void leg_set(uint8_t leg_dev, float duty_cycle,\n\
//                  float phase_shift);\n\
//  leg_set(1, 0.5, 0);\n\
//  leg_set(2, 0.5, 0.5);\n");
    leg_set(1, 0.5, 0);
    leg_set(2, 0.5, 0.5);

    puts("\tk_msleep(10000); // sleep for 10 seconds\n");
    k_msleep(10000);

    puts("\
//  void leg_stop(uint8_t leg_dev);\n\
//  leg_stop(2);\n");
    leg_stop(2);

    puts("\tk_msleep(10000); // sleep for 10 seconds\n");
    k_msleep(10000);

    puts("\
//  leg_set(2, 0.5, 0.5);\n");
    leg_set(2, 0.5, 0.5);

    //~ puts("*** the end ***\n");
}
