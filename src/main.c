/*
 * Copyright (c) 2021 LAAS-CNRS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief       Test for OwnTech's hrtim driver integration in
 *              half_bridge.cpp
 *
 * @author      Hugues Larrive <hugues.larrive@laas.fr>
 */

#include <zephyr.h>
#include <stdio.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "half_bridge.h"        // PWM generation for DC/DC converter


void main(void)
{
	printf("\
Libre Solar Charge Controller: %s\
\n", CONFIG_BOARD);

    puts("\n\
 ______________________________________________________________________\n\
|                                                                      |\n\
|              Test for OwnTech's high resolution timer                |\n\
|                driver integration in half_bridge.cpp                 |\n\
|______________________________________________________________________|\n\
\n");

    puts("\
//  void half_bridge_init(  print freq_kHz, int deadtime_ns,\n\
//                          float min_duty, float max_duty  );\n\
    half_bridge_init(200000, 100, 0.1, 0.9);\n");
    half_bridge_init(200000, 100, 0.1, 0.9);

    puts("\
//  void half_bridge_set_duty_cycle(float duty);\n\
    half_bridge_set_duty_cycle(0.5);\n");
    half_bridge_set_duty_cycle(0.5);

    puts("\
//  void half_bridge_start();\n\
    half_bridge_start();\n");
    half_bridge_start();

    puts("*** the end ***\n");
}
