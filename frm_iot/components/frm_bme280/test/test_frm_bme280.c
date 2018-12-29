/**
 * Copyright 2018 Fink Labs GmbH
 * Additions Copyright 2018 Espressif Systems (Shanghai) PTE LTD
 * Additions Copyright 2017 Project Iota, Drasko Draskovic (https://github.com/drasko)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "frm_bme280_sensor.h"


TEST_CASE("test frm_bme280_sprintf", "[frm_bme280]")
{
    struct bme280_data comp_data;
    comp_data = {
        .temperature = 100;
        .pressure = 20000;
        .humidity = 300;
    };
    frm_bme280_type device;
    device.name = "stable\0";
    device.data = comp_data;
    char payload[100];

    frm_bme280_sprintf(&device, payload);
    TEST_ASSERT_EQUAL_STRING("sla\n", payload);
}
