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
#include "driver/i2c.h"

#include "unity.h"
#include "frm_bme280_sensor.h"

#include "mocks/mock_bme280.h"
#include "mocks/mock_i2c.h"


TEST_CASE("test frm_bme280_init", "[frm_bme280]")
{
    frm_bme280_type *device;
    frm_params_type argv = {"sensor1", "119"};

    //i2c_master_init_Expect(I2C_EXAMPLE_MASTER_SDA_IO, I2C_EXAMPLE_MASTER_SCL_IO);
    i2c_param_config_ExpectAnyArgsAndReturn(0);
    i2c_driver_install_ExpectAnyArgsAndReturn(0);

    bme280_init_ExpectAnyArgsAndReturn(0);
    bme280_set_sensor_settings_ExpectAndReturn(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, NULL, 0);
    bme280_set_sensor_settings_IgnoreArg_dev();

    device = frm_bme280_init(2, argv);
    TEST_ASSERT_EQUAL_STRING("sensor1", device->name);
    TEST_ASSERT_EQUAL_INT(119, device->dev.dev_id);

    frm_bme280_destroy(device);
}

TEST_CASE("test frm_bme280_sprintf", "[frm_bme280]")
{
    //struct bme280_data comp_data = {
    //    .temperature = 100.0,
    //    .pressure = 20000.0,
    //    .humidity = 300.0,
    //};
    frm_bme280_type device;
    strcpy(device.name, "sensor1\0");
    device.data.temperature = 100.0;
    device.data.pressure = 20000.0;
    device.data.humidity = 300.0;
    char payload[100];

    frm_bme280_sprintf(&device, payload);
    TEST_ASSERT_EQUAL_STRING("sensor1,temperature,100.00\nsensor1,pressure,200.00\nsensor1,humidity,300.00\n", payload);
}

TEST_CASE("test frm_bme280_get_temperature", "[frm_bme280]")
{
    frm_bme280_type device;
    strcpy(device.name, "sensor1\0");
    device.data.temperature = 100.0;
    device.data.pressure = 20000.0;
    device.data.humidity = 300.0;
    
    TEST_ASSERT_EQUAL_DOUBLE(100.0, frm_bme280_get_temperature(&device));
}

TEST_CASE("test frm_bme280_get_pressure", "[frm_bme280]")
{
    frm_bme280_type device;
    strcpy(device.name, "sensor1\0");
    device.data.temperature = 100.0;
    device.data.pressure = 20000.0;
    device.data.humidity = 300.0;
    
    TEST_ASSERT_EQUAL_DOUBLE(200.0, frm_bme280_get_pressure(&device));
}

TEST_CASE("test frm_bme280_get_humidity", "[frm_bme280]")
{
    frm_bme280_type device;
    strcpy(device.name, "sensor1\0");
    device.data.temperature = 100.0;
    device.data.pressure = 20000.0;
    device.data.humidity = 300.0;
    
    TEST_ASSERT_EQUAL_DOUBLE(300.0, frm_bme280_get_humidity(&device));
}
