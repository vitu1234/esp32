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

//#include "mock_bme280.h"
/*
TEST_CASE("test frm_bme280_init", "[frm_bme280]")
{
    frm_bme280_type *device;
    frm_params_type argv = {"sensor1", "119"};

    i2c_master_init_Expect(I2C_EXAMPLE_MASTER_SDA_IO, I2C_EXAMPLE_MASTER_SCL_IO);
    bme280_init_ExpectAndReturn(device->dev, 0);
    bme280_set_sensor_settings_ExpectAndReturn(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, device->dev, 0);

    device = frm_bme280_init(2, argv);
    TEST_ASSERT_EQUAL_STRING("sensor1", device->name);
    TEST_ASSERT_EQUAL_INT(119, device->dev->dev_id);
    // TODO assert called
}
*/


/*
int frm_bme280_init(int argc, frm_params_type argv)
{
    int8_t rslt;
    int8_t address;
    uint8_t settings_sel;
    frm_params_type params = {{0}};  // we do not need this one!
    frm_bme280_type *device = malloc(sizeof(frm_bme280_type));

    assert(argc == 2);
    memcpy(params, argv, sizeof(frm_params_type));
    memcpy(device->name, params[0], sizeof(params[0]));
    //address = atoi(params[1]);  // TODO
    address = 119;

    i2c_master_init(I2C_EXAMPLE_MASTER_SDA_IO, I2C_EXAMPLE_MASTER_SCL_IO);

    // fill in bme280_dev structure
    device->dev->dev_id = address;
    device->dev->intf = BME280_I2C_INTF;
    device->dev->read = user_i2c_read;
    device->dev->write = user_i2c_write;
    device->dev->delay_ms = user_delay_ms;
    // mode of operation for weather data
    device->dev->settings.osr_h = BME280_OVERSAMPLING_2X;
    device->dev->settings.osr_p = BME280_OVERSAMPLING_4X;
    device->dev->settings.osr_t = BME280_OVERSAMPLING_4X;
    device->dev->settings.filter = BME280_FILTER_COEFF_2;

    ESP_LOGI(TAG, "BME-280 '%s' initialization", device->name);
    rslt = bme280_init(device->dev);
    if(rslt != 0) {
        ESP_LOGE(TAG, "BME-280 '%s' initialization error. code: %d", device->name, rslt);
        abort();
    };

    ESP_LOGI(TAG, "BME-280 '%s' settings", device->name);
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, device->dev);
    if(rslt != 0) {
        ESP_LOGE(TAG, "BME-280 '%s' setting error. code: %d", device->name, rslt);
        abort();
    };

    return device;
};
*/


TEST_CASE("test frm_bme280_sprintf", "[frm_bme280]")
{
    struct bme280_data comp_data = {
        .temperature = 100.0,
        .pressure = 20000.0,
        .humidity = 300.0,
    };
    frm_bme280_type device;
    strcpy(device.name, "sensor1\0");
    device.data = &comp_data;
    char payload[100];

    frm_bme280_sprintf(&device, payload);
    TEST_ASSERT_EQUAL_STRING("sensor1,temperature,100.00\nsensor1,pressure,200.00\nsensor1,humidity,300.00\n", payload);
}

TEST_CASE("test frm_bme280_get_temperature", "[frm_bme280]")
{
    struct bme280_data comp_data = {
        .temperature = 100.0,
        .pressure = 20000.0,
        .humidity = 300.0,
    };
    frm_bme280_type device;
    strcpy(device.name, "sensor1\0");
    device.data = &comp_data;
    
    TEST_ASSERT_EQUAL_DOUBLE(100.0, frm_bme280_get_temperature(&device));
}

TEST_CASE("test frm_bme280_get_pressure", "[frm_bme280]")
{
    struct bme280_data comp_data = {
        .temperature = 100.0,
        .pressure = 20000.0,
        .humidity = 300.0,
    };
    frm_bme280_type device;
    strcpy(device.name, "sensor1\0");
    device.data = &comp_data;
    
    TEST_ASSERT_EQUAL_DOUBLE(200.0, frm_bme280_get_pressure(&device));
}

TEST_CASE("test frm_bme280_get_humidity", "[frm_bme280]")
{
    struct bme280_data comp_data = {
        .temperature = 100.0,
        .pressure = 20000.0,
        .humidity = 300.0,
    };
    frm_bme280_type device;
    strcpy(device.name, "sensor1\0");
    device.data = &comp_data;
    
    TEST_ASSERT_EQUAL_DOUBLE(300.0, frm_bme280_get_humidity(&device));
}

