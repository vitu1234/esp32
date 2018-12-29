/**
 * Copyright 2018 Fink Labs GmbH
 * Additions Copyright 2018 Espressif Systems (Shanghai) PTE LTD
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

/**
 * @file bme280_sensor.h
 * @brief bme280 sensor component
 */
#include "frm_config"
#include "bme280_defs.h"

#ifndef FRM_BME280_SENSOR_H
#define FRM_BME280_SENSOR_H


/*!
 * @brief FRM_BME280_t type represents a bme280 device.
 *
 */
typedef struct {
    char name[16];
    struct bme280_dev *dev;
    struct bme280_data *data;
} frm_bme280_type;


/*!
 * @brief Create new device instance and initialize.
 *
 * @param argc - arg count.
 * @param argv - arg values: first argv value is the device name.
 *
 * @return FRM_BME280_t.
 */
int frm_bme280_init(int argc, frm_params_type argv);

/*!
 * @brief Update task typically comprises of reading values from the device and storing them in FRM_DME280_t.
 */
void frm_bme280_update(const frm_bme280_type *device);

/*!
 * @brief Update task typically comprises of reading values from the device and storing them in FRM_DME280_t.
 *
 * @param payload - buffer to use for sprintf.
 */
void frm_bme280_sprintf(const frm_bme280_type *device, char* payload);

/*!
 * @brief get the temperature value (first you need to run an update!).
 *
 * @return temperature value.
 */
double frm_bme280_get_temperature(const frm_bme280_type *device);

/*!
 * @brief get the pressure value (first you need to run an update!).
 *
 * @return pressure value.
 */
double frm_bme280_get_pressure(const frm_bme280_type *device);

/*!
 * @brief get the humidity value (first you need to run an update!).
 *
 * @return humidity value.
 */
double frm_bme280_get_humidity(const frm_bme280_type *device);

#endif /* FRM_BME280_SENSOR_H */
