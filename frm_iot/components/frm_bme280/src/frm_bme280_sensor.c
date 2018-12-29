/*
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
#include <stdio.h>
#include <string.h>
//#include <mutex>

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "bme280.h"
#include "bme280_defs.h"
#include "frm_config.h"
#include "frm_bme280_sensor.h"

#include "esp_log.h"
static const char *TAG = "FRM_BME280";


#define I2C_EXAMPLE_MASTER_SCL_IO GPIO_NUM_19  /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO GPIO_NUM_18  /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0       /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0    /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0    /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000      /*!< I2C master clock frequency */
#define ACK_CHECK_EN 0x1                       /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                      /*!< I2C master will not check ack from slave */


void i2c_master_init(gpio_num_t sda_io_pin, gpio_num_t scl_io_pin) {
    // this should be executed only once!
    //std::call_once(i2c_init, [](gpio_num_t sda_io_pin, gpio_num_t scl_io_pin){
    i2c_port_t i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_io_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_io_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    //});
};


void frm_bme280_update(const frm_bme280_type *device)
{
    bme280_set_sensor_mode(BME280_FORCED_MODE, device->dev);
    device->dev->delay_ms(40);  // Wait for the measurement to complete and print data
    bme280_get_sensor_data(BME280_ALL, device->data, device->dev);
};


void frm_bme280_sprintf(const frm_bme280_type *device, char* payload)
{
    // convenience function to write the sensor values to CSV
    sprintf(payload,
        "%s,temperature,%0.2f\n"
        "%s,pressure,%0.2f\n"
        "%s,humidity,%0.2f\n",
        device->name, device->data->temperature, 
        device->name, device->data->pressure / 100.0, 
        device->name, device->data->humidity
    );
};


double frm_bme280_get_temperature(const frm_bme280_type *device)
{
    return device->data->temperature;
};


double frm_bme280_get_pressure(const frm_bme280_type *device)
{
    return device->data->pressure / 100.0;
};


double frm_bme280_get_humidity(const frm_bme280_type *device)
{
    return device->data->humidity;
};


// init, read, and write functionality from here:
// https://github.com/akbarhash/esp32/tree/master/bme280
static esp_err_t XI2CWrite(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t *data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_add << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t XI2CRead(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t *data_rd, size_t size) {
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_add << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1)
        i2c_master_read(cmd, data_rd, size - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data_rd + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t X_WriteMulti(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t index, uint32_t count, uint8_t *data_wr) {
    uint8_t XI2CBuffer[20];
    XI2CBuffer[0] = index;
    memcpy(&XI2CBuffer[1], data_wr, count);
    esp_err_t ret = XI2CWrite(i2c_num, i2c_add, XI2CBuffer, count + 1);
    return ret;
}

esp_err_t X_ReadMulti(i2c_port_t i2c_num, uint8_t i2c_add, uint8_t index, uint32_t count, uint8_t *data_rd) {
    esp_err_t ret;
    ret = XI2CWrite(i2c_num, i2c_add, &index, 0x1);
    if (ret == ESP_FAIL)
        return ret;
    ret = XI2CRead(i2c_num, i2c_add, data_rd, count);
    return ret;
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    /* Return 0 for Success, non-zero for failure */
    esp_err_t ret = X_ReadMulti(I2C_EXAMPLE_MASTER_NUM, dev_id, reg_addr, len, reg_data);
    vTaskDelay( 10 / portTICK_PERIOD_MS);
    return ret;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    /* Return 0 for Success, non-zero for failure */
    esp_err_t ret = X_WriteMulti(I2C_EXAMPLE_MASTER_NUM, dev_id, reg_addr, len, reg_data);
    vTaskDelay( 10 / portTICK_PERIOD_MS);
    return ret;
}

void user_delay_ms(uint32_t period) {
    vTaskDelay( period / portTICK_PERIOD_MS);
}


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

    // bme280 initialization and settings
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

    //struct bme280_dev dev2 = dev1;
    //dev2.dev_id = BME280_I2C_ADDR_SEC;  // sensor2

    //ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    ESP_LOGI(TAG, "BME-280 '%s' initialization", device->name);
    rslt = bme280_init(device->dev);
    if(rslt != 0) {
        ESP_LOGE(TAG, "BME-280 '%s' init or setting error. code: %d", device->name, rslt);
        abort();
    };
    //rslt = bme280_init(&dev2);
    //if(rslt != SUCCESS) {
    //    ESP_LOGE(TAG, "BME-280 sensor2 init or setting error. code: %d", rslt);
    //    abort();
    //}

    //ESP_LOGI(TAG, "BME-280 settings");
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, device->dev);
    if(rslt != 0) {
        ESP_LOGE(TAG, "BME-280 '%s' setting error. code: %d", device->name, rslt);
        abort();
    };
    //rslt = bme280_set_sensor_settings(settings_sel, &dev2);
    //if(rslt != SUCCESS) {
    //    ESP_LOGE(TAG, "BME-280 sensor2 setting error. code: %d", rslt);
    //    abort();
    //}
    return device;
};
