/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_wifi.h"

#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "bme280.h"
#include "bme280_defs.h"

#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "cJSON.h"

#include <stdint.h>

#include "esp_log.h"
static const char *TAG = "ESP_IOT";
static const char *PROJECT_VERSION = "0.0.3";

// aws_iot related code
/* The example uses simple WiFi configuration.
 * You can WiFi information and MQTT hostname via 'make menuconfig'.
 */
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

/* CA Root certificate, device ("Thing") certificate and device
 * ("Thing") key.

   Example can be configured one of two ways:

   "Embedded Certs" are loaded from files in "certs/" and embedded into the app binary.

   "Filesystem Certs" are loaded from the filesystem (SD card, etc.)

   See example README for more details.
*/
#if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)
extern const uint8_t aws_s3_server_pem_start[] asm("_binary_aws_s3_server_pem_start");
extern const uint8_t aws_s3_server_pem_end[] asm("_binary_aws_s3_server_pem_end");
extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");
#elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)
static const char * S3_SERVER_PATH = CONFIG_EXAMPLE_S3_SERVER_PATH;
static const char * DEVICE_CERTIFICATE_PATH = CONFIG_EXAMPLE_CERTIFICATE_PATH;
static const char * DEVICE_PRIVATE_KEY_PATH = CONFIG_EXAMPLE_PRIVATE_KEY_PATH;
static const char * ROOT_CA_PATH = CONFIG_EXAMPLE_ROOT_CA_PATH;
#else
#error "Invalid method for loading certs"
#endif

/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
char HostAddress[255] = AWS_IOT_MQTT_HOST;

/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
uint32_t port = AWS_IOT_MQTT_PORT;


static esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            /* This is a workaround as ESP32 WiFi libs don't currently
               auto-reassociate. */
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled. Attempting reconnect now...");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}

// relais
#define GPIO_OUTPUT_PIN_RELAIS 22           /*!< gpio number for relais shield */
#define RH_ON 45.0
#define RH_OFF 50.0

// bme 280 related code
#define I2C_EXAMPLE_MASTER_SCL_IO 19        /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO 18        /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define ACK_CHECK_EN 0x1                    /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                   /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                         /*!< I2C ack value */
#define NACK_VAL 0x1                        /*!< I2C nack value */


// init, read, and write from here:
// https://github.com/akbarhash/esp32/tree/master/bme280
void i2c_master_init() {
    uint8_t i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

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
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
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


// http and mqtt code
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    ESP_LOGI(TAG, "http event handler....");
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}


void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    esp_err_t rc;
    ESP_LOGI(TAG, "processing message from update topic...");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);

    char *update_url = NULL;
    const cJSON *url = NULL;
    cJSON *root = cJSON_Parse((char *)params->payload);
    if (root == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        ESP_LOGE(TAG, "error parsing payload: %s", error_ptr);
        goto end;
    }
    url = cJSON_GetObjectItemCaseSensitive(root, "update_url");
    if (cJSON_IsString(url) && (url->valuestring != NULL)) {
        update_url = strdup(url->valuestring);
        // TODO check url format (https, ...)!
    } else {
        ESP_LOGE(TAG, "\"update_url\" missing in payload");
        goto end;
    }
    ESP_LOGI(TAG, "update_url: %s", url->valuestring);

    esp_http_client_config_t http_config = {0};
    http_config.event_handler = _http_event_handler;

#if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)
    http_config.cert_pem = (const char *)aws_s3_server_pem_start;
#elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)
    http_config.cert_pem = S3_SERVER_PATH;
#endif

    // download the binary from the presigned_url and run the update
    http_config.url = update_url;
    rc = esp_https_ota(&http_config);
    if (rc == ESP_OK) {
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed code: %d", rc);
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    end:
    cJSON_Delete(root);
}


bool allowed_to_switch_on( TickType_t * const pxPreviousWakeTime, const TickType_t xTimeIncrement ) {
    // utility to avoid component wear
    // based on vTaskDelayUntil - we need a similar behaviour for this check without sleeping
    TickType_t xTimeToStart;
    const TickType_t xConstTickCount = xTaskGetTickCount();
    xTimeToStart = *pxPreviousWakeTime + xTimeIncrement;

    if (xConstTickCount < *pxPreviousWakeTime) {
        /* The tick count has overflowed since this function was
        lasted called.  In this case the only time we should ever
        actually delay is if the wake time has also	overflowed,
        and the wake time is greater than the tick time.  When this
        is the case it is as if neither time had overflowed. */
        if ((xTimeToStart < *pxPreviousWakeTime) &&
            (xTimeToStart > xConstTickCount)) {
            //xShouldDelay = pdTRUE;
            return true;
        }
    } else {
        /* The tick time has not overflowed.  In this case we will
        delay if either the wake time has overflowed, and/or the
        tick time is less than the wake time. */
        if ((xTimeToStart < *pxPreviousWakeTime) ||
            (xTimeToStart > xConstTickCount)) {
            // xShouldDelay = pdTRUE;
            return true;
        }
    }
    ESP_LOGI(TAG, "not allowed to switch on yet");
    return false;
}


void aws_iot_task(void *param) {
    char *aws_iot_client_id = (char*) param;
    int8_t rslt;
    uint8_t settings_sel;
    struct bme280_data comp_data;
    bool relais_on = false;
    TickType_t last_used_tick = 0;

    ESP_LOGI(TAG, "App Version %s", PROJECT_VERSION);
    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    // bme280 initialization and settings
    struct bme280_dev dev1 = {
            .dev_id = BME280_I2C_ADDR_PRIM,  // sensor1
            .intf = BME280_I2C_INTF,
            .read = user_i2c_read,
            .write = user_i2c_write,
            .delay_ms = user_delay_ms,
            .settings = { // mode of operation for weather data
                    .osr_h = BME280_OVERSAMPLING_2X,
                    .osr_p = BME280_OVERSAMPLING_4X,
                    .osr_t = BME280_OVERSAMPLING_4X,
                    .filter = BME280_FILTER_COEFF_2
            }
    };

    gpio_set_direction(GPIO_OUTPUT_PIN_RELAIS, GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "BME-280 initialization");
    rslt = bme280_init(&dev1);
    if(rslt != SUCCESS) {
        ESP_LOGE(TAG, "BME-280 sensor1 init or setting error. code: %d", rslt);
        abort();
    }

    ESP_LOGI(TAG, "BME-280 settings");
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, &dev1);
    if(rslt != SUCCESS) {
        ESP_LOGE(TAG, "BME-280 sensor1 setting error. code: %d", rslt);
        abort();
    }

    // MQTT initialization and settings
    IoT_Error_t rc = FAILURE;
    AWS_IoT_Client client;

    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;

#if defined(CONFIG_EXAMPLE_EMBEDDED_CERTS)
    mqttInitParams.pRootCALocation = (const char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (const char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (const char *)private_pem_key_start;

#elif defined(CONFIG_EXAMPLE_FILESYSTEM_CERTS)
    mqttInitParams.pRootCALocation = ROOT_CA_PATH;
    mqttInitParams.pDeviceCertLocation = DEVICE_CERTIFICATE_PATH;
    mqttInitParams.pDevicePrivateKeyLocation = DEVICE_PRIVATE_KEY_PATH;
#endif

    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 5000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;

#ifdef CONFIG_EXAMPLE_SDCARD_CERTS
    ESP_LOGI(TAG, "Mounting SD card...");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3,
    };
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
        abort();
    }
#endif

    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    // Wait for WiFI to show as connected
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;
    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    connectParams.pClientID = aws_iot_client_id;
    connectParams.clientIDLen = (uint16_t) strlen(aws_iot_client_id);
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS IOT...");
    do {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    } while(SUCCESS != rc);

    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    char payload[150];
    char topic[30];
    char update_topic[30];
    IoT_Publish_Message_Params params;
    params.qos = QOS1;
    params.payload = (void *) payload;
    params.isRetained = 0;

    // subscribe to devices update topic
    sprintf(update_topic, "farm/%s/update", aws_iot_client_id);

    ESP_LOGI(TAG, "Subscribing to %s", update_topic);
    rc = aws_iot_mqtt_subscribe(&client, update_topic, strlen(update_topic), QOS0, iot_subscribe_callback_handler, NULL);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Error subscribing to %s : %d ", topic, rc);
        abort();
    }

    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {
        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);
        if(NETWORK_ATTEMPTING_RECONNECT == rc) {
            // if the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(10 / portTICK_RATE_MS);

        sprintf(topic, "farm/%s/sensor01", aws_iot_client_id);

        // read sensor1
        bme280_set_sensor_mode(BME280_FORCED_MODE, &dev1);
        dev1.delay_ms(40);  // Wait for the measurement to complete
        bme280_get_sensor_data(BME280_ALL, &comp_data, &dev1);

        // do we need to switch?
        //if(relais_on) {
        if(comp_data.humidity >= RH_OFF) {
            // with this we switch off many times but this can not hurt, right!
            // and might fix the glitch
            gpio_set_level(GPIO_OUTPUT_PIN_RELAIS, 0);
            if(relais_on) {
                relais_on = false;
                // in this case we want to send a message
                sprintf(payload,
                        "{"
                        "  \"temperature\": %0.2f,"
                        "  \"pressure\": %0.2f,"
                        "  \"humidity\": %0.2f,"
                        "  \"relais\": \"OFF\","
                        "  \"version\": %s"
                        "}",
                        comp_data.temperature, comp_data.pressure / 100.0, comp_data.humidity, PROJECT_VERSION
                );
                ESP_LOGI(TAG, "%d - #%s#", (int) params.payloadLen, (char *)params.payload);
                params.payloadLen = strlen(payload);
                rc = aws_iot_mqtt_publish(&client, topic, strlen(topic), &params);
                // store tick (used to avoid relais wear)
                last_used_tick = xTaskGetTickCount();
            }
        } else {
            if(!relais_on && comp_data.humidity < RH_ON &&
                    allowed_to_switch_on(&last_used_tick, 30000.0 / portTICK_PERIOD_MS)) {
                gpio_set_level(GPIO_OUTPUT_PIN_RELAIS, 1);
                relais_on = true;
                sprintf(payload,
                        "{"
                        "  \"temperature\": %0.2f,"
                        "  \"pressure\": %0.2f,"
                        "  \"humidity\": %0.2f,"
                        "  \"relais\": \"ON\","
                        "  \"version\": %s"
                        "}",
                        comp_data.temperature, comp_data.pressure / 100.0, comp_data.humidity, PROJECT_VERSION
                );
                ESP_LOGI(TAG, "%d - #%s#", (int) params.payloadLen,
                         (char *) params.payload);
                params.payloadLen = strlen(payload);
                rc = aws_iot_mqtt_publish(&client, topic, strlen(topic), &params);
            }
        }

        ESP_LOGI(TAG, "going to sleep");
        vTaskDelay(5000.0 / portTICK_PERIOD_MS);
    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}

// wifi
static void initialise_wifi(void) {
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}


void app_main() {
    // initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // get client id
    printf("\nOpening Non-Volatile Storage (NVS) handle... ");
    char *aws_iot_client_id = CONFIG_AWS_EXAMPLE_CLIENT_ID;
    nvs_handle partition_handle;
    err = nvs_open("storage", NVS_READWRITE, &partition_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");
        printf("Reading AWS IoT Client ID from NVS ... ");
        size_t required_size;
        err = nvs_get_str(partition_handle, "client_id", NULL, &required_size);  // get the size
        switch (err) {
            case ESP_OK:
                aws_iot_client_id = malloc(required_size);
                err = nvs_get_str(partition_handle, "client_id", aws_iot_client_id, &required_size);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("Client ID is not initialized yet!\n");
                err = nvs_set_str(partition_handle, "client_id", CONFIG_AWS_EXAMPLE_CLIENT_ID);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                printf("Committing updates in NVS ... ");
                err = nvs_commit(partition_handle);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        // Close
        nvs_close(partition_handle);
    }
    printf("AWS IoT Client ID: %s\n", aws_iot_client_id);

    initialise_wifi();
    i2c_master_init();

    //xTaskCreate(&control_task, "control_task",  2048, NULL, 6, NULL);
    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 9216, aws_iot_client_id, 5, NULL, 1);
}
