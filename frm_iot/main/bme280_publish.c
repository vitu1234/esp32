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

#include <BME280Sensor.h>

#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "cJSON.h"

#include "esp_log.h"
static const char *TAG = "ESP_IOT";


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


void aws_iot_task(void *param) {
    char *aws_iot_client_id = (char*) param;
    TickType_t start_tick;
    int8_t rslt;

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    const char *config = "{"
        "\"name\": \"inside\", "
        "\"component\": \"comp123\", "
        "\"params\": [\"119\"] }";



    //uint8_t settings_sel;
    //struct bme280_data comp_data;

    // bme280 initialization and settings
    /*struct bme280_dev dev1 = {
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

    struct bme280_dev dev2 = dev1;
    dev2.dev_id = BME280_I2C_ADDR_SEC;  // sensor2

    ESP_LOGI(TAG, "BME-280 initialization");
    rslt = bme280_init(&dev1);
    if(rslt != SUCCESS) {
        ESP_LOGE(TAG, "BME-280 sensor1 init or setting error. code: %d", rslt);
        abort();
    }
    rslt = bme280_init(&dev2);
    if(rslt != SUCCESS) {
        ESP_LOGE(TAG, "BME-280 sensor2 init or setting error. code: %d", rslt);
        abort();
    }

    ESP_LOGI(TAG, "BME-280 settings");
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    rslt = bme280_set_sensor_settings(settings_sel, &dev1);
    if(rslt != SUCCESS) {
        ESP_LOGE(TAG, "BME-280 sensor1 setting error. code: %d", rslt);
        abort();
    }
    rslt = bme280_set_sensor_settings(settings_sel, &dev2);
    if(rslt != SUCCESS) {
        ESP_LOGE(TAG, "BME-280 sensor2 setting error. code: %d", rslt);
        abort();
    }
    */
    BME280Sensor sensor1(0x76, "sensor1");
    BME280Sensor sensor2(0x77, "sensor2");

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

    char payload[100];
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
        if(NETWORK_RECONNECTED == rc || SUCCESS == rc) {
            // no setting start_tick again in case of an reconnect attempt
            // with that the measurements are on average 5 min. apart
            // => considering network latency this is good enough!
            // to get precise 5 min. intervals we would have to do the reconnect
            // to AWS at the end of the loop after a 2nd wakeup at say 290 sec.
            start_tick = xTaskGetTickCount();
        }
        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);
        if(NETWORK_ATTEMPTING_RECONNECT == rc) {
            // if the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }

        ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(10 / portTICK_RATE_MS);

        // sensor1
        //bme280_set_sensor_mode(BME280_FORCED_MODE, &dev1);
        //dev1.delay_ms(40);  // Wait for the measurement to complete and print data
        //bme280_get_sensor_data(BME280_ALL, &comp_data, &dev1);
        sprintf(topic, "farm/%s", aws_iot_client_id);
        //sprintf(payload,
        //    "sensor01,temperature,%0.2f\n"
        //    "sensor01,pressure,%0.2f\n"
        //    "sensor01,humidity,%0.2f\n",
        //    comp_data.temperature, comp_data.pressure / 100.0, comp_data.humidity
        //);
        sensor1.update();
        sensor2.update();
        sensor1.to_csv(payload);
        sensor2.to_csv(payload);

        // sensor2
        /*
        bme280_set_sensor_mode(BME280_FORCED_MODE, &dev2);
        dev2.delay_ms(40);  // Wait for the measurement to complete and print data
        bme280_get_sensor_data(BME280_ALL, &comp_data, &dev2);
        sprintf(payload + strlen(payload),
            "sensor02,temperature,%0.2f\n"
            "sensor02,pressure,%0.2f\n"
            "sensor02,humidity,%0.2f\n",
            comp_data.temperature, comp_data.pressure / 100.0, comp_data.humidity
        );
        */

        params.payloadLen = strlen(payload);
        rc = aws_iot_mqtt_publish(&client, topic, strlen(topic), &params);
        if (rc == MQTT_REQUEST_TIMEOUT_ERROR) {
            ESP_LOGW(TAG, "QOS0 publish ack not received.");
            rc = SUCCESS;
        }
        // ESP_LOGI(TAG, "%d - #%s#", (int) params.payloadLen, (char *)params.payload);

        // reduce traffic to one measurement every 5 minutes
        ESP_LOGI(TAG, "going to sleep");
        vTaskDelayUntil(&start_tick, 300000.0 / portTICK_PERIOD_MS);
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
    //i2c_master_init();

    //xTaskCreate(&aws_iot_task, "aws_iot_task",  2048, NULL, 6, NULL);
    xTaskCreatePinnedToCore(&aws_iot_task, "aws_iot_task", 9216, aws_iot_client_id, 5, NULL, 1);
}
