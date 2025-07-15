#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "iota_wifi_manager.h"
#include "iota_mqtt.h"
#include "iota_system_task.h"

#define DHT_PIN        GPIO_NUM_3
#define DUST_LED_PIN   GPIO_NUM_5
#define DUST_ADC_CH    ADC1_CHANNEL_4 // GPIO32
#define TAG            "AQI_APP"

TaskHandle_t sensor_task_handle = NULL;
volatile bool print_once = false;

typedef struct {
    float temperature;
    float humidity;
    bool success;
} dht11_data_t;

float get_pm25_concentration(float voltage) {
    float dust_density = ((voltage - 0.9f) / 0.5f) * 100.0f;
    if (dust_density < 0) dust_density = 0;
    return dust_density;
}

float calculate_pm25_aqi(float concentration) {
    struct {
        float c_low, c_high;
        int i_low, i_high;
    } aqi_table[] = {
        {0.0, 12.0, 0, 50},
        {12.1, 35.4, 51, 100},
        {35.5, 55.4, 101, 150},
        {55.5, 150.4, 151, 200},
        {150.5, 250.4, 201, 300},
        {250.5, 500.4, 301, 500},
    };

    for (int i = 0; i < sizeof(aqi_table)/sizeof(aqi_table[0]); i++) {
        if (concentration >= aqi_table[i].c_low && concentration <= aqi_table[i].c_high) {
            return ((aqi_table[i].i_high - aqi_table[i].i_low) * (concentration - aqi_table[i].c_low) / 
                   (aqi_table[i].c_high - aqi_table[i].c_low)) + aqi_table[i].i_low;
        }
    }
    return 500.0f;
}

static int dht11_wait_level(gpio_num_t pin, int level, int timeout_us) {
    int us = 0;
    while (gpio_get_level(pin) == level) {
        if (++us > timeout_us) return -1;
        esp_rom_delay_us(1);
    }
    return us;
}

dht11_data_t read_dht11() {
    dht11_data_t result = {0};
    int data[40] = {0};
    uint8_t bits[5] = {0};

    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);

    if (dht11_wait_level(DHT_PIN, 0, 1000) < 0 || dht11_wait_level(DHT_PIN, 1, 1000) < 0) {
        result.success = false;
        return result;
    }

    for (int i = 0; i < 40; i++) {
        if (dht11_wait_level(DHT_PIN, 0, 1000) < 0) return result;
        int width = dht11_wait_level(DHT_PIN, 1, 1000);
        if (width < 0) return result;
        data[i] = (width > 40) ? 1 : 0;
    }

    for (int i = 0; i < 40; i++) {
        bits[i / 8] <<= 1;
        bits[i / 8] |= data[i];
    }

    if ((uint8_t)(bits[0] + bits[1] + bits[2] + bits[3]) != bits[4]) {
        result.success = false;
        return result;
    }

    result.humidity = bits[0];
    result.temperature = bits[2];
    result.success = true;
    return result;
}

float get_dust_density() {
    int raw = 0;
    gpio_set_level(DUST_LED_PIN, 0);
    esp_rom_delay_us(280);
    raw = adc1_get_raw(DUST_ADC_CH);
    esp_rom_delay_us(40);
    gpio_set_level(DUST_LED_PIN, 1);

    float voltage = raw * (3.3f / 4095.0f);
    return get_pm25_concentration(voltage);
}

void mqtt_handler_callback(const char *topic, const char *payload) {
    if (strcmp(payload, "print") == 0) {
        print_once = true;
        ESP_LOGI(TAG, "Received print command from MQTT");
    }
}

void sensor_task(void *arg) {
    while (1) {
        if (print_once) {
            print_once = false;
            dht11_data_t dht = read_dht11();
            float pm25 = get_dust_density();
            float aqi = calculate_pm25_aqi(pm25);

            if (dht.success) {
                ESP_LOGI(TAG,
                    "Temp: %.1f°C, Humidity: %.1f%%, PM2.5: %.1f µg/m³, AQI: %.1f",
                    dht.temperature, dht.humidity, pm25, aqi);
                char payload[128];
                snprintf(payload, sizeof(payload), 
                    "{\"temp\":%.1f,\"humidity\":%.1f,\"aqi\":%.1f}", 
                    dht.temperature, dht.humidity, aqi);
                iota_mqtt_client_publish("esp32_aqi", payload, 0, 1, 0);
            } else {
                ESP_LOGE(TAG, "DHT11 failed to read");
                iota_mqtt_client_publish("esp32_aqi", "{\"error\":\"DHT11 read failed\"}", 0, 1, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    gpio_set_direction(DUST_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DUST_LED_PIN, 1);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(DUST_ADC_CH, ADC_ATTEN_DB_11);

    iota_wifi_start();
    iota_register_mqtt_callbacks(mqtt_handler_callback);

    iota_create_system_task(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
