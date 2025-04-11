#include <string.h>
#include <inttypes.h> // For PRId32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "mqtt_client.h"
#include "esp_rom_sys.h"
#include "nvs.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_netif.h"
#include "esp_timer.h"

#define MQTT_PORT 1883
#define MQTT_TOPIC "esp32/dht11"
#define DHT_PIN GPIO_NUM_4
#define UART_NUM UART_NUM_0 // Use UART0 for serial communication
#define BUF_SIZE 256

static const char *TAG = "mqtt_dht";
static char mqtt_broker[64] = "";

static float temperature = 0.0;
static float humidity = 0.0;
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Event group to signal Wi-Fi connection
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static char ssid[32] = "";
static char password[64] = "";

void init_console(void) {
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    // Set UART pins (default for console)
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Suppress deprecation warnings for ESP-IDF v5.4.1
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    // Register UART for VFS
    esp_vfs_dev_uart_register();

    // Set line endings for the UART port
    esp_vfs_dev_uart_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
    esp_vfs_dev_uart_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
#pragma GCC diagnostic pop
}

static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void init_dht11(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DHT_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

static esp_err_t read_dht11(float *temp, float *hum)
{
    // Simulate DHT11 not being connected
    ESP_LOGI(TAG, "DHT11 not connected, skipping read...");
    return ESP_FAIL;
}

static void load_provision_data(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("wifi", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return;
    }

    size_t len;

    len = sizeof(ssid);
    nvs_get_str(nvs_handle, "ssid", ssid, &len);

    len = sizeof(password);
    nvs_get_str(nvs_handle, "password", password, &len);

    len = sizeof(mqtt_broker);
    nvs_get_str(nvs_handle, "broker", mqtt_broker, &len);

    nvs_close(nvs_handle);
}


static void save_provision_data(const char *new_ssid, const char *new_pass, const char *new_broker)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("wifi", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
        return;
    }

    nvs_set_str(nvs_handle, "ssid", new_ssid);
    nvs_set_str(nvs_handle, "password", new_pass);
    nvs_set_str(nvs_handle, "broker", new_broker);

    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}


static void provision_wifi_credentials(void)
{
    char buffer[BUF_SIZE];
    int pos = 0;

    // Load existing credentials if they exist
    load_provision_data();
    if (strlen(ssid) > 0 && strlen(password) > 0 && strlen(mqtt_broker) > 0) {
        printf("Current SSID: %s, MQTT Broker: %s\n", ssid, mqtt_broker);
        printf("Press 'r' to reprovision, or any other key to continue...\n");
        fflush(stdout);

        char choice;
        int len = uart_read_bytes(UART_NUM, (uint8_t*)&choice, 1, 5000 / portTICK_PERIOD_MS); // 5-second timeout
        if (len <= 0 || choice != 'r') {
            return; // Use existing credentials
        }
    }

    // Prompt for SSID
    printf("Enter Wi-Fi SSID: ");
    fflush(stdout);
    pos = 0;
    while (1) {
        int len = uart_read_bytes(UART_NUM, (uint8_t*)&buffer[pos], 1, 1000 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (buffer[pos] == '\n' || buffer[pos] == '\r') {
                buffer[pos] = '\0';
                strncpy(ssid, buffer, sizeof(ssid) - 1);
                ssid[sizeof(ssid) - 1] = '\0';
                break;
            }
            pos++;
            if (pos >= sizeof(ssid) - 1) {
                buffer[pos] = '\0';
                strncpy(ssid, buffer, sizeof(ssid) - 1);
                ssid[sizeof(ssid) - 1] = '\0';
                break;
            }
        }
    }

    // Prompt for password
    printf("Enter Wi-Fi Password: ");
    fflush(stdout);
    pos = 0;
    while (1) {
        int len = uart_read_bytes(UART_NUM, (uint8_t*)&buffer[pos], 1, 1000 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (buffer[pos] == '\n' || buffer[pos] == '\r') {
                buffer[pos] = '\0';
                strncpy(password, buffer, sizeof(password) - 1);
                password[sizeof(password) - 1] = '\0';
                break;
            }
            pos++;
            if (pos >= sizeof(password) - 1) {
                buffer[pos] = '\0';
                strncpy(password, buffer, sizeof(password) - 1);
                password[sizeof(password) - 1] = '\0';
                break;
            }
        }
    }

    // Prompt for MQTT broker
    printf("Enter MQTT Broker (e.g., 192.168.1.158): ");
    fflush(stdout);
    pos = 0;
    while (1) {
        int len = uart_read_bytes(UART_NUM, (uint8_t*)&buffer[pos], 1, 1000 / portTICK_PERIOD_MS);
        if (len > 0) {
            if (buffer[pos] == '\n' || buffer[pos] == '\r') {
                buffer[pos] = '\0';
                strncpy(mqtt_broker, buffer, sizeof(mqtt_broker) - 1);
                mqtt_broker[sizeof(mqtt_broker) - 1] = '\0';
                break;
            }
            pos++;
            if (pos >= sizeof(mqtt_broker) - 1) {
                buffer[pos] = '\0';
                strncpy(mqtt_broker, buffer, sizeof(mqtt_broker) - 1);
                mqtt_broker[sizeof(mqtt_broker) - 1] = '\0';
                break;
            }
        }
    }

    printf("Saving SSID: %s, Password: %s, MQTT Broker: %s\n", ssid, password, mqtt_broker);
    save_provision_data(ssid, password, mqtt_broker);
}


static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    static int retry_count = 0;
    const int MAX_RETRIES = 10;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi started, attempting to connect...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < MAX_RETRIES) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Wi-Fi disconnected, reason: %d, retrying (%d/%d)...", 
                     ((wifi_event_sta_disconnected_t*)event_data)->reason, retry_count, MAX_RETRIES);
        } else {
            ESP_LOGE(TAG, "Failed to connect to Wi-Fi after %d retries", MAX_RETRIES);
            esp_restart();
        }
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        retry_count = 0; // Reset retry count on successful connection
    }
}

static bool mqtt_connected = false;

static void mqtt_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected to broker");
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGI(TAG, "MQTT disconnected from broker");
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT message published, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            break;
        default:
            ESP_LOGI(TAG, "Other MQTT event id:%" PRId32, event_id);
            break;
    }
}

void app_main(void)
{
    // Initialize UART console
    init_console();

    // Initialize NVS
    init_nvs();

    // Initialize DHT11
    init_dht11();

    // Provision Wi-Fi credentials
    provision_wifi_credentials();

    // Check if Wi-Fi credentials are set
    if (strlen(ssid) == 0 || strlen(password) == 0) {
        ESP_LOGE(TAG, "Wi-Fi credentials not set, cannot connect");
        return;
    }

    // Initialize Wi-Fi
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    // Register Wi-Fi event handlers
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // Explicitly set WPA2-PSK
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    // Wait for Wi-Fi connection
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi connected successfully");

    if (mqtt_broker[0] == '\0') {
        ESP_LOGE(TAG, "MQTT broker string is empty, aborting...");
        return;
    }

    // Initialize MQTT
	ESP_LOGI(TAG, "Attempting to connect to MQTT broker at %s:%d", mqtt_broker,
			 MQTT_PORT);
			 
	char full_uri[96];
	memset(full_uri, 0, sizeof(full_uri));

	snprintf(full_uri, sizeof(full_uri), "mqtt://%s:%d", mqtt_broker,
			 MQTT_PORT);
			 
	// Log final URI to inspect its integrity
	ESP_LOGI(TAG, "Using broker URI: %s", full_uri);
	ESP_LOG_BUFFER_HEXDUMP(TAG, full_uri, strlen(full_uri), ESP_LOG_INFO);

	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = full_uri, // e.g. "mqtt://192.168.1.158:1883"
	};

    ESP_LOGI(TAG, "Free heap before MQTT init: %" PRIu32, esp_get_free_heap_size()); // Requires #include <inttypes.h>
	// Initialize MQTT client
	ESP_LOGI(TAG, "Initializing MQTT client...");
	mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
	if (mqtt_client == NULL) {
		ESP_LOGE(TAG, "Failed to initialize MQTT client");
		// ===>>> ADD THIS: Stop further execution <<<===
		// Option 1: Restart the ESP32
		esp_restart();
		// Option 2: Enter an infinite loop or return from app_main if
		// appropriate while(1) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
		// return;
	}

	esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    // Read DHT11 sensor and publish data
    while (1) {
        if (read_dht11(&temperature, &humidity) == ESP_OK) {
            int msg_id = -1;
            char payload[128];
            snprintf(payload, sizeof(payload), "{\"temperature\": %.1f, \"humidity\": %.1f}", temperature, humidity);

            ESP_LOGI(TAG, "Publishing data: %s", payload);
            msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
            if (msg_id >= 0) {
                ESP_LOGI(TAG, "Published: %s, msg_id=%d", payload, msg_id);
            } else {
                ESP_LOGE(TAG, "Failed to publish message, msg_id=%d", msg_id);
            }
            
			if (!mqtt_connected) {
                ESP_LOGE(TAG, "MQTT not connected, cannot publish data");
            }
        } else {
            ESP_LOGE(TAG, "Failed to read DHT11 sensor, retrying...");
            int msg_id = -1;
            char payload[128];
            snprintf(payload, sizeof(payload), "DHT11 Sensor offline. Retrying...");

            ESP_LOGI(TAG, "Publishing offline message: %s", payload);
            msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
            if (msg_id >= 0) {
                ESP_LOGI(TAG, "Published: %s, msg_id=%d", payload, msg_id);
            } else {
                ESP_LOGE(TAG, "Failed to publish offline message, msg_id=%d", msg_id);
            }
            if(!mqtt_connected) {
                ESP_LOGE(TAG, "MQTT not connected, cannot publish offline message");
            }
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Publish every 5 seconds
    }
}