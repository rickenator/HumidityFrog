#include <string.h>
#include <inttypes.h> // For PRId32, PRIu32
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
#include "esp_rom_sys.h" // For esp_rom_delay_us
#include "nvs.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_netif.h"
#include "esp_timer.h"   // For esp_timer_get_time

#define MQTT_PORT 1883
#define MQTT_TOPIC "esp32/dht11"
#define DHT_PIN GPIO_NUM_4
#define UART_NUM UART_NUM_0 // Use UART0 for serial communication
#define BUF_SIZE 256
#undef  DEBUG_PASSWORD_SHOW // Define via build system if needed: -DDEBUG_PASSWORD_SHOW=1
// #define SIMULATE_DHT11 // Comment out this line to use actual DHT11 reading

static const char *TAG = "mqtt_dht";

// --- Configuration Variables ---
static char ssid[32] = "";
static char password[64] = "";
static char mqtt_broker[64] = "";
static char mqtt_client_id[64] = "";

// --- Sensor & MQTT State ---
static float temperature = 0.0;
static float humidity = 0.0;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

// Event group to signal Wi-Fi connection
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// --- Console Initialization (No changes) ---
void init_console(void) {
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
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    esp_vfs_dev_uart_register();
    esp_vfs_dev_uart_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
    esp_vfs_dev_uart_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);
#pragma GCC diagnostic pop
}

// --- NVS Initialization (No changes) ---
static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// --- DHT11 GPIO Initialization (No changes) ---
static void init_dht11(void) {
    // Ensure pull-up is enabled as DHT11 idles high
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DHT_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD, // Use Open Drain for bi-directional comms
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    // Set initial level high (let pull-up do its work)
    gpio_set_level(DHT_PIN, 1);
}

// --- DHT11 Reading Implementation ---

// Helper function to wait for a specific pin level with timeout
static esp_err_t wait_for_level(gpio_num_t pin, int level, uint32_t timeout_us) {
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if ((esp_timer_get_time() - start_time) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
        // Brief yield or delay might be needed if timeout is long, but
        // for DHT11's short pulses, busy-waiting is often necessary.
        // esp_rom_delay_us(1); // Minimal delay
    }
    return ESP_OK;
}


/*
 * @brief Reads temperature and humidity data from the DHT11 sensor.
 *
 * Connections:
 *   ESP32 Pin   |   DHT11 Pin
 * ------------- | -------------
 *    GPIO_NUM_4 |   DATA (Pin 2)  <- Set by DHT_PIN define
 *    3.3V       |   VCC (Pin 1)
 *    GND        |   GND (Pin 4)
 *
 *   Note: A 4.7k Ohm to 10k Ohm pull-up resistor between DATA and VCC is recommended.
 *         The internal ESP32 pull-up might be sufficient sometimes, but external is more reliable.
 *         Pin 3 on DHT11 is Not Connected.
 *
 * ASCII Diagram (Typical 3/4 pin module):
 *
 *       ESP32                     DHT11 Module
 *      +-----+                   +-----------+
 *      | 3V3 |-------------------| VCC / +   |
 *      |     |                   |           |
 *      | GND |-------------------| GND / -   |
 *      |     |       +----[R]---+           |  R = 4.7k-10k Pull-up
 *      | GPIO4 |-------+---------| DATA / out|
 *      +-----+                   +-----------+
 *
 * @param temp Pointer to float where temperature (Celsius) will be stored.
 * @param hum Pointer to float where humidity (%) will be stored.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, ESP_ERR_INVALID_CRC on checksum failure,
 *         ESP_FAIL for other internal errors.
 */
static esp_err_t read_dht11(float *temp, float *hum)
{
    #ifdef SIMULATE_DHT11
        ESP_LOGI(TAG, "SIMULATE_DHT11: Skipping actual read.");
        *temp = 25.5;
        *hum = 55.0;
        // return ESP_FAIL; // Simulate read failure
        return ESP_OK;     // Simulate successful read
    #else

    uint8_t data[5] = {0, 0, 0, 0, 0};
    esp_err_t read_status = ESP_FAIL; // Default to failure
    int bit_error_index = -1; // Track which bit failed, if any

    // Critical section for timing-sensitive operations
    portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mutex);

    // --- Send Start Signal ---
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(18 * 1000);
    gpio_set_level(DHT_PIN, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);

    // --- Wait for DHT11 Response ---
    if (wait_for_level(DHT_PIN, 0, 90) != ESP_OK) {
        read_status = ESP_ERR_TIMEOUT; // Error: No response low
        goto exit_critical; // Use goto for cleaner exit from critical section on error
    }
    if (wait_for_level(DHT_PIN, 1, 90) != ESP_OK) {
        read_status = ESP_ERR_TIMEOUT; // Error: No response high
        goto exit_critical;
    }

    // --- Read Data Bits (40 bits) ---
    for (int i = 0; i < 40; i++) {
        if (wait_for_level(DHT_PIN, 0, 60) != ESP_OK) { // Wait for start of bit (low)
            read_status = ESP_ERR_TIMEOUT; bit_error_index = i; goto exit_critical;
        }
        if (wait_for_level(DHT_PIN, 1, 80) != ESP_OK) { // Wait for end of data pulse (high)
            read_status = ESP_ERR_TIMEOUT; bit_error_index = i; goto exit_critical;
        }

        int64_t start_time_high = esp_timer_get_time();
        // Wait for pulse end (low again) - MUST complete for timing
        if (wait_for_level(DHT_PIN, 0, 90) != ESP_OK) {
             read_status = ESP_ERR_TIMEOUT; bit_error_index = i; goto exit_critical;
        }
        int64_t end_time_high = esp_timer_get_time();
        uint32_t high_duration = (uint32_t)(end_time_high - start_time_high);

        data[i / 8] <<= 1;
        if (high_duration > 40) { // Threshold is ~30-40us
            data[i / 8] |= 1;
        }
    }

    read_status = ESP_OK; // If loop completed without error

exit_critical:
    // Set pin back to high-impedance input with pull-up (its default state)
    // Though setting direction to input already does this with OD+Pullup config
    gpio_set_level(DHT_PIN, 1);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT); // Redundant but safe
    portEXIT_CRITICAL(&mutex);
    // --- End of Critical Section ---


    // --- Process results *outside* critical section ---
    if (read_status == ESP_ERR_TIMEOUT) {
        if (bit_error_index < 0) { // Timeout occurred during response phase
             ESP_LOGE(TAG, "DHT11 Read Error: Timeout waiting for response.");
        } else { // Timeout occurred during data bit reading
             ESP_LOGE(TAG, "DHT11 Read Error: Timeout waiting for bit %d.", bit_error_index);
        }
        return ESP_ERR_TIMEOUT;
    } else if (read_status != ESP_OK) {
        // Catch any other unexpected error status from the critical section
         ESP_LOGE(TAG, "DHT11 Read Error: Unknown error during timing sequence.");
         return ESP_FAIL;
    }

    // Checksum Validation
    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) {
        ESP_LOGE(TAG, "DHT11 Checksum Error. Received: %02x %02x %02x %02x Checksum: %02x, Calculated: %02x",
                 data[0], data[1], data[2], data[3], data[4], checksum);
        return ESP_ERR_INVALID_CRC;
    }

    // Data Conversion
    *hum = (float)data[0];
    *temp = (float)data[2];

    // Use DEBUG level for raw data as it's less critical than errors/success
    ESP_LOGD(TAG, "DHT11 Raw Data: %02x %02x %02x %02x %02x", data[0], data[1], data[2], data[3], data[4]);
    ESP_LOGI(TAG, "DHT11 Read Success: Temp=%.1fC, Hum=%.1f%%", *temp, *hum);

    return ESP_OK;

    #endif // SIMULATE_DHT11
}

// --- NVS Data Handling (No changes) ---
static void load_provision_data(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("wifi", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for reading: %s. Assuming first boot.", esp_err_to_name(ret));
        return;
    }
    size_t len;
    len = sizeof(ssid);
    nvs_get_str(nvs_handle, "ssid", ssid, &len);
    len = sizeof(password);
    nvs_get_str(nvs_handle, "password", password, &len);
    len = sizeof(mqtt_broker);
    nvs_get_str(nvs_handle, "broker", mqtt_broker, &len);
    len = sizeof(mqtt_client_id);
    nvs_get_str(nvs_handle, "client_id", mqtt_client_id, &len);
    mqtt_client_id[sizeof(mqtt_client_id) - 1] = '\0';
    nvs_close(nvs_handle);

    // Add hex dump after loading
    ESP_LOGI(TAG, "Client ID loaded from NVS: '%s'", mqtt_client_id);
    ESP_LOG_BUFFER_HEXDUMP(TAG, mqtt_client_id, strlen(mqtt_client_id) + 1, ESP_LOG_INFO);
}

// --- Save Provision Data (No changes) ---
static void save_provision_data(const char *new_ssid, const char *new_pass, const char *new_broker, const char *new_client_id)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open("wifi", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(ret));
        return;
    }
    nvs_set_str(nvs_handle, "ssid", new_ssid);
    nvs_set_str(nvs_handle, "password", new_pass);
    nvs_set_str(nvs_handle, "broker", new_broker);
    nvs_set_str(nvs_handle, "client_id", new_client_id);
    ret = nvs_commit(nvs_handle);
     if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS data: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Provisioning data saved successfully.");
    }
    nvs_close(nvs_handle);
}

// --- Read Line from UART (No changes to function itself) ---
static void read_line_from_uart(char* dest_buffer, size_t max_len, const char* prompt, bool is_password) {
    if (dest_buffer == NULL || max_len == 0) {
        ESP_LOGE(TAG, "read_line_from_uart: Invalid arguments");
        return;
    }

    printf("%s", prompt);
    fflush(stdout);
    memset(dest_buffer, 0, max_len); // Clear destination buffer first
    int current_pos = 0;

    while (current_pos < max_len - 1) {
        char received_char;
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&received_char, 1, portMAX_DELAY);
        if (len > 0) {
            if (received_char == '\b' || received_char == 127) {
                if (current_pos > 0) {
                    current_pos--;
                    uart_write_bytes(UART_NUM_0, "\b \b", 3);
                }
            }
            else if (received_char == '\n' || received_char == '\r') {
                dest_buffer[current_pos] = '\0';
                printf("\n");
                fflush(stdout);
                break;
            }
            else if (received_char >= ' ' && received_char <= '~') {
                if (is_password) {
                    #ifdef DEBUG_PASSWORD_SHOW
                        uart_write_bytes(UART_NUM_0, &received_char, 1);
                    #else
                        uart_write_bytes(UART_NUM_0, "*", 1);
                    #endif
                } else {
                    uart_write_bytes(UART_NUM_0, &received_char, 1);
                }
                dest_buffer[current_pos] = received_char;
                current_pos++;
            }
        }
    }
    dest_buffer[max_len - 1] = '\0';
}

// --- Provisioning Wi-Fi Credentials (No changes to function itself) ---
static void provision_wifi_credentials(void)
{
    load_provision_data();

    if (strlen(ssid) > 0 && strlen(password) > 0 && strlen(mqtt_broker) > 0 && strlen(mqtt_client_id) > 0) {
        printf("Current SSID: %s\n", ssid);
        printf("Current MQTT Broker: %s\n", mqtt_broker);
        printf("Current MQTT Client ID: %s\n", mqtt_client_id);
        printf("Press 'r' to reprovision, or any other key to continue...\n");
        fflush(stdout);
        char choice;
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&choice, 1, pdMS_TO_TICKS(5000));
        if (len <= 0 || (choice != 'r' && choice != 'R')) {
            ESP_LOGI(TAG, "Using existing credentials.");
            return;
        }
        ESP_LOGI(TAG, "Reprovisioning requested.");
        memset(ssid, 0, sizeof(ssid));
        memset(password, 0, sizeof(password));
        memset(mqtt_broker, 0, sizeof(mqtt_broker));
        memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    } else {
         ESP_LOGI(TAG, "No complete credentials found in NVS, starting provisioning.");
    }

    read_line_from_uart(ssid, sizeof(ssid), "Enter Wi-Fi SSID: ", false);
    read_line_from_uart(password, sizeof(password), "Enter Wi-Fi Password: " , true);
    read_line_from_uart(mqtt_broker, sizeof(mqtt_broker), "Enter MQTT Broker IP or Hostname: ", false);
    read_line_from_uart(mqtt_client_id, sizeof(mqtt_client_id),
                        "Enter Unique MQTT Client ID (use A-Z, a-z, 0-9, -, _ ; avoid spaces, /, #, +): ",
                        false);

    printf("\nSaving configuration:\n");
    printf("  SSID: %s\n", ssid);
    printf("  Password: [HIDDEN]\n");
    printf("  MQTT Broker: %s\n", mqtt_broker);
    printf("  MQTT Client ID: %s\n", mqtt_client_id);
    fflush(stdout);

    save_provision_data(ssid, password, mqtt_broker, mqtt_client_id);
}


// --- Wi-Fi Event Handler (No changes) ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    static int retry_count = 0;
    const int MAX_RETRIES = 10;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi started, attempting to connect...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Wi-Fi disconnected, reason: %d, retrying (%d/%d)...",
                     ((wifi_event_sta_disconnected_t*)event_data)->reason, retry_count, MAX_RETRIES);
        } else {
            ESP_LOGE(TAG, "Failed to connect to Wi-Fi after %d retries, restarting...", MAX_RETRIES);
            esp_restart();
        }
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        retry_count = 0;
    }
}

// --- MQTT Event Handler (No changes) ---
static void mqtt_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected to broker (Client ID: %s)", mqtt_client_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected from broker");
            break;
        case MQTT_EVENT_PUBLISHED:
            break;
         case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGD(TAG, "MQTT_EVENT_BEFORE_CONNECT"); // Changed to Debug level
            break;
        case MQTT_EVENT_ERROR:
             ESP_LOGE(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGE(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGE(TAG, "Last captured errno : %d (%s)", event->error_handle->esp_transport_sock_errno,
                        strerror(event->error_handle->esp_transport_sock_errno));
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGE(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            } else {
                ESP_LOGE(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
            break;
        default:
            ESP_LOGD(TAG, "Other MQTT event id: %" PRId32, event_id); // Changed to Debug level
            break;
    }
}

// --- Main Application Logic ---
void app_main(void)
{
    init_console();
    ESP_LOGI(TAG, "System Booted. Console Initialized.");
    init_nvs();
    ESP_LOGI(TAG, "NVS Initialized.");
    init_dht11();
    ESP_LOGI(TAG, "DHT11 GPIO Initialized.");

    provision_wifi_credentials();

    if (strlen(ssid) == 0 || strlen(password) == 0 || strlen(mqtt_broker) == 0 || strlen(mqtt_client_id) == 0) {
        ESP_LOGE(TAG, "Incomplete credentials (SSID, PWD, Broker, ClientID). Please provision via console and restart.");
         while(1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
        return;
    }
    ESP_LOGI(TAG, "Credentials loaded/provisioned.");
    ESP_LOGI(TAG, "Connecting to SSID: %s", ssid);
    ESP_LOGI(TAG, "Using MQTT Broker: %s", mqtt_broker);
    ESP_LOGI(TAG, "Using MQTT Client ID: %s", mqtt_client_id);

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
         ESP_LOGI(TAG, "Wi-Fi connected successfully.");
    } else {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi. Restarting...");
        esp_restart();
    }

    ESP_LOGI(TAG, "Attempting to connect to MQTT broker at %s:%d", mqtt_broker, MQTT_PORT);
    char full_uri[96];
    snprintf(full_uri, sizeof(full_uri), "mqtt://%s:%d", mqtt_broker, MQTT_PORT);
    ESP_LOGI(TAG, "Using broker URI: %s", full_uri);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = full_uri,
        .credentials.client_id = mqtt_client_id,
    };

    ESP_LOGI(TAG, "Client ID being passed to init: '%s'", mqtt_cfg.credentials.client_id);
    ESP_LOG_BUFFER_HEXDUMP(TAG, mqtt_cfg.credentials.client_id, strlen(mqtt_cfg.credentials.client_id) + 1, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Free heap before MQTT init: %" PRIu32, esp_get_free_heap_size());
    ESP_LOGI(TAG, "Initializing MQTT client...");
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client. Restarting...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
    ESP_LOGI(TAG, "MQTT client started. Waiting for connection...");

    char payload[200];
    while (1) {
        if (mqtt_connected) {
            if (read_dht11(&temperature, &humidity) == ESP_OK) {
                 // Use consistent prefixing style
                snprintf(payload, sizeof(payload), "%s: {\"temperature\": %.1f, \"humidity\": %.1f}",
                         mqtt_client_id, temperature, humidity);

                ESP_LOGI(TAG, "Publishing data: %s", payload);
                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
                if (msg_id < 0) {
                    // Log publish error only if queuing fails
                    ESP_LOGE(TAG, "Failed to queue publish message, error code %d", msg_id);
                }
                // Success logging is handled by MQTT_EVENT_PUBLISHED if needed, or implied by lack of error
            } else {
                ESP_LOGW(TAG, "Failed to read DHT11 sensor.");
                snprintf(payload, sizeof(payload), "%s: DHT11 Sensor offline.", mqtt_client_id);
                ESP_LOGI(TAG, "Publishing offline status: %s", payload);
                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0);
                if (msg_id < 0) {
                    ESP_LOGE(TAG, "Failed to queue publish offline status, error code %d", msg_id);
                }
            }
        } else {
             ESP_LOGW(TAG, "MQTT not connected, skipping publish attempt.");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}