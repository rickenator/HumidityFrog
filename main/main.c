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
#undef  DEBUG_PASSWORD_SHOW // Define to enable password echo for debugging

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

// --- Console Initialization ---
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

// --- NVS Initialization ---
static void init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

// --- DHT11 Initialization & Reading ---
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

// --- NVS Data Handling ---
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
    // Ensure null termination even if NVS read fails or string is max length
    mqtt_client_id[sizeof(mqtt_client_id) - 1] = '\0';


    nvs_close(nvs_handle);
}

// --- Save Provision Data ---
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

// --- Read Line from UART ---

static void read_line_from_uart(char* dest_buffer, size_t max_len, const char* prompt, bool is_password) { // Added bool is_password
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
        // Read one byte at a time, waiting indefinitely
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&received_char, 1, portMAX_DELAY);

        if (len > 0) {
            // Handle Backspace ('\b' or ASCII 8) and DEL (ASCII 127)
            if (received_char == '\b' || received_char == 127) {
                if (current_pos > 0) {
                    current_pos--;
                    // Erase character visually on the console:
                    uart_write_bytes(UART_NUM_0, "\b \b", 3);
                }
            }
            // Handle Enter/Newline (CR or LF)
            else if (received_char == '\n' || received_char == '\r') {
                dest_buffer[current_pos] = '\0'; // Null-terminate the string
                printf("\n");                   // Move to the next line on the console
                fflush(stdout);
                break; // Exit loop on Enter key
            }
            // Handle regular printable characters
            else if (received_char >= ' ' && received_char <= '~') {
                // --- START Conditional Echo Logic ---
                if (is_password) {
                    #ifdef DEBUG_PASSWORD_SHOW
                        // If debug macro is defined, echo the actual character
                        uart_write_bytes(UART_NUM_0, &received_char, 1);
                    #else
                        // Otherwise, echo an asterisk '*'
                        uart_write_bytes(UART_NUM_0, "*", 1);
                    #endif
                } else {
                    // Not a password, echo the actual character
                    uart_write_bytes(UART_NUM_0, &received_char, 1);
                }
                // --- END Conditional Echo Logic ---

                // Store the actual character in buffer regardless of echo
                dest_buffer[current_pos] = received_char;
                current_pos++;
            }
            // Ignore other non-printable characters
        }
    }
    // Ensure null termination even if max_len-1 characters were entered without Enter
    dest_buffer[max_len - 1] = '\0';
}

// --- Provisioning Wi-Fi Credentials ---
static void provision_wifi_credentials(void)
{

    // Load existing credentials if they exist
	load_provision_data();
	ESP_LOGI(TAG, "Client ID loaded from NVS: '%s'", mqtt_client_id);
	ESP_LOG_BUFFER_HEXDUMP(TAG, mqtt_client_id, strlen(mqtt_client_id) + 1, ESP_LOG_INFO); // +1 for null term

    if (strlen(ssid) > 0 && strlen(password) > 0 && strlen(mqtt_broker) > 0 && strlen(mqtt_client_id) > 0) {
        printf("Current SSID: %s\n", ssid);
        printf("Current MQTT Broker: %s\n", mqtt_broker);
        printf("Current MQTT Client ID: %s\n", mqtt_client_id); // Show loaded client ID
        printf("Press 'r' to reprovision, or any other key to continue...\n");
        fflush(stdout);

        char choice;
        int len = uart_read_bytes(UART_NUM_0, (uint8_t*)&choice, 1, pdMS_TO_TICKS(5000)); // 5-second timeout
        if (len <= 0 || (choice != 'r' && choice != 'R')) {
            ESP_LOGI(TAG, "Using existing credentials.");
            return; // Use existing credentials
        }
        ESP_LOGI(TAG, "Reprovisioning requested.");
        // Clear existing strings before asking for new ones
        memset(ssid, 0, sizeof(ssid));
        memset(password, 0, sizeof(password));
        memset(mqtt_broker, 0, sizeof(mqtt_broker));
        memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    } else {
         ESP_LOGI(TAG, "No complete credentials found in NVS, starting provisioning.");
    }

    // Prompt for SSID
    read_line_from_uart(ssid, sizeof(ssid), "Enter Wi-Fi SSID: ", false);

    // Prompt for password
    read_line_from_uart(password, sizeof(password), "Enter Wi-Fi Password: " , true);

    // Prompt for MQTT broker
    read_line_from_uart(mqtt_broker, sizeof(mqtt_broker), "Enter MQTT Broker IP or Hostname: ", false);

    // Prompt for MQTT Client ID with corrected, simple guidance string
    read_line_from_uart(mqtt_client_id, sizeof(mqtt_client_id),
    "Enter Unique MQTT Client ID (use A-Z, a-z, 0-9, -, _ ; avoid spaces, /, #, +): ",
    false);

    printf("\nSaving configuration:\n");
    printf("  SSID: %s\n", ssid);
    printf("  Password: [HIDDEN]\n"); // Keep password hidden
    printf("  MQTT Broker: %s\n", mqtt_broker);
    printf("  MQTT Client ID: %s\n", mqtt_client_id);
    fflush(stdout);

    save_provision_data(ssid, password, mqtt_broker, mqtt_client_id);
}


// --- Wi-Fi Event Handler ---
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    static int retry_count = 0;
    const int MAX_RETRIES = 10;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi started, attempting to connect...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(5000)); // Add delay before retrying
            esp_wifi_connect();
            retry_count++;
            ESP_LOGI(TAG, "Wi-Fi disconnected, reason: %d, retrying (%d/%d)...",
                     ((wifi_event_sta_disconnected_t*)event_data)->reason, retry_count, MAX_RETRIES);
        } else {
            ESP_LOGE(TAG, "Failed to connect to Wi-Fi after %d retries, restarting...", MAX_RETRIES);
            esp_restart(); // Restart if connection fails persistently
        }
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        retry_count = 0; // Reset retry count on successful connection
    }
}

// --- MQTT Event Handler ---
static void mqtt_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_connected = true;
            ESP_LOGI(TAG, "MQTT connected to broker (Client ID: %s)", mqtt_client_id); // Log client ID on connect
            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected from broker"); // Changed to Warning
            // The client library handles reconnection automatically by default
            break;
        case MQTT_EVENT_PUBLISHED:
            // This event can be verbose, often logged just before in the publish call
            // ESP_LOGI(TAG, "MQTT message published, msg_id=%d", event->msg_id);
            break;
         case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
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
            ESP_LOGI(TAG, "Other MQTT event id: %" PRId32, event_id);
            break;
    }
}

// --- Main Application Logic ---
void app_main(void)
{
    // Initialize UART console
    init_console();
    ESP_LOGI(TAG, "System Booted. Console Initialized.");

    // Initialize NVS
    init_nvs();
    ESP_LOGI(TAG, "NVS Initialized.");

    // Initialize DHT11 (GPIO setup)
    init_dht11();
    ESP_LOGI(TAG, "DHT11 GPIO Initialized.");

    // Provision Wi-Fi & MQTT credentials
    provision_wifi_credentials(); // This loads first, then prompts if needed

    if (strlen(ssid) == 0 || strlen(password) == 0 || strlen(mqtt_broker) == 0 || strlen(mqtt_client_id) == 0) {
        ESP_LOGE(TAG, "Incomplete credentials (SSID, PWD, Broker, ClientID). Please provision via console and restart.");
        // Enter an infinite loop or restart after a delay
         while(1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
        // esp_restart(); // Or just restart
        return; // Should not be reached if looping or restarting
    }
    ESP_LOGI(TAG, "Credentials loaded/provisioned.");
    ESP_LOGI(TAG, "Connecting to SSID: %s", ssid);
    ESP_LOGI(TAG, "Using MQTT Broker: %s", mqtt_broker);
    ESP_LOGI(TAG, "Using MQTT Client ID: %s", mqtt_client_id);

    // Initialize Wi-Fi
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Register Wi-Fi event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // Or other appropriate auth mode
    // Set PMF configuration based on network requirements if needed
    // wifi_config.sta.pmf_cfg.capable = true;
    // wifi_config.sta.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    // Wait for Wi-Fi connection
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
         ESP_LOGI(TAG, "Wi-Fi connected successfully.");
    } else {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi. Restarting...");
        esp_restart(); // Restart if connection fails
    }

    // Construct Broker URI
    ESP_LOGI(TAG, "Attempting to connect to MQTT broker at %s:%d", mqtt_broker, MQTT_PORT);
    char full_uri[96]; // Increased size slightly for safety
    snprintf(full_uri, sizeof(full_uri), "mqtt://%s:%d", mqtt_broker, MQTT_PORT);
    ESP_LOGI(TAG, "Using broker URI: %s", full_uri);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = full_uri,
        .credentials.client_id = mqtt_client_id,
        // Defaults: TCP transport, auto-reconnect enabled
    };

	ESP_LOGI(TAG, "Client ID being passed to init: '%s'", mqtt_cfg.credentials.client_id);
 	ESP_LOG_BUFFER_HEXDUMP(TAG, mqtt_cfg.credentials.client_id, strlen(mqtt_cfg.credentials.client_id) + 1, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Free heap before MQTT init: %" PRIu32, esp_get_free_heap_size());
    ESP_LOGI(TAG, "Initializing MQTT client...");
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client. Restarting...");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart(); // Restart if client init fails
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
    ESP_LOGI(TAG, "MQTT client started. Waiting for connection...");

    // Main loop to read sensor and publish
    char payload[200]; // Increased size to accommodate client ID prefix
    while (1) {
        if (mqtt_connected) { // Only try to publish if connected
            if (read_dht11(&temperature, &humidity) == ESP_OK) {
                snprintf(payload, sizeof(payload), "%s: {\"temperature\": %.1f, \"humidity\": %.1f}",
                         mqtt_client_id, temperature, humidity);

                ESP_LOGI(TAG, "Publishing data: %s", payload);
                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0); // QoS 1
                if (msg_id >= 0) {
                    ESP_LOGI(TAG, "Publish requested (QoS 1), msg_id=%d. Payload: %s", msg_id, payload);
                } else {
                    ESP_LOGE(TAG, "Failed to queue publish message, error code %d", msg_id);
                }
            } else {
                 ESP_LOGW(TAG, "Failed to read DHT11 sensor."); // Changed to warning
                snprintf(payload, sizeof(payload), "%s: DHT11 Sensor offline.", mqtt_client_id);

                ESP_LOGI(TAG, "Publishing offline status: %s", payload);
                 int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 1, 0); // QoS 1
                if (msg_id >= 0) {
                    ESP_LOGI(TAG, "Publish requested (QoS 1), msg_id=%d. Payload: %s", msg_id, payload);
                } else {
                    ESP_LOGE(TAG, "Failed to queue publish message, error code %d", msg_id);
                }
            }
        } else {
             ESP_LOGW(TAG, "MQTT not connected, skipping publish attempt.");
             // Optionally add a longer delay here if disconnected to avoid spamming logs
             // vTaskDelay(pdMS_TO_TICKS(10000)); // e.g., wait 10 seconds if disconnected
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Publish every 5 seconds (adjust as needed)
    }
}