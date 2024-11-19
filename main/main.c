// Board: AI Thinker NodeMCU-32S
// Amplifier: Adafruit MAX98357A

#include <math.h>
#include <freertos/FreeRTOS.h>
#include "soc/i2s_reg.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_tls_crypto.h"
#include "esp_websocket_client.h"

static const char *WIFI_TAG = "WiFi";
static const char *I2S_TAG = "I2S";

static uint8_t message_buffer[1024];
static const char MESSAGE_HEADER[] = "{ \"type\": \"input_audio_buffer.append\", \"audio\":\"";
static const char MESSAGE_FOOTER[] = "\"}";
static const int MESSAGE_HEADER_LEN = sizeof(MESSAGE_HEADER) - 1;
static const int MESSAGE_FOOTER_LEN = sizeof(MESSAGE_FOOTER) - 1;

// ============================================================================
// WiFi
// ============================================================================

static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:
        case WIFI_EVENT_STA_DISCONNECTED:
            esp_err_t err = esp_wifi_connect();
            if (err != ESP_OK)
            {
                ESP_LOGE(WIFI_TAG, "wifi_event_handler: esp_wifi_connect() returned %d", err);
                // Retry connecting
                esp_wifi_connect();
            }
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            break;
        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void setup_wifi(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &event_handler,
        NULL,
        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &event_handler,
        NULL,
        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Tarallini",
            .password = "itarallisonobuoni",
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK}};
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(WIFI_TAG, "wifi_init_sta finished.");
}

// ----------------------------------------------------------------------------
// Websocket
// ----------------------------------------------------------------------------

static EventGroupHandle_t s_websocket_event_group;
static const int WEBSOCKET_CONNECTED_BIT = BIT0;
static esp_websocket_client_handle_t ws_client;

static void websocket_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
        printf("Connected to websocket\n");
        xEventGroupSetBits(s_websocket_event_group, WEBSOCKET_CONNECTED_BIT);
        break;
    case WEBSOCKET_EVENT_CLOSED:
    case WEBSOCKET_EVENT_DISCONNECTED:
        printf("Disconnected from websocket\n");
        xEventGroupClearBits(s_websocket_event_group, WEBSOCKET_CONNECTED_BIT);
        break;
    default:
        break;
    }
}

static esp_err_t websocket_connect()
{
    const esp_websocket_client_config_t ws_cfg = {
        .uri = "ws://192.168.1.102",
        .port = 48000,
        .disable_auto_reconnect = false,
        .reconnect_timeout_ms = 1000,
    };

    printf("Connecting to websocket\n");
    ws_client = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(ws_client, WEBSOCKET_EVENT_ANY, websocket_event_handler, NULL);
    return esp_websocket_client_start(ws_client);
}

// ============================================================================
// I2S
// ============================================================================

static i2s_chan_handle_t tx_chan;
static i2s_chan_handle_t rx_chan;

// ----------------------------------------------------------------------------
// Pin definitions
// ----------------------------------------------------------------------------

// Amplifier:
const int PIN_WS = 32;
const int PIN_BCLK = 33;
const int PIN_DOUT = 25;

// Microphone:
const int MIC_PIN_WS = 26;
const int MIC_PIN_BCLK = 27;
const int MIC_PIN_DOUT = 25;

// ----------------------------------------------------------------------------
// Buffers
// ----------------------------------------------------------------------------

typedef int32_t in_sample;
typedef int16_t filtered_sample;

#define SAMPLE_RATE 24000
#define SAMPLE_BUFFER_LEN (SAMPLE_RATE / 100)
#define SAMPLE_BUFFER_SIZE (SAMPLE_BUFFER_LEN * sizeof(in_sample))
#define FILTERED_BUFFER_LEN (SAMPLE_BUFFER_LEN * sizeof(filtered_sample))

// Incoming samples have 18-bit data, but comes in 32-bit containers
DMA_ATTR int32_t in_samples[SAMPLE_BUFFER_LEN];

/*
 * Setup for the MAX98357A amplifier
 */
void setup_amp(void)
{
    i2s_chan_config_t i2s_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&i2s_chan_cfg, &tx_chan, NULL));

    i2s_std_config_t i2s_chan_0_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, // some codecs may require mclk signal, this example doesn't need it
            .bclk = PIN_BCLK,
            .ws = PIN_WS,
            .dout = PIN_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &i2s_chan_0_cfg));
    ESP_LOGI(I2S_TAG, "I2S initialized");
}

/*
 * Setup for the GY-SPH0645 I2S MEMS microphone
 */
void setup_mic(void)
{
    i2s_chan_config_t i2s_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&i2s_chan_cfg, NULL, &rx_chan));

    i2s_std_config_t i2s_chan_1_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_PIN_BCLK,
            .ws = MIC_PIN_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = MIC_PIN_DOUT,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &i2s_chan_1_cfg));
}

void recording()
{
    const float ALPHA = 0.9980403523506681;

    err_t err;
    size_t rx_bytes;

    // The last unfiltered value
    int16_t last_x_value;
    // The last filtered value
    int16_t last_y_value;
    // Signifies if the last values has been initialized
    bool filter_initialized = false;

    memcpy(
        message_buffer,
        MESSAGE_HEADER,
        MESSAGE_HEADER_LEN);

    /* Enable the RX channel */
    err = i2s_channel_enable(rx_chan);
    if (err != ERR_OK)
    {
        ESP_LOGE(I2S_TAG, "Failed to enable RX channel");
        vTaskDelete(NULL);
    }

    // We will reuse the same buffer for the filtered samples
    // As we consume 32-bit samples, and produce 16-bit samples, the two pointers shouldn't trump on each other
    int16_t *filtered_samples = (int16_t *)in_samples;
    while (1)
    {
        EventBits_t websocketStatusBits = xEventGroupWaitBits(s_websocket_event_group, WEBSOCKET_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        // If the websocket was not connected, we need to reinitialize the filter
        if ((websocketStatusBits & WEBSOCKET_CONNECTED_BIT) == 0)
        {
            printf("Websocket was reconnected, reinitializing websocket\n");
            // Wait for the bit
            // We will probably skip audio data incoming, so we need to reset the filter
            filter_initialized = false;
        }

        /* Read i2s data */
        if (i2s_channel_read(rx_chan, in_samples, SAMPLE_BUFFER_SIZE, &rx_bytes, 1000) == ESP_OK)
        {
            // If this is the first time we read into the buffer, we initialize the last values
            if (!filter_initialized)
            {
                last_x_value = (int16_t)(in_samples[0] >> 16);
                last_y_value = 0;
                filter_initialized = true;
            }

            for (int i = 0; i < SAMPLE_BUFFER_LEN; i++)
            {
                // We convert the 18-bit sample to a 16-bit one
                int16_t current_x_value = (int16_t)(in_samples[i] >> 16);
                // This is overflow safe
                int16_t current_y_value = (int16_t)(ALPHA * (last_y_value + (current_x_value - last_x_value)));
                // We update the last values
                last_x_value = current_x_value;
                last_y_value = current_y_value;

                // We write the filtered value back to the buffer
                filtered_samples[i] = current_y_value;
            }

            // Base64 encode the data
            size_t written_bytes;
            esp_crypto_base64_encode(
                //
                message_buffer + MESSAGE_HEADER_LEN,
                sizeof(message_buffer) - MESSAGE_HEADER_LEN - MESSAGE_FOOTER_LEN,
                &written_bytes,
                (uint8_t *)filtered_samples,
                FILTERED_BUFFER_LEN);
            memcpy(
                message_buffer + MESSAGE_HEADER_LEN + written_bytes,
                MESSAGE_FOOTER,
                MESSAGE_FOOTER_LEN);

            esp_websocket_client_send_text(
                ws_client,
                (char *)message_buffer,
                MESSAGE_HEADER_LEN + written_bytes + MESSAGE_FOOTER_LEN,
                portMAX_DELAY);

            // printf("%.*s\n", written_bytes, message_buffer + sizeof(MESSAGE_HEADER));
        }
        else
        {
            printf("Read Task: i2s read failed\n");
            vTaskDelete(NULL);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    s_websocket_event_group = xEventGroupCreate();
    s_wifi_event_group = xEventGroupCreate();

    setup_wifi();
    printf("WiFi setup done, waiting for connection\n");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    printf("Connected to WiFi\n");
    websocket_connect();

    // setup_amp();
    setup_mic();

    recording();
}