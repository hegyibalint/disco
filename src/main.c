// Board: AI Thinker NodeMCU-32S
// Amplifier: Adafruit MAX98357A

#include <math.h>
#include <freertos/FreeRTOS.h>
#include <soc/i2s_reg.h>
#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <sys/socket.h>

static const char *WIFI_TAG = "WiFi";
static const char *I2S_TAG = "I2S";

// ============================================================================
// WiFi
// ============================================================================

static SemaphoreHandle_t s_wifi_connected;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
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
            break;
        default:
            break;
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xSemaphoreGive(s_wifi_connected);
    }
}

void setup_wifi(void)
{
    xSemaphoreTake(s_wifi_connected, 0);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    s_wifi_connected = xSemaphoreCreateBinary();
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

// ============================================================================
// I2S
// ============================================================================

typedef int32_t i2s_sample_t;
static i2s_chan_handle_t tx_chan;
static i2s_chan_handle_t rx_chan;

const int FREQUENCY = 440; // Hz

// Amplifier:
const int PIN_WS = 32;
const int PIN_BCLK = 33;
const int PIN_DOUT = 25;

// Microphone:
const int MIC_PIN_WS = 26;
const int MIC_PIN_BCLK = 27;
const int MIC_PIN_DOUT = 14;

// Buffers:
#define SAMPLE_RATE 48000
#define AUDIO_BUFFER_LEN (SAMPLE_RATE / 100)
#define AUDIO_BUFFER_SIZE (AUDIO_BUFFER_LEN * sizeof(i2s_sample_t))
// Currently, the RX and TX buffers are the same size

static DMA_ATTR i2s_sample_t rx_buffer[AUDIO_BUFFER_LEN];
static DMA_ATTR i2s_sample_t tx_buffer[AUDIO_BUFFER_LEN];

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

void send_sinewave()
{
    // Make a simple square wave
    const size_t buf_len = 160;
    i2s_sample_t *buf = malloc(buf_len * sizeof(i2s_sample_t));
    for (size_t i = 0; i < buf_len; i++)
    {
        buf[i] = (i2s_sample_t)sin(2 * M_PI * FREQUENCY * i / (buf_len * 4)) * INT32_MAX / 512;
    }

    /* Enable the TX channel */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    size_t bytes_written;
    while (1)
    {
        /* Write i2s data */
        if (i2s_channel_write(tx_chan, buf, buf_len * sizeof(i2s_sample_t), &bytes_written, 1000) != ESP_OK)
        {
            printf("Write Task: i2s write failed\n");
        }
    }
    free(buf);
    vTaskDelete(NULL);
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

void recording(void *arg)
{
    const float ALPHA = 0.9980403523506681;

    err_t err;
    size_t rx_bytes;

    // The last unfiltered value
    int32_t last_x_value;
    // The last filtered value
    int32_t last_y_value;
    // Signifies if the last values has been initialized
    bool filter_initialized = false;

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(I2S_TAG, "Failed to create socket");
        vTaskDelete(NULL);
    }

    // Packet address is 192.168.1.103
    struct sockaddr_in receiver_addr = {0};
    receiver_addr.sin_family = AF_INET;
    receiver_addr.sin_port = htons(48000);
    inet_pton(AF_INET, "192.168.1.103", &(receiver_addr.sin_addr));

    // Connect to the server
    err = connect(sock, (struct sockaddr *)&receiver_addr, sizeof(receiver_addr));
    if (err < 0)
    {
        printf("Failed to connect to server, err: %d\n", err);
        vTaskDelete(NULL);
    }

    /* Enable the RX channel */
    err = i2s_channel_enable(tx_chan);
    if (err != ERR_OK)
    {
        ESP_LOGE(I2S_TAG, "Failed to enable TX channel");
        vTaskDelete(NULL);
    }
    err = i2s_channel_enable(rx_chan);
    if (err != ERR_OK)
    {
        ESP_LOGE(I2S_TAG, "Failed to enable RX channel");
        vTaskDelete(NULL);
    }

    // We do one dummy read to make sure the microphone is ready
    err = i2s_channel_read(rx_chan, rx_buffer, AUDIO_BUFFER_SIZE, &rx_bytes, 1000);
    if (err != ERR_OK)
    {
        ESP_LOGE(I2S_TAG, "Failed to read from RX channel");
        vTaskDelete(NULL);
    }

    while (1)
    {
        /* Read i2s data */
        if (i2s_channel_read(rx_chan, rx_buffer, AUDIO_BUFFER_SIZE, &rx_bytes, 1000) == ESP_OK)
        {
            // If this is the first time we read into the buffer, we initialize the last values
            if (!filter_initialized)
            {
                last_x_value = rx_buffer[0];
                last_y_value = 0;
                filter_initialized = true;
            }
            for (int i = 0; i < AUDIO_BUFFER_LEN; i++)
            {
                // We save the current unfiltered value
                int32_t current_x_value = rx_buffer[i];
                // We replace it by the filtered value
                int32_t current_y_value = (int32_t)(ALPHA * (last_y_value + (current_x_value - last_x_value)));
                last_x_value = current_x_value;
                last_y_value = current_y_value;
                tx_buffer[i] = current_y_value;
            }

            err = i2s_channel_write(tx_chan, tx_buffer, AUDIO_BUFFER_SIZE, &rx_bytes, 1000);
            if (err != ERR_OK)
            {
                printf("Write Task: i2s write failed\n");
            }

            // Send the data to the server
            send(sock, tx_buffer, AUDIO_BUFFER_SIZE, 0);
        }
        else
        {
            printf("Read Task: i2s read failed\n");
            vTaskDelete(NULL);
        }
    }
    free(rx_buffer);
    vTaskDelete(NULL);
}

void app_main(void)
{
    s_wifi_connected = xSemaphoreCreateBinary();
    setup_wifi();
    printf("WiFi setup done, waiting for connection\n");
    xSemaphoreTake(s_wifi_connected, portMAX_DELAY);
    printf("Connected to WiFi\n");

    setup_amp();
    setup_mic();

    // xTaskCreate(send_sinewave, "send_sinewave", 2048, NULL, 5, NULL);
    xTaskCreate(recording, "recording", 2048, NULL, 5, NULL);
}