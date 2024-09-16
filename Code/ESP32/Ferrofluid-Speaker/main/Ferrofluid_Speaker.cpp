/*
    Inspiration for this came from Seung Hoon Jung's ferrofluid speaker.
    https://makezine.com/article/craft/fine-art/we-cant-stop-watching-this-diy-ferrofluid-bluetooth-speaker/

    Tested with ESP-ADF v2.7, ESP-IDF v5.3.1, ESP32-A2DP v1.8.3

    menuconfig
    Partition Table
        Partition Table
            Single factor app (large), no OTA
    Compiler options
        Optimization Level -> Optimize for performance (-O2)
    Component config
        Bluetooth
            Controller Options
                Bluetooth controller mode -> BR/EDR Only
            Bluedroid Options
                Classic Bluetooth -> A2DP & SPP
        Log output
            Default log verbosity -> No output

    Test Music:
        Artist: Tom Misch
        Album: Geography
        Song: Movie

        Artist: Metallica
        Album: 72 Seasons
        Song: Lux Æterna

        Artist: Ozzy Osbourne
        Album: Memoirs of a Madman
        Song: Crazy Train

        Artist: Queens of the Stone Age
        Album: Songs for the Deaf
        Song: No One Knows
*/

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_dsp.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "BluetoothA2DPSink.h"
#include "TAS5805M.hpp"
#include "STUSB4500.hpp"

// Vars for logging
const char *TAG = "Ferrofluid";
const int64_t OUTPUT_INTERVAL = 2000000; // Two seconds
int64_t last_output = 0;

// Setup aliases for GPIO pins that drive the electromagnet.
const gpio_num_t MAGNET = GPIO_NUM_5;

// Fast fourier transform variables.
const size_t NUM_FFT_SAMPLES = 4096 / 2 / 2; // See read_data_stream
static_assert(NUM_FFT_SAMPLES == 1024, "FFT lib requires 1024 samples.");
const float FFT_THRESHOLD = 0.2; // FFT magnitude has to be greater than this multiplied by by max value read.
// Window coefficients
__attribute__((aligned(16))) float fft_window[NUM_FFT_SAMPLES];
// working complex array
__attribute__((aligned(16))) float y_cf[NUM_FFT_SAMPLES * 2];
// Pointers to result arrays
float *fft_output_left = &y_cf[0];
float *fft_output_right = &y_cf[NUM_FFT_SAMPLES];

// Setup config constants for ESP's I2C communications.
const uint32_t I2C_FREQUENCY = 400000;
const i2c_port_t I2C_PORT = I2C_NUM_0;
const uint8_t AMP_ADDR = 0x2C;      // Amp's I2C address (8-bit write address: 0x58)
const uint8_t USB_CTRL_ADDR = 0x2B; // USB controller's I2C address (8-bit write address: 0x56)
const int I2C_XFER_TIMEOUT = -1;
const gpio_num_t I2C_GPIO_SCL = GPIO_NUM_19;
const gpio_num_t I2C_GPIO_SDA = GPIO_NUM_18;
i2c_master_bus_config_t i2c_mst_config = {
    .i2c_port = I2C_PORT,
    .sda_io_num = I2C_GPIO_SDA,
    .scl_io_num = I2C_GPIO_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,
    .trans_queue_depth = 0,
    .flags{.enable_internal_pullup = true}, // ESP32's pull-ups are only good for 100kHz; set to true to remove warning.
};
i2c_master_bus_handle_t bus_handle;

// Setup aliases for GPIO pins that communicate with the amp.
const gpio_num_t PDN = GPIO_NUM_23;
const gpio_num_t FAULT = GPIO_NUM_21;

// I2C interface variables for TAS5805M.
i2c_wrapper *i2c_tas5805m;
TAS5805M *amp;

// Constants for USB / power requirements.
const unsigned int M_VOLTS = 12000; // Millivolts
const unsigned int M_AMPS = 1000;   // Milliamps

// I2C interface variables to STUSB4500.
i2c_wrapper *i2c_usb_ctrler;
STUSB4500 *usb_ctrler;

BluetoothA2DPSink a2dp_sink;

#define CHECK_ERROR(error_code, ...)  \
    if (error_code != ESP_OK)         \
    {                                 \
        ESP_LOGE(TAG, ##__VA_ARGS__); \
        return error_code;            \
    }

void read_data_stream(const uint8_t *data, uint32_t length)
{
    // *data contains left and right channel data. First two bytes is left (int16_t).
    // Next two bytes is the right channel data. length contains the total number of
    // bytes.
    ESP_LOGD(TAG, "Length: %d", length);
    size_t sample_count = length / 2 / 2;
    if (sample_count != NUM_FFT_SAMPLES)
    {
        ESP_LOGE(TAG, "Sample Count: %d", sample_count);
        assert(sample_count == NUM_FFT_SAMPLES);
    }

    // Create a new pointer cast as 16 bits.
    int16_t *samples = (int16_t *)data;

    // Convert two input vectors to one complex vector
    for (int idx = 0; idx < NUM_FFT_SAMPLES; idx++)
    {
        y_cf[idx * 2 + 0] = float(samples[idx * 2]) * fft_window[idx];
        y_cf[idx * 2 + 1] = float(samples[idx * 2 + 1]) * fft_window[idx];
    }

    // Execute the FFT
    dsps_fft2r_fc32(y_cf, NUM_FFT_SAMPLES);
    // Bit reverse
    dsps_bit_rev_fc32(y_cf, NUM_FFT_SAMPLES);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(y_cf, NUM_FFT_SAMPLES);

    bool high_freq = false;
    for (int i = 0; i < NUM_FFT_SAMPLES / 2; i++)
    {
        // FFT returns complex numbers, where the first half of y_cf is the left channel and the second
        // half of y_cf is the right channel.
        //
        // Compute the normalized output so it can be compared with the maximum (current volume).
        float normalized_left = 2.0 * (sqrtf(fft_output_left[i * 2 + 0] * fft_output_left[i * 2 + 0] + fft_output_left[i * 2 + 1] * fft_output_left[i * 2 + 1]) / NUM_FFT_SAMPLES);
        float normalized_right = 2.0 * (sqrtf(fft_output_right[i * 2 + 0] * fft_output_right[i * 2 + 0] + fft_output_right[i * 2 + 1] * fft_output_right[i * 2 + 1]) / NUM_FFT_SAMPLES);

        // If the normalized value is above 63% of the max volume, turn on the magnet.
        int current_volume = a2dp_sink.get_volume();
        const float fft_threshold = (float(current_volume) / INT8_MAX) * (exp2(11) - 1) * 0.63;
        if (normalized_left > fft_threshold || normalized_right > fft_threshold)
        {
            ESP_LOGI(TAG, "Current volume: %d, Threshold: %f", current_volume, fft_threshold);
            ESP_LOGI(TAG, "%d-th freq (%dHZ) - Left: %f, Right: %f", i, (i * a2dp_sink.sample_rate()) / NUM_FFT_SAMPLES, fft_output_left[i], fft_output_right[i]);
            // ESP_LOGI(TAG, "Input Left: %d, Input Right: %d", samples[i * 2], samples[i * 2 + 1]);
            gpio_set_level(MAGNET, 1);
            high_freq = true;
            break;
        }
    }

    if (!high_freq)
    {
        gpio_set_level(MAGNET, 0);
    }
}

void i2s_state_change_pre(esp_a2d_audio_state_t state, void *obj)
{
    switch (state)
    {
    case ESP_A2D_AUDIO_STATE_STARTED:
        amp->start_pre_i2s();
        break;

    case ESP_A2D_AUDIO_STATE_SUSPEND:
        amp->stop();
        gpio_set_level(MAGNET, 0);
        break;

    default:
        break;
    }
}

void i2s_state_change_post(esp_a2d_audio_state_t state, void *obj)
{
    switch (state)
    {
    case ESP_A2D_AUDIO_STATE_STARTED:
        amp->start_post_i2s();
        break;

    case ESP_A2D_AUDIO_STATE_SUSPEND:
        amp->stop();

        gpio_set_level(MAGNET, 0);
        break;

    default:
        break;
    }
}

bool setup()
{
    esp_err_t result;

    // Setup I2C bus.
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Setup I2C for TAS5805M.
    i2c_tas5805m = new i2c_wrapper(&bus_handle, I2C_FREQUENCY, AMP_ADDR, I2C_XFER_TIMEOUT);
    amp = new TAS5805M(i2c_tas5805m, PDN, FAULT);

    // Setup I2C for STUSB4500.
    i2c_usb_ctrler = new i2c_wrapper(&bus_handle, I2C_FREQUENCY, USB_CTRL_ADDR, I2C_XFER_TIMEOUT);
    usb_ctrler = new STUSB4500(i2c_usb_ctrler);

    // Confirm power
    unsigned int m_volt;
    unsigned int m_amp;
    bool mismatch;

    result = usb_ctrler->read_power(&m_volt, &m_amp, &mismatch);
    CHECK_ERROR(result, "First power read failed: %s", esp_err_to_name(result));
    if (mismatch || m_volt != M_VOLTS || m_amp < M_AMPS)
    {
        ESP_LOGD(TAG, "setting up power requirements");
        result = usb_ctrler->set_power(M_VOLTS, M_AMPS);
        CHECK_ERROR(result, "Set power failed: %s", esp_err_to_name(result));

        result = usb_ctrler->read_power(&m_volt, &m_amp, &mismatch);
        CHECK_ERROR(result, "Second power read failed: %s", esp_err_to_name(result));
        if (mismatch || m_volt < M_VOLTS || m_amp < M_AMPS)
        {
            // Still do not have the required power. Stop powering up.
            ESP_LOGE(TAG, "Power mismatch");
            ESP_LOGE(TAG, "mismatch: %d", mismatch);
            ESP_LOGE(TAG, "m_volt: %d", m_volt);
            ESP_LOGE(TAG, "m_amp: %d", m_amp);
            return false;
        }
    }

    // Configure pin for magnet.
    gpio_config_t io_conf = {};
    // Set the bit mask for the output pins.
    io_conf.pin_bit_mask = ((1ULL << MAGNET));
    // Set all pins as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // Disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // Configure GPIO pins with the given settings
    result = gpio_config(&io_conf);
    CHECK_ERROR(result, "Could not configure output pins: %d", result);
    gpio_set_level(MAGNET, 0);
    /*gpio_set_level(LEFT_LED, 0);
    gpio_set_level(CENTER_LED, 0);
    gpio_set_level(RIGHT_LED, 0);*/

    // Initialize the FFT
    result = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    CHECK_ERROR(result, "Could not initialize FFT. Error = %i", result);
    // Generate hann window
    dsps_wind_hann_f32(fft_window, NUM_FFT_SAMPLES);

    // Configure Bluetooth (and I2S) library.
    a2dp_sink.set_stream_reader(read_data_stream);
    a2dp_sink.set_on_audio_state_changed(i2s_state_change_pre, NULL);
    a2dp_sink.set_on_audio_state_changed_post(i2s_state_change_post, NULL);
    // a2dp_sink.set_bits_per_sample(24);  // Bad things seem to happen when set to 24. The ESP can't keep up with the BT data.
    //  Enable Bluetooth which will also enable I2S (a2dp_sink.start)
    a2dp_sink.start("Ferrofluid");

    return true;
}

extern "C" void app_main(void)
{
    if (setup())
    {
        a2dp_sink.set_volume(UINT8_MAX / 2 / 2);
        while (true)
        {
            if (amp->error())
            {
                ESP_LOGE(TAG, "Amp's FAULT_PIN is active.");
                a2dp_sink.stop();
                break;
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);  // Check the amp for errors every half second.
        }
    }
}
