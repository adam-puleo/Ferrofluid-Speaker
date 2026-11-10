/*
    Inspiration for this came from Seung Hoon Jung's ferrofluid speaker.
    https://makezine.com/article/craft/fine-art/we-cant-stop-watching-this-diy-ferrofluid-bluetooth-speaker/

    Tested with ESP-ADF v2.8, ESP-IDF v5.5.5, ESP32-A2DP v1.8.12, arduino-audio-tools v1.3.0

    menuconfig
    Partition Table
        Partition Table
            Single factory app (large), no OTA
    Compiler options
        Optimization Level -> Optimize for performance (-O2)
    Component config
        Bluetooth (Enable Bluetooth)
            Bluedroid Options
                Classic Bluetooth -> A2DP & SPP
                Deselect -> Bluetooth Low Energy
            Controller Options
                Bluetooth controller mode -> BR/EDR Only
        Log
            Log Level
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

        Artist: Iron Maiden
        Album: Senjutsu
        Song: Senjutsu

        Artist: In This Moment
        Album: Blood
        Song: Blood
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_dsp.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include "TAS5805M.hpp"
#include "STUSB4500.hpp"

// Vars for logging
const char *TAG = "Ferrofluid";
const int64_t OUTPUT_INTERVAL = 2000000; // Two seconds
int64_t last_output = 0;

// Setup aliases for the GPIO pin that drives the electromagnet.
const gpio_num_t MAGNET = GPIO_NUM_5;

// ADC used to set PEAK_RATIO. ADC1 channel 0 is GPIO36 on the ESP32.
// Input range is 0 - 1.1V. 2.5dB attenuation is used because 0dB tops out around 950mV.
const adc_unit_t PEAK_RATIO_ADC_UNIT = ADC_UNIT_1;
const adc_channel_t PEAK_RATIO_ADC_CHANNEL = ADC_CHANNEL_0;
const adc_atten_t PEAK_RATIO_ADC_ATTEN = ADC_ATTEN_DB_2_5;
adc_oneshot_unit_handle_t adc_handle;
adc_cali_handle_t adc_cali_handle;

// Fast fourier transform variables.
const size_t NUM_FFT_SAMPLES = 4096 / 2 / 2; // See read_data_stream
static_assert(NUM_FFT_SAMPLES == 1024, "FFT size must match the 4096-byte A2DP buffer (1024 stereo frames).");
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
    .flags{.enable_internal_pullup = true, // ESP32's pull-ups are only good for 100kHz; set to true to remove warning.
           .allow_pd = false,
        },
};
i2c_master_bus_handle_t bus_handle;

// Setup aliases for GPIO pins that communicate with the amp.
const gpio_num_t PDN = GPIO_NUM_23;
const gpio_num_t FAULT = GPIO_NUM_21;
const gpio_num_t I2S_LRCLK = GPIO_NUM_25;
const gpio_num_t I2S_SCLK = GPIO_NUM_26;
const gpio_num_t I2S_DIN = GPIO_NUM_22;

// I2C interface variables for TAS5805M.
i2c_wrapper *i2c_tas5805m;
TAS5805M *amp;

// Constants for USB / power requirements.
const unsigned int M_VOLTS = 12000; // Millivolts
const unsigned int M_AMPS = 1000;   // Milliamps

// I2C interface variables to STUSB4500.
i2c_wrapper *i2c_usb_ctrler;
STUSB4500 *usb_ctrler;

// Setup the Bluetooth A2DP sink and I2S.
I2SStream i2s;
BluetoothA2DPSink a2dp_sink(i2s);

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
        ESP_LOGE(TAG, "Unexpected sample count: %d", sample_count);
        gpio_set_level(MAGNET, 0);
        return;
    }

    // Create a new pointer cast as 16 bits.
    int16_t *samples = (int16_t *)data;

    // FFT analysis lifted from https://github.com/espressif/esp-dsp/blob/master/examples/fft/main/dsps_fft_main.c
    // De-interleave the stereo frames into one complex vector (left in the real part,
    // right in the imaginary part) and apply the window function.
    for (int i = 0; i < NUM_FFT_SAMPLES; i++) {
        y_cf[i * 2 + 0] = float(samples[i * 2 + 0]) * fft_window[i];
        y_cf[i * 2 + 1] = float(samples[i * 2 + 1]) * fft_window[i];
    }

    // Execute the FFT
    dsps_fft2r_fc32(y_cf, NUM_FFT_SAMPLES);
    // Bit reverse
    dsps_bit_rev_fc32(y_cf, NUM_FFT_SAMPLES);
    // Convert one complex vector to two complex vectors
    dsps_cplx2reC_fc32(y_cf, NUM_FFT_SAMPLES);
    // FFT returns complex numbers, where the first half of y_cf is the left channel and the second
    // half of y_cf is the right channel.

    // Find the strongest bin and the average power across all bins, skipping DC (bin 0).
    const int NUM_BINS = NUM_FFT_SAMPLES / 2;
    float max_power = 0.0f;
    float total_power = 0.0f;
    int max_bin = 0;
    for (int i = 1; i < NUM_BINS; i++) {
        float pl = fft_output_left[i * 2 + 0] * fft_output_left[i * 2 + 0] + fft_output_left[i * 2 + 1] * fft_output_left[i * 2 + 1];
        float pr = fft_output_right[i * 2 + 0] * fft_output_right[i * 2 + 0] + fft_output_right[i * 2 + 1] * fft_output_right[i * 2 + 1];
        float p = pl > pr ? pl : pr;
        total_power += p;
        if (p > max_power)
        {
            max_power = p;
            max_bin = i;
        }
    }
    const float mean_power = total_power / (NUM_BINS - 1);

    // A bin is "dominant" when it holds far more power than the average bin. The ratio is
    // independent of volume (the A2DP library scales the samples before this callback).
    // PEAK_RATIO is set by the ADC voltage in millivolts (0 - 1100), so it can be tuned by ear.
    // MIN_POWER keeps silence/noise from triggering the magnet.
    int peak_ratio_mv = 0;
    esp_err_t adc_result = adc_oneshot_get_calibrated_result(adc_handle, adc_cali_handle, PEAK_RATIO_ADC_CHANNEL, &peak_ratio_mv);
    if (adc_result != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not read PEAK_RATIO ADC: %s", esp_err_to_name(adc_result));
        gpio_set_level(MAGNET, 0);
        return;
    }
    const float PEAK_RATIO = float(peak_ratio_mv);  // / 10.0f; // mean_power is consistently about 100 times less than max_power thus the /10.0f is not needed. It was used in early testing to make the numbers more readable.
    const float MIN_POWER = 1e6f;
    const bool dominant = max_power > MIN_POWER && max_power > PEAK_RATIO * mean_power;

    // Short-circuit so esp_timer_get_time is only called when info messages will be logged.
    if (LOG_LOCAL_LEVEL >= ESP_LOG_INFO && esp_log_level_get(TAG) >= ESP_LOG_INFO &&
        esp_timer_get_time() - last_output > OUTPUT_INTERVAL)
    {
        ESP_LOGI(
            TAG,
            "Dominant bin %d (%dHz), Peak ratio: %f, min power: %f, max power: %f, mean power: %f",
            max_bin,
            (max_bin * a2dp_sink.sample_rate()) / NUM_FFT_SAMPLES,
            PEAK_RATIO,
            MIN_POWER,
            max_power,
            mean_power
        );
        last_output = esp_timer_get_time();
    }

    gpio_set_level(MAGNET, dominant ? 1 : 0);
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

    // Configure the ADC used to set PEAK_RATIO.
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {};
    adc_init_cfg.unit_id = PEAK_RATIO_ADC_UNIT;
    result = adc_oneshot_new_unit(&adc_init_cfg, &adc_handle);
    CHECK_ERROR(result, "Could not initialize ADC unit: %s", esp_err_to_name(result));

    adc_oneshot_chan_cfg_t adc_chan_cfg = {};
    adc_chan_cfg.atten = PEAK_RATIO_ADC_ATTEN;
    adc_chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    result = adc_oneshot_config_channel(adc_handle, PEAK_RATIO_ADC_CHANNEL, &adc_chan_cfg);
    CHECK_ERROR(result, "Could not configure ADC channel: %s", esp_err_to_name(result));

    // The ESP32 only supports the line fitting calibration scheme.
    adc_cali_line_fitting_config_t adc_cali_cfg = {};
    adc_cali_cfg.unit_id = PEAK_RATIO_ADC_UNIT;
    adc_cali_cfg.atten = PEAK_RATIO_ADC_ATTEN;
    adc_cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    result = adc_cali_create_scheme_line_fitting(&adc_cali_cfg, &adc_cali_handle);
    CHECK_ERROR(result, "Could not create ADC calibration: %s", esp_err_to_name(result));

    // Initialize the FFT
    result = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    CHECK_ERROR(result, "Could not initialize FFT. Error = %i", result);
    // Generate Hann window
    dsps_wind_hann_f32(fft_window, NUM_FFT_SAMPLES);

    /* initialize NVS — it is used to store PHY calibration data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Configure Bluetooth (and I2S) library.
    auto cfg = i2s.defaultConfig();
    cfg.pin_bck = I2S_SCLK;
    cfg.pin_ws = I2S_LRCLK;
    cfg.pin_data = I2S_DIN;
    i2s.begin(cfg);

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
