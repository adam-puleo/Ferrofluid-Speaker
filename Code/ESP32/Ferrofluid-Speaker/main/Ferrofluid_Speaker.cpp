/*
    menuconfig
    Compiler options
        Optimization Level -> Optimize for performance (-O2)
    Component config
        Bluetooth
            Bluetooth Controller
                Bluetooth controller mode -> BR/EDR Only
        Bluedroid Options
            Classic Bluetooth -> A2DP & SPP
        Log output
            Default -> No output

    Test Music:
    Artist: Tom Misch
    Album: Geography
    Song: Movie
*/

#include <time.h>

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "driver/adc.h"

#include "BluetoothA2DPSink.h"
#include "fft.h"
#include "TAS5805M.hpp"

// Vars for logging
const char* TAG = "Ferrofluid";
const int64_t OUTPUT_INTERVAL = 2000000; // Two seconds
int64_t last_output = 0;

// Setup aliases for GPIO pins that drive the electromagnet.
const gpio_num_t MAGNET = GPIO_NUM_5;

// Fast fourier transform variables.
const adc1_channel_t FFT_THRESHOLD_PIN = ADC1_CHANNEL_0;
const adc_atten_t ATTENUATION = ADC_ATTEN_DB_2_5;
const uint16_t MAX_ADC = 4095;
int16_t max_sample = 0;
const size_t NUM_FFT_SAMPLES = 4096 / 2 / 2;  // See read_data_stream
const size_t STRIDE = NUM_FFT_SAMPLES / 4;
float fft_buffer[STRIDE + NUM_FFT_SAMPLES];
float *where_to_write = fft_buffer;
float fft_output[NUM_FFT_SAMPLES];
fft_config_t *real_fft_plan;

// Setup config constants for ESP's I2C communication to the amp.
const uint32_t I2C_FREQUENCY = 400000;
const i2c_port_t I2C_PORT = I2C_NUM_0;
const uint8_t AMP_WRITE_ADDR = 0x58;  // I2C write address

// Setup aliases for GPIO pins that communicate with the amp.
const gpio_num_t PDN = GPIO_NUM_23;
const gpio_num_t FAULT = GPIO_NUM_21;
const gpio_num_t I2C_GPIO_SCL = GPIO_NUM_19;
const gpio_num_t I2C_GPIO_SDA = GPIO_NUM_18;

BluetoothA2DPSink a2dp_sink;

TAS5805M amp(AMP_WRITE_ADDR,
             I2C_PORT,
             I2C_FREQUENCY,
             PDN,
             FAULT,
             I2C_GPIO_SCL,
             I2C_GPIO_SDA);

void read_data_stream(const uint8_t *data, uint32_t length) {
    int64_t current_time = esp_timer_get_time();

    // *data contains left and right channel data. First two bytes is left (int16_t).
    // Next two bytes is the right channel data. length contains the total number of
    // bytes.
    // ESP_LOGI(TAG, "Length: %d", length);
    size_t sample_count = length / 2 / 2;
    if (sample_count != NUM_FFT_SAMPLES) {
        ESP_LOGE(TAG, "Sample Count: %d", sample_count);
        assert(sample_count == NUM_FFT_SAMPLES);
    }

    // Create a new pointer cast as 16 bits.
    int16_t *samples = (int16_t*) data;

    // Fill the FFT buffer with the largest of the left or right channel.
    // While we are here, find the max sample.
    max_sample = 0;  // Reset
    for (size_t idx = 0; idx < sample_count; idx++) {
        int16_t left_sample = samples[idx * 2];
        int16_t right_sample = samples[idx * 2 + 1];

        if (abs(left_sample > max_sample)) {
            max_sample = left_sample;
        } else if (abs(right_sample > max_sample)) {
            max_sample = right_sample;
        }

        // Grab the largest sample otherwise everything becomes muted.
        if (abs(left_sample) > abs(right_sample)) {
            where_to_write[idx] = left_sample;
        } else {
            where_to_write[idx] = right_sample;
        }
    }

    fft_execute(real_fft_plan);

    // Shuffle the buffer down, dumping a stride's worth of samples.
    for (size_t idx = STRIDE; idx < NUM_FFT_SAMPLES; idx++) {
        fft_buffer[idx - STRIDE] = fft_buffer[idx];
    }
    if (where_to_write == fft_buffer) {
        where_to_write = &fft_buffer[STRIDE];
    }

    const uint16_t fft_threshold = (adc1_get_raw(FFT_THRESHOLD_PIN) * max_sample) / MAX_ADC;
    // const int16_t fft_threshold = ((a2dp_sink.get_volume() * exp2(11)) / (INT8_MAX + 1)) * 0.70;  // Works with Tom's music, but not other types of music.
    float max_mag = 0.0;
    int max_mag_bucket = 0; 
    bool high_freq = false;
    for (int k = 1; k < real_fft_plan->size / 2; k++) {
        float mag = sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + 
                          real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / NUM_FFT_SAMPLES / 2;
        if (mag > max_mag) {
            max_mag = mag;
            max_mag_bucket = k;
        }
        if (mag > fft_threshold) {
            gpio_set_level(MAGNET, 1);
            high_freq = true;

            if (current_time - last_output > OUTPUT_INTERVAL) {
                ESP_LOGI(TAG, "%d-th freq (%dHZ), mag: %f", k, (k * a2dp_sink.sample_rate()) / NUM_FFT_SAMPLES, mag);
            }
            break;
        }
    }
    if (!high_freq) {
        gpio_set_level(MAGNET, 0);
    }
    if (current_time - last_output > OUTPUT_INTERVAL) {
        last_output = current_time;
        ESP_LOGI(TAG, "max input: %d, threshold: %d, max mag: %f, mag freq: %dHZ", max_sample, fft_threshold, max_mag, (max_mag_bucket * a2dp_sink.sample_rate()) / NUM_FFT_SAMPLES);
    }
}

void i2s_state_change_pre(esp_a2d_audio_state_t state, void *obj) {
    switch (state) {
        case ESP_A2D_AUDIO_STATE_STARTED:
            amp.start_pre_i2s();
            break;
            
        case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
        case ESP_A2D_AUDIO_STATE_STOPPED:
            /* code */
            break;
        
        default:
            break;
    }
}

void i2s_state_change_post(esp_a2d_audio_state_t state, void *obj) {
    switch (state) {
        case ESP_A2D_AUDIO_STATE_STARTED:
            amp.start_post_i2s();
            break;
            
        case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
        case ESP_A2D_AUDIO_STATE_STOPPED:
            amp.stop();

            gpio_set_level(MAGNET, 0);
            /*gpio_set_level(LEFT_LED, 0);
            gpio_set_level(CENTER_LED, 0);
            gpio_set_level(RIGHT_LED, 0);*/
            break;
        
        default:
            break;
    }
}

bool setup() {
    esp_err_t result;

    gpio_config_t io_conf = {};
    // Set the bit mask for the output pins. 
    io_conf.pin_bit_mask = ((1ULL << MAGNET)); // | (1ULL << LEFT_LED) | (1ULL << CENTER_LED) | (1ULL << RIGHT_LED));
    // Set all pins as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // Disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // Configure GPIO pins with the given settings
    result = gpio_config(&io_conf);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: Could not configure output pins: %d", result);
        return false;
    }
    gpio_set_level(MAGNET, 0);
    /*gpio_set_level(LEFT_LED, 0);
    gpio_set_level(CENTER_LED, 0);
    gpio_set_level(RIGHT_LED, 0);*/

    // Configure ADC
    result = adc1_config_width(ADC_WIDTH_12Bit);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: Could not configure ADC width.");
        return false;
    }
    result = adc1_config_channel_atten(FFT_THRESHOLD_PIN, ATTENUATION);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "ERROR: Could not configure ADC attenuation.");
        return false;
    }

    // Create the FFT config structure
    real_fft_plan = fft_init(NUM_FFT_SAMPLES, FFT_REAL, FFT_FORWARD, fft_buffer, fft_output);

    // Configure Bluetooth (and I2S) library.
    a2dp_sink.set_stream_reader(read_data_stream);
    a2dp_sink.set_on_audio_state_changed(i2s_state_change_pre, NULL);
    a2dp_sink.set_on_audio_state_changed_post(i2s_state_change_post, NULL);
    //a2dp_sink.set_bits_per_sample(24);  // Bad things seem to happen when set to 24. The ESP can't keep up with the BT data.
    // Enable Bluetooth which will also enables I2S (a2dp_sink.start)
    a2dp_sink.start("Ferrofluid");

    return true;
}

extern "C" void app_main(void) {
    if (setup()) {
        while (true) {
            //ESP32_LOGI(TAG, "FFT PIN: %d", gpio_get_level(FFT_THRESHOLD_PIN));
            if (amp.error()) {
                a2dp_sink.stop();
                break;
            }
            delay(2000);
        }
    }
}
