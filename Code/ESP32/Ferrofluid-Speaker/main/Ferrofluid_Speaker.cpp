/*
    Inspiration for this came from Seung Hoon Jung's ferrofluid speaker.
    https://makezine.com/article/craft/fine-art/we-cant-stop-watching-this-diy-ferrofluid-bluetooth-speaker/

    Tested with ESP-IDF v4.4.3. Has not been updated for v5.x yet.

    menuconfig
    Partition Table
        Partition Table
            Single factor app (large), no OTA
    Compiler options
        Optimization Level -> Optimize for performance (-O2)
    Component config
        Bluetooth
            Bluetooth Controller
                Bluetooth controller mode -> BR/EDR Only
        Bluedroid Options
            Classic Bluetooth -> A2DP & SPP
        Log output
            Default log verbosity -> No output

    Test Music:
    Artist: Tom Misch
    Album: Geography
    Song: Movie

    Artist: Red Hot Chili Peppers
    Album: Blood Sugar Sex Magik
    Song: Give It Away Now

    Artist: Metallica
    Album: 72 Seasons
    Song: Lux Æterna
*/

#include <time.h>

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "driver/adc.h"

#include "BluetoothA2DPSink.h"
#include "fft.h"
#include "TAS5805M.hpp"
#include "STUSB4500.hpp"
#include "i2c-wrapper.hpp"

// Vars for logging
const char* TAG = "Ferrofluid";
const int64_t OUTPUT_INTERVAL = 2000000; // Two seconds
int64_t last_output = 0;

// Setup aliases for GPIO pins that drive the electromagnet.
const gpio_num_t MAGNET = GPIO_NUM_5;

// Fast fourier transform variables.
const size_t NUM_FFT_SAMPLES = 4096 / 2 / 2;  // See read_data_stream
const size_t STRIDE = 0;  // NUM_FFT_SAMPLES / 2;
const float FFT_THRESHOLD = 0.2;  // FFT magnitude has to be greater than this multiplied by by max value read.
//float FFT_THRESHOLD = 0.0;
float left_fft_buffer[STRIDE + NUM_FFT_SAMPLES];
float right_fft_buffer[STRIDE + NUM_FFT_SAMPLES];
float *where_to_write_left = left_fft_buffer;
float *where_to_write_right = right_fft_buffer;
float fft_output_left[NUM_FFT_SAMPLES];
float fft_output_right[NUM_FFT_SAMPLES];
fft_config_t *real_fft_plan_left;
fft_config_t *real_fft_plan_right;

// Potentiometer variables.
const long POT_POLL_DELAY = 1000 / 4;  // Poll and set the FFT threshold every 250 milliseconds, 4 times a second.
const adc1_channel_t POT_PIN = ADC1_CHANNEL_0;
const adc_atten_t ATTENUATION = ADC_ATTEN_DB_2_5;
const uint16_t MAX_ADC = 4095;

// Setup config constants for ESP's I2C communications.
const uint32_t I2C_FREQUENCY = 400000;
const i2c_port_t I2C_PORT = I2C_NUM_0;
const uint8_t AMP_WRITE_ADDR = 0x58;  // Amp's I2C write address
const uint8_t USB_CTRL_WRITE_ADDR = 0x56;  // USB controller's I2C write address.
const gpio_num_t I2C_GPIO_SCL = GPIO_NUM_19;
const gpio_num_t I2C_GPIO_SDA = GPIO_NUM_18;
i2c_wrapper i2c_interface(I2C_PORT, I2C_FREQUENCY, I2C_GPIO_SCL, I2C_GPIO_SDA);

// Setup aliases for GPIO pins that communicate with the amp.
const gpio_num_t PDN = GPIO_NUM_23;
const gpio_num_t FAULT = GPIO_NUM_21;
TAS5805M amp(&i2c_interface,
             AMP_WRITE_ADDR,
             PDN,
             FAULT);

// Constants for USB / power requirements.
const unsigned int M_VOLTS = 12000;  // Millivolts
const unsigned int M_AMPS = 1000;  // Milliamps

STUSB4500 usb_ctrler(&i2c_interface, USB_CTRL_WRITE_ADDR);

BluetoothA2DPSink a2dp_sink;

#define CHECK_ERROR(error_code, ...) if (error_code != ESP_OK) {ESP_LOGE(TAG, ##__VA_ARGS__); return error_code;}

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

    // Fill the FFT buffers.
    int16_t left_max = 0;
    int16_t right_max = 0;
    for (size_t idx = 0; idx < sample_count; idx++) {
        int16_t left_sample = samples[idx * 2];
        int16_t right_sample = samples[idx * 2 + 1];

        if (abs(left_sample) > left_max) {
            left_max = abs(left_sample);
        }

        if (abs(right_sample) > right_max) {
            right_max = abs(right_sample);
        }

        where_to_write_left[idx] = left_sample;
        where_to_write_right[idx] = right_sample;
    }

    fft_execute(real_fft_plan_left);
    fft_execute(real_fft_plan_right);

    // Shuffle the buffer down, dumping a stride's worth of samples.
    /*for (size_t idx = STRIDE; idx < NUM_FFT_SAMPLES; idx++) {
        fft_buffer[idx - STRIDE] = fft_buffer[idx];
    }
    if (where_to_write == fft_buffer) {
        where_to_write = &fft_buffer[STRIDE];
    }*/

    // const int16_t fft_threshold = ((a2dp_sink.get_volume() * exp2(11)) / (INT8_MAX + 1)) * 0.70;  // Works with Tom's music, but not other types of music.
    bool high_freq = false;
    float left_threshold = FFT_THRESHOLD * left_max;
    float right_threshold = FFT_THRESHOLD * right_max;
    for (int k = 1; k < real_fft_plan_left->size / 2; k++) {
        float mag_left = sqrtf(real_fft_plan_left->output[2*k] * real_fft_plan_left->output[2*k] + 
                               real_fft_plan_left->output[2*k+1] * real_fft_plan_left->output[2*k+1]) / NUM_FFT_SAMPLES;
        float mag_right = sqrtf(real_fft_plan_right->output[2*k] * real_fft_plan_right->output[2*k] + 
                                real_fft_plan_right->output[2*k+1] * real_fft_plan_right->output[2*k+1]) / NUM_FFT_SAMPLES;
        if (mag_left > left_threshold || mag_right > right_threshold) {
            gpio_set_level(MAGNET, 1);
            high_freq = true;

            if (current_time - last_output > OUTPUT_INTERVAL) {
                last_output = current_time;
                if (mag_right > left_threshold) {
                    ESP_LOGI(TAG, "Left max: %d", left_max);
                    ESP_LOGI(TAG, "Left: %d-th freq (%dHZ), mag: %f", k, (k * a2dp_sink.sample_rate()) / NUM_FFT_SAMPLES, mag_left);
                } else {
                    ESP_LOGI(TAG, "Right max: %d", right_max);
                    ESP_LOGI(TAG, "Right: %d-th freq (%dHZ), mag: %f", k, (k * a2dp_sink.sample_rate()) / NUM_FFT_SAMPLES, mag_right);
                }
            }
            break;
        }
    }
    if (!high_freq) {
        gpio_set_level(MAGNET, 0);
    }
}

void i2s_state_change_pre(esp_a2d_audio_state_t state, void *obj) {
    switch (state) {
        case ESP_A2D_AUDIO_STATE_STARTED:
            amp.start_pre_i2s();
            break;
            
        case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND:
        case ESP_A2D_AUDIO_STATE_STOPPED:
            amp.stop();
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

    // Confirm power
    unsigned int m_volt;
    unsigned int m_amp;
    bool mismatch;

    result = usb_ctrler.read_power(&m_volt, &m_amp, &mismatch);
    CHECK_ERROR(result, "First power read failed: %s", esp_err_to_name(result));
    if (mismatch || m_volt != M_VOLTS || m_amp < M_AMPS) {
        ESP_LOGD(TAG, "setting up power requirements");
        result = usb_ctrler.set_power(M_VOLTS, M_AMPS);
        CHECK_ERROR(result, "Set power failed: %s", esp_err_to_name(result));

        result = usb_ctrler.read_power(&m_volt, &m_amp, &mismatch);
        CHECK_ERROR(result, "Second power read failed: %s", esp_err_to_name(result));
        if (mismatch || m_volt < M_VOLTS || m_amp < M_AMPS) {
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
    io_conf.pin_bit_mask = ((1ULL << MAGNET)); // | (1ULL << LEFT_LED) | (1ULL << CENTER_LED) | (1ULL << RIGHT_LED));
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

    // Configure ADC for volume control.
    result = adc1_config_width(ADC_WIDTH_BIT_12);
    CHECK_ERROR(result, "Could not configure ADC width.");
    result = adc1_config_channel_atten(POT_PIN, ATTENUATION);
    CHECK_ERROR(result, "Could not configure ADC attenuation.");

    // Create the FFT config structure
    real_fft_plan_left = fft_init(NUM_FFT_SAMPLES, FFT_REAL, FFT_FORWARD, left_fft_buffer, fft_output_left);
    real_fft_plan_right = fft_init(NUM_FFT_SAMPLES, FFT_REAL, FFT_FORWARD, right_fft_buffer, fft_output_right);

    // Configure Bluetooth (and I2S) library.
    a2dp_sink.set_stream_reader(read_data_stream);
    a2dp_sink.set_on_audio_state_changed(i2s_state_change_pre, NULL);
    a2dp_sink.set_on_audio_state_changed_post(i2s_state_change_post, NULL);
    //a2dp_sink.set_bits_per_sample(24);  // Bad things seem to happen when set to 24. The ESP can't keep up with the BT data.
    // Enable Bluetooth which will also enable I2S (a2dp_sink.start)
    a2dp_sink.start("Ferrofluid");

    return true;
}

extern "C" void app_main(void) {
    if (setup()) {
        /*uint8_t previous_volume = 0;
        a2dp_sink.set_volume(previous_volume);*/
        a2dp_sink.set_volume(UINT8_MAX / 2);
        while (true) {
            /*int pot_raw = adc1_get_raw(POT_PIN);
            FFT_THRESHOLD = (pot_raw * 0x7FF) / MAX_ADC;
            ESP_LOGI(TAG, "pot_raw: %d, FFT_THRESHOLD: %f", pot_raw, FFT_THRESHOLD);
            //const uint8_t volume = (adc1_get_raw(POT_PIN) * 0xFF) / MAX_ADC;
            //if (volume != previous_volume) {
            //    a2dp_sink.set_volume(volume);
            //    previous_volume = volume;
            //}*/
            if (amp.error()) {
                ESP_LOGE(TAG, "Amp's FAULT_PIN is active.");
                a2dp_sink.stop();
                break;
            }
            delay(POT_POLL_DELAY);
        }
    }
}
