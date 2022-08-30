/*
    menuconfig
    Compiler options
        Optimization Level -> Optimize for performance (-O2)
    Component config
        Bluetooth
            Bluetooth Controller
                Bluetooth controller mode -> BR/EDR Only
            Bluedriod Options -> A2DP & SPP
        Log output
            Default -> No output
*/

#include <time.h>
#include <vector>

#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
const char* TAG = "Ferrofluid";
#include "esp_log.h"
#include "driver/adc.h"

#include "BluetoothA2DPSink.h"
#include "fft.h"
#include "TAS5805M.h"

BluetoothA2DPSink a2dp_sink;

// Setup aliases for GPIO pins that drive the electromagnet.
const gpio_num_t MAGNET = GPIO_NUM_5;

// Setup aliases for GPIO pins that drive the LEDs.
/*const gpio_num_t LEFT_LED = GPIO_NUM_14;
const gpio_num_t CENTER_LED = GPIO_NUM_27;
const gpio_num_t RIGHT_LED = GPIO_NUM_17;*/

// Fast fourier transform variables.
const adc1_channel_t FFT_THRESHOLD_PIN = ADC1_CHANNEL_0;
const adc_atten_t ATTENUATION = ADC_ATTEN_DB_2_5;
const uint16_t MAX_ADC = 4095;
//const uint16_t MAX_FFT = 255;
//const uint num_buckets = 7;
//const float FFT_THRESHOLD = 0.80; // 150.0;  // 100
const int64_t CALCULATION_INTERVAL = 1000; //125;  // Microseconds // 250
int64_t last_calculation = 0;
const int64_t OUTPUT_INTERVAL = 2000000; // Two seconds
int64_t last_output = 0;

// Setup config constants for ESP's I2C communication to the amp.
const uint32_t I2C_FREQUENCY = 400000;
const i2c_port_t I2C_PORT = I2C_NUM_0;
const uint8_t AMP_WRITE_ADDR = 0x58;  // I2C write address

// Setup aliases for GPIO pins that communicate with the amp.
const gpio_num_t PDN = GPIO_NUM_23;
const gpio_num_t FAULT = GPIO_NUM_21;
const gpio_num_t I2C_GPIO_SCL = GPIO_NUM_19;
const gpio_num_t I2C_GPIO_SDA = GPIO_NUM_18;

TAS5805M amp(AMP_WRITE_ADDR,
             I2C_PORT,
             I2C_FREQUENCY,
             PDN,
             FAULT,
             I2C_GPIO_SCL,
             I2C_GPIO_SDA);

// Compute the magnitude of each bucket.
std::vector<float> *frequency_magnitudes(float* fft_output, uint fft_size, float samples, uint num_buckets) {
    std::vector<float> *magnitudes = new std::vector<float>(num_buckets);
    for (uint idx = 1; idx < num_buckets; idx++) {
        magnitudes->at(idx - 1) = sqrtf(fft_output[2*idx] * fft_output[2*idx] + fft_output[2*idx+1] * fft_output[2*idx+1]) / samples;
    }

    return magnitudes;
}

// Compute the average magnitude of a target frequency window.
float window_average(const std::vector<float> *magnitudes) {
    float sum = 0.0;

    for (uint idx = 0; idx < magnitudes->size(); idx++) {
        sum += magnitudes->at(idx);
    }

    return sum / magnitudes->size();
}

float find_max(const std::vector<float> *magnitudes) {
    float max = 0.0;

    for (float idx = 0; idx < magnitudes->size(); idx++) {
        if (magnitudes->at(idx) > max) {
            max = magnitudes->at(idx);
        }
    }

    return max;
}

void read_data_stream(const uint8_t *data, uint32_t length) {
    int64_t current_time = esp_timer_get_time();
    if (current_time - last_calculation > CALCULATION_INTERVAL) {
        last_calculation = current_time;

        /*uint32_t sample_count = length / 2 / 2;
        int16_t *samples = (int16_t*) data;

        // Dump the left channel.
        printf("%d\n", sample_count);
        for (uint32_t idx = 0; idx < sample_count; idx++)
            printf("%f\n", (float)samples[idx * 2]);

        printf("data\n");
        for (uint32_t idx = 0; idx < length; idx=idx+2) {
            printf("%x %x\n", data[idx], data[idx+1]);
        }*/

        /*ESP_LOGI(TAG, "Data addr: %p", data);
        ESP_LOGI(TAG, "Data");
        ESP_LOG_BUFFER_HEXDUMP(TAG, data, 16, ESP_LOG_INFO);*/

        // *data contains left and right channel data. First two bytes is left (uint16_t).
        // Next two bytes is the right channel data. length contains the total number of
        // bytes.
        uint32_t sample_count = length / 2 / 2;
        // ESP_LOGI(TAG, "Length: %d", length);

        // Create a new pointer cast as 16 bits.
        int16_t *samples = (int16_t*) data;

        // Create the FFT config structure
        fft_config_t *real_fft_plan = fft_init(sample_count, FFT_REAL, FFT_FORWARD, NULL, NULL);

        uint16_t sample_rate = a2dp_sink.sample_rate();
        // ESP_LOGI(TAG, "bin_freq: %f", sample_rate / float(real_fft_plan->size));

        // Fill the FFT array with the average of the left and right channels and find the max sample.
        uint16_t max = 0;
        for (uint32_t idx = 0; idx < sample_count; idx++) {
            if (samples[idx * 2] > max) {
                max = samples[idx * 2];
            }
            if (samples[idx * 2 + 1] > max) {
                max = samples[idx * 2 + 1];
            }
            // Keep the arithmetic as integers for as long as possible for speed.
            // Divide each sample so there's no overflow on addition.
            real_fft_plan->input[idx] = (float) (samples[idx * 2] / 2 + samples[idx * 2 + 1] / 2);
        }

        /* ESP_LOGI(TAG, "Avg");
        for (int idx = 0; idx < 8; idx++)
            ESP_LOGI(TAG, "%f", real_fft_plan->input[idx]); */

        // Execute transformation
        fft_execute(real_fft_plan);

        /*// Now do something with the output
        ESP_LOGI(TAG, "DC component : %f", real_fft_plan->output[0] / real_fft_plan->size);  // DC is at [0]
        for (int k = 1; k < real_fft_plan->size / 2; k++) {
        // for (int k = 1; k < 20; k++) {
            ESP_LOGI(TAG, "%d-th freq (%dHZ) mag: %f", k, (k * sample_rate) / real_fft_plan->size, sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / real_fft_plan->size);
        }
        ESP_LOGI(TAG, "Middle component (%dHz): %f", sample_rate/2, real_fft_plan->output[1] / real_fft_plan->size);  // N/2 is real and stored at [1]

        // Fill array with the right channel.
        for (uint32_t idx = 0; idx < sample_count; idx++)
            real_fft_plan->input[idx] = (float)samples[idx * 2 + 1];

        ESP_LOGI(TAG, "Right");
        for (int idx = 0; idx < 8; idx++)
            ESP_LOGI(TAG, "%f", real_fft_plan->input[idx]);

        // Execute transformation
        fft_execute(real_fft_plan);

        // Now do something with the output
        ESP_LOGI(TAG, "DC component : %f", real_fft_plan->output[0] / real_fft_plan->size);  // DC is at [0]
        // for (int k = 1; k < real_fft_plan->size / 2; k++) {
        for (int k = 1; k < 20; k++) {
            ESP_LOGI(TAG, "%d-th freq (%dHZ) mag: %f", k, (k * sample_rate) / real_fft_plan->size, sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / real_fft_plan->size);
        }
        ESP_LOGI(TAG, "Middle component (%dHz): %f", sample_rate/2, real_fft_plan->output[1] / real_fft_plan->size);  // N/2 is real and stored at [1] */

        /*const uint bucket_size = real_fft_plan->size / 2 / num_magnets;
        float buckets[num_magnets];
        for (uint bucket = 0; bucket < num_magnets; bucket++) {
            uint idx_start = bucket * bucket_size + 1;  // DC component and mid-frequency are at IDX 0.
            uint idx_stop = idx_start + bucket_size;
            buckets[bucket] = window_average(real_fft_plan->output, 
                                             real_fft_plan->size, 
                                             idx_start, 
                                             idx_stop);
        }

        if (buckets[0] > FFT_THRESHOLD ||
            buckets[1] > FFT_THRESHOLD ||
            buckets[2] > FFT_THRESHOLD) {
            gpio_set_level(MAG_LEFT, 1);
        } else {
            gpio_set_level(MAG_LEFT, 0);
        }

        if (buckets[3] > FFT_THRESHOLD) {
            gpio_set_level(MAG_CENTER, 1);
        } else {
            gpio_set_level(MAG_CENTER, 0);
        }

        if (buckets[4] > FFT_THRESHOLD ||
            buckets[5] > FFT_THRESHOLD ||
            buckets[6] > FFT_THRESHOLD) {
            gpio_set_level(MAG_RIGHT, 1);
        } else {
            gpio_set_level(MAG_RIGHT, 0);
        }*/


        /*const uint bucket_size = real_fft_plan->size / 2 / num_buckets;
        bool high_freq = false;
        for (int k = 1; k <= bucket_size; k++) {
            if (sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / real_fft_plan->size > FFT_THRESHOLD) {
                gpio_set_level(MAG_LEFT, 1);
                high_freq = true;
                break;
            }
        }
        if (high_freq) {
            high_freq = false;
        } else {
            gpio_set_level(MAG_LEFT, 0);
        }        

        for (int k = bucket_size + 1; k <= bucket_size + bucket_size; k++) {
            if (sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / real_fft_plan->size > FFT_THRESHOLD) {
                gpio_set_level(MAG_CENTER, 1);
                high_freq = true;
                break;
            }
        }
        if (high_freq) {
            high_freq = false;
        } else {
            gpio_set_level(MAG_CENTER, 0);
        }        

        for (int k = bucket_size + bucket_size + 1; k <= bucket_size + bucket_size + bucket_size; k++) {
            if (sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / real_fft_plan->size > FFT_THRESHOLD) {
                gpio_set_level(MAG_RIGHT, 1);
                high_freq = true;
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(MAG_RIGHT, 0);
        }*/

        /*const uint bucket_size = real_fft_plan->size / 2 / num_buckets;
        const uint16_t fft_threshold = (adc1_get_raw(FFT_THRESHOLD_PIN) * MAX_FFT) / MAX_ADC;
        bool high_freq = false;
        for (int k = 1; k <= bucket_size; k++) {
            if (sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / real_fft_plan->size > fft_threshold) {
                gpio_set_level(MAG_LEFT, 1);
                high_freq = true;
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(MAG_LEFT, 0);
        }*/
        
        /*if (current_time - last_output > OUTPUT_INTERVAL) {
            last_output = current_time;
            ESP_LOGI(TAG, "Size: %d, Rate: %d", real_fft_plan->size, sample_rate);
        }
        const uint SIXTY_THREE_HZ_BUCKET = (63 * real_fft_plan->size) / sample_rate;
        const uint ONE_SIXTY_HZ_BUCKET = (160 * real_fft_plan->size) / sample_rate;
        const uint FOUR_HUNDRED_HZ_BUCKET = (400 * real_fft_plan->size) / sample_rate;
        const uint ONE_K_HZ_BUCKET = (1000 * real_fft_plan->size) / sample_rate;
        const uint TWENTY_FIVE_K_HZ_BUCKET = (2500 * real_fft_plan->size) / sample_rate;
        const uint SIXTY_TWO_FIFTY_HZ_BUCKET = (6250 * real_fft_plan->size) / sample_rate;
        const uint SIXTEEN_K_HZ_BUCKET = (16000 * real_fft_plan->size) / sample_rate;
        const uint FREQS_TO_SAMPLE[] = {SIXTY_THREE_HZ_BUCKET, ONE_SIXTY_HZ_BUCKET, FOUR_HUNDRED_HZ_BUCKET, ONE_K_HZ_BUCKET,
                                        TWENTY_FIVE_K_HZ_BUCKET, SIXTY_TWO_FIFTY_HZ_BUCKET, SIXTEEN_K_HZ_BUCKET};
        const uint NUM_BUCKETS_SIDE = 50;
        const uint16_t fft_threshold = (adc1_get_raw(FFT_THRESHOLD_PIN) * MAX_FFT) / MAX_ADC;
        bool high_freq = false;
        for (uint freq_bucket_idx = 0; freq_bucket_idx < sizeof(FREQS_TO_SAMPLE) / sizeof(FREQS_TO_SAMPLE[0]); freq_bucket_idx++) {
            if (NUM_BUCKETS_SIDE <= FREQS_TO_SAMPLE[freq_bucket_idx] && FREQS_TO_SAMPLE[freq_bucket_idx] <= real_fft_plan->size - NUM_BUCKETS_SIDE) {
                for (uint k = FREQS_TO_SAMPLE[freq_bucket_idx] - NUM_BUCKETS_SIDE; k <= FREQS_TO_SAMPLE[freq_bucket_idx] - NUM_BUCKETS_SIDE; k++)
                for (uint k = 1; k <= 461; k++) {
                    if (sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / real_fft_plan->size > fft_threshold) {
                        gpio_set_level(MAG_LEFT, 1);
                        high_freq = true;
                        break;
                }
            }
            if (high_freq) {
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(MAG_LEFT, 0);
        }*/

        /*// These buckets are FFT indexes. I use them to index magnitudes which is shifted by one so I'm
        // off by one bucket. Close enough though.
        const uint ONE_TWENTY_FIVE_HZ_BUCKET = (125 * real_fft_plan->size) / sample_rate;
        const uint TWO_K_HZ_BUCKET = (2000 * real_fft_plan->size) / sample_rate;
        const uint SIXTEEN_K_HZ_BUCKET = (16000 * real_fft_plan->size) / sample_rate;
        std::vector<float> *magnitudes = frequency_magnitudes(real_fft_plan->output, real_fft_plan->size, sample_count, SIXTEEN_K_HZ_BUCKET);
        const float fft_threshold = FFT_THRESHOLD * (a2dp_sink.get_volume() / 255.0) * max;
        if (current_time - last_output > OUTPUT_INTERVAL) {
            last_output = current_time;
            ESP_LOGI(TAG, "max input: %d, max: %f, threshold: %f, volume: %d", max, find_max(magnitudes), fft_threshold, a2dp_sink.get_volume());
            ESP_LOGI(TAG, "Buckets: %d, %d, %d", ONE_TWENTY_FIVE_HZ_BUCKET, TWO_K_HZ_BUCKET, SIXTEEN_K_HZ_BUCKET);
        }
        //const uint16_t fft_threshold = (adc1_get_raw(FFT_THRESHOLD_PIN) * MAX_FFT) / MAX_ADC;
        bool high_freq = false;
        for (uint k = 0; k < ONE_TWENTY_FIVE_HZ_BUCKET; k++) {  // Low frequencies
            if (magnitudes->at(k) > fft_threshold) {
                gpio_set_level(MAG_CENTER, 1);
                high_freq = true;
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(MAG_CENTER, 0);
        }
        for (uint k = ONE_TWENTY_FIVE_HZ_BUCKET; k < TWO_K_HZ_BUCKET; k++) {  // Mid frequencies
            if (magnitudes->at(k) > fft_threshold) {
                gpio_set_level(MAG_LEFT, 1);
                high_freq = true;
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(MAG_LEFT, 0);
        }
        for (uint k = TWO_K_HZ_BUCKET; k < SIXTEEN_K_HZ_BUCKET; k++) {
            if (magnitudes->at(k) > fft_threshold) {
                gpio_set_level(MAG_RIGHT, 1);
                high_freq = true;
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(MAG_RIGHT, 0);
        }*/

        uint TWENTY_K_HZ_BUCKET = (20000 * real_fft_plan->size) / sample_rate;
        if (TWENTY_K_HZ_BUCKET > real_fft_plan->size / 2) {
            // Cap at half the Nyquist frequency.
            TWENTY_K_HZ_BUCKET = real_fft_plan->size / 2;
        }
        const uint16_t fft_threshold = (adc1_get_raw(FFT_THRESHOLD_PIN) * max) / MAX_ADC;
        bool high_freq = false;
        for (int k = 1; k <= TWENTY_K_HZ_BUCKET; k++) {
            if (sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / sample_count > fft_threshold) {
                gpio_set_level(MAGNET, 1);
                high_freq = true;
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(MAGNET, 0);
        }

        /*const uint ONE_TWENTY_FIVE_HZ_BUCKET = (125 * real_fft_plan->size) / sample_rate;
        const uint TWO_K_HZ_BUCKET = (2000 * real_fft_plan->size) / sample_rate;
        uint TWENTY_K_HZ_BUCKET = (20000 * real_fft_plan->size) / sample_rate;
        if (TWENTY_K_HZ_BUCKET > real_fft_plan->size / 2) {
            // Cap at half the Nyquist frequency.
            TWENTY_K_HZ_BUCKET = real_fft_plan->size / 2;
        }
        const uint16_t fft_threshold = (adc1_get_raw(FFT_THRESHOLD_PIN) * max) / MAX_ADC;
        bool high_freq = false;
        bool magnet_on = false;
        for (uint k = 1; k < ONE_TWENTY_FIVE_HZ_BUCKET; k++) {  // Low frequencies
            if (sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / sample_count > fft_threshold) {
                gpio_set_level(MAGNET, 1);
                magnet_on = true;
                gpio_set_level(LEFT_LED, 1);
                high_freq = true;
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(LEFT_LED, 0);
        }
        for (uint k = ONE_TWENTY_FIVE_HZ_BUCKET; k < TWO_K_HZ_BUCKET; k++) {  // Mid frequencies
            if (sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / sample_count > fft_threshold) {
                gpio_set_level(MAGNET, 1);
                magnet_on = true;
                gpio_set_level(CENTER_LED, 1);
                high_freq = true;
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(CENTER_LED, 0);
        }
        for (uint k = TWO_K_HZ_BUCKET; k < TWENTY_K_HZ_BUCKET; k++) {  // High frequencies
            if (sqrtf(real_fft_plan->output[2*k] * real_fft_plan->output[2*k] + real_fft_plan->output[2*k+1] * real_fft_plan->output[2*k+1]) / sample_count > fft_threshold) {
                gpio_set_level(MAGNET, 1);
                magnet_on = true;
                gpio_set_level(RIGHT_LED, 1);
                break;
            }
        }
        if (!high_freq) {
            gpio_set_level(RIGHT_LED, 0);
        }
        if (!magnet_on) {
            gpio_set_level(MAGNET, 0);
        }*/

        // Don't forget to clean up at the end to free all the memory that was allocated
        //magnitudes->clear();
        //delete magnitudes;
        fft_destroy(real_fft_plan);
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
