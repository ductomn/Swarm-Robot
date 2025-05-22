#include "adc_lib.h"


// Global configuration structure
static adc1_config_t adc1_config;
static adc2_config_t adc2_config;

static int adc1_results[2];
static int adc2_results[4];


// Initialize ADC1 channels
void adc1_lib_init(adc1_config_t *config) {
    adc1_config = *config;

    // Configure ADC1
    adc1_config_width(adc1_config.width);
    for (int i = 0; i < adc1_config.adc1_num_channels; i++) {
        adc1_config_channel_atten(adc1_config.adc1_channels[i], adc1_config.atten);
    }
}

// Initialize ADC2 channels
void adc2_lib_init(adc2_config_t *config) {
    adc2_config = *config;

    // Configure ADC2
    for (int i = 0; i < adc2_config.adc2_num_channels; i++) {
        adc2_config_channel_atten(adc2_config.adc2_channels[i], adc2_config.atten);
    }
}

// Initialize all ADC channels
void adc_lib_init_all(adc1_config_t *config1, adc2_config_t *config2) {
    adc1_config = *config1;
    adc2_config = *config2;

    // Configure ADC1
    adc1_config_width(adc1_config.width);
    for (int i = 0; i < adc1_config.adc1_num_channels; i++) {
        adc1_config_channel_atten(adc1_config.adc1_channels[i], adc1_config.atten);
    }
    
    // Configure ADC2
    for (int i = 0; i < adc2_config.adc2_num_channels; i++) {
        adc2_config_channel_atten(adc2_config.adc2_channels[i], adc2_config.atten);
    }
}

// Read single ADC1 channel
int adc1_lib_read(adc1_channel_t channel) {
    return adc1_get_raw(channel);
}

// Read single ADC2 channel
int adc2_lib_read(adc2_channel_t channel) {
    int raw;
    adc2_get_raw(channel, adc2_config.width, &raw);
    return raw;
}

// Read all configured ADC channels
void adc_lib_read_all(int *adc1_results, int *adc2_results) {

    // Read ADC1 channels
    for (int i = 0; i < adc1_config.adc1_num_channels; i++) {
        adc1_results[i] = adc1_get_raw(adc1_config.adc1_channels[i]);
    }

    // Read ADC2 channels
    for (int i = 0; i < adc2_config.adc2_num_channels; i++) {
        esp_err_t err = adc2_get_raw(adc2_config.adc2_channels[i], adc2_config.width, &adc2_results[i]);
        if (err != ESP_OK) {
            adc2_results[i] = -1; // Assign an invalid value for debugging
        }
    }
}

// Read all configured ADC channels
void adc_lib_read_all_logical(int *adc1_results, int *adc2_results, int *rx_buffer, int threshold) {

    // Read ADC1 channels
    for (int i = 0; i < adc1_config.adc1_num_channels; i++) {
        adc1_results[i] = adc1_get_raw(adc1_config.adc1_channels[i]);
        if (adc1_results[i] >= threshold) rx_buffer[i] = (rx_buffer[i] << 1) | 1;
        else rx_buffer[i] = (rx_buffer[i] << 1) | 0;
    }

    // Read ADC2 channels
    for (int i = 0; i < adc2_config.adc2_num_channels; i++) {
        esp_err_t err = adc2_get_raw(adc2_config.adc2_channels[i], adc2_config.width, &adc2_results[i]);
        if (err != ESP_OK) {
            adc2_results[i] = -1; // Assign an invalid value for debugging
        }

        if (adc2_results[i] >= threshold) rx_buffer[i + adc1_config.adc1_num_channels] = (rx_buffer[i + adc1_config.adc1_num_channels] << 1) | 1;
        else rx_buffer[i + adc1_config.adc1_num_channels] = (rx_buffer[i + adc1_config.adc1_num_channels] << 1) | 0;
    }

}


// **************       Distance ADC       **************

// Global configuration structure
static adc1_config_t adc_dis_config;

void adc_lib_dis_init(adc1_config_t *config) {
    adc_dis_config = *config;

    // Configure ADC1
    adc1_config_width(adc_dis_config.width);
    for (int i = 0; i < adc_dis_config.adc1_num_channels; i++) {
        adc1_config_channel_atten(adc_dis_config.adc1_channels[i], adc_dis_config.atten);
    }
}

// Read all configured ADC_DIS channels
void adc_lib_dis_read_all(int *adc_dis_results) {

    // Read ADC1 channels
    for (int i = 0; i < adc_dis_config.adc1_num_channels; i++) {
        adc_dis_results[i] = adc1_get_raw(adc_dis_config.adc1_channels[i]);
    }

}