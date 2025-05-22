/**
 * Library for ADC
 * 
 * Includes functions for reading single or multiple channels.
 * Readings can be compared to chosen threshold and returns
 * corresponding bit value.
 * 
 * Used legacy driver adc.h, because after testing (reading ADC 
 * 100000 times and tracking time) it was found out that adc.h 
 * was a little faster than adc_oneshot.h (especially ADC2, which 
 * is used a lot).
 * 
 */


#ifndef ADC_LIB_H
#define ADC_LIB_H

// C/C++ libraries
#include <stdio.h>

// ESP-IDF libraries
#include "driver/adc.h"
#include "esp_timer.h"

// Personal libraries
#include "io_define.h"

// Structure to hold ADC configuration
typedef struct {
    adc1_channel_t *adc1_channels;
    int adc1_num_channels;
    adc_bits_width_t width;
    adc_atten_t atten;
} adc1_config_t;

typedef struct {
    adc2_channel_t *adc2_channels;
    int adc2_num_channels;
    adc_bits_width_t width;
    adc_atten_t atten;
} adc2_config_t;


/**
 * @brief Initialize ADC1
 * 
 * @param config    ADC1 configuration
 * 
 */
void adc1_lib_init(adc1_config_t *config);

/**
 * @brief Initialize ADC2
 * 
 * @param config    ADC2 configuration
 * 
 */
void adc2_lib_init(adc2_config_t *config);

/**
 * @brief Initialize ADC1 and ADC2
 * 
 * @param config1    ADC1 configuration
 * @param config2    ADC2 configuration
 * 
 */
void adc_lib_init_all(adc1_config_t *config1, adc2_config_t *config2);


/**
 * @brief Read ADC1 from chosen channel
 * 
 * @param channel   Chosen channel
 * @return Read value
 */
int adc1_lib_read(adc1_channel_t channel);

/**
 * @brief Read ADC2 from chosen channel
 * 
 * @param channel   Chosen channel
 * @return Read value
 */
int adc2_lib_read(adc2_channel_t channel);

/**
 * @brief Read all configured channels
 * 
 * @param adc1_results  Array for values from ADC1
 * @param adc2_results  Array for values from ADC2
 */
void adc_lib_read_all(int *adc1_results, int *adc2_results);

/**
 * @brief Read all configured channels and compare to set threshold
 * 
 * Corresponding bit values are added to the end by shifting
 * 
 * @param adc1_results  Array for values from ADC1
 * @param adc2_results  Array for values from ADC2
 * @param rx_buffer     Array for bit values
 * @param threshold     Chosen threshold
 */
void adc_lib_read_all_logical(int *adc1_results, int *adc2_results, int *rx_buffer, int threshold);


/**
 * @brief Initialize ADC1 for distance measuring (obstacle detecting)
 * 
 * @param config    ADC1 configuration
 * 
 */
void adc_lib_dis_init(adc1_config_t *config);

/**
 * @brief Read all configured channels
 * 
 * This is meant for distance measuring (obstacle detecting)
 * 
 * @param adc_dis_results  Array for values
 */
void adc_lib_dis_read_all(int *adc_dis_results);

#endif // ADC_LIB_H