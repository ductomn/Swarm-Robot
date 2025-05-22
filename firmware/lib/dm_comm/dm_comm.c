#include "dm_comm.h"

static int timer_comm = 1;

// timing
static long long int time1_last;

// LED and ADC setup
static adc1_channel_t *adc1_channels;
static adc2_channel_t *adc2_channels;
static int adc1_size, adc2_size;
static gpio_num_t *led_pins;
static int led_size;

// Arrays to decode data
static int adc1_results[2];
static int adc2_results[4];

static int rx_buffer[CHANNEL_NUM] = {0};
static int msg[CHANNEL_NUM] = {0};
static int tx_buffer;

// Flags
static uint8_t tx_bit_index;
static uint8_t sending = 0;
static uint8_t reading = 1;
static uint8_t read_flag;

static uint8_t rx_count[CHANNEL_NUM] = {0};
static bool start_detected[CHANNEL_NUM] = {0};

// Flags for CSMA/CA
static uint8_t channel_occupied = 0;  // Flag for busy channel
static uint8_t backoff_active = 0;    // Flag for backoff state
static int backoff_countdown = 0;     // Backoff time (random)

// Array for coop.h
static int sig_adc1_results[2];
static int sig_adc2_results[4];


static void timer1_callback() {

    #if WITH_LEADER
        //#if !LEADER 

        if(!reading) return;
        
        multiple_led_drive(led_pins, led_size, 0);
        
        if(reading){    
            adc_lib_read_all_logical(adc1_results, adc2_results, rx_buffer, SIG_THRESHOLD);
            
            for (int i = 0; i < CHANNEL_NUM; i++) rx_count[i]++;
            //rx_count++;
        }

        //#else
        

        if (!sending) return;

        static uint8_t current_level = 0;
        static uint8_t send_flag = 0;
        uint8_t bit;

        if ((tx_bit_index >= START_SIG_LEN) && !send_flag) {
            send_flag = 1;
            current_level = 0;
            tx_bit_index = 0;
        }

        if (!send_flag) {
            current_level = 1;

            // make the last bit 0 ----> for START_SIG = 0b1110
            if (START_SIG == 0b1110){
                if (tx_bit_index >= START_SIG_LEN-1) current_level = 0;
            }

        } else {
            bit = (tx_buffer >> ((MSG_LENGTH - 1) - ((tx_bit_index / 2) % MSG_LENGTH))) & 1;
            if (!(tx_bit_index & 1)) {
                if (!bit) current_level = !current_level;
            } else {
                current_level = !current_level;
            }
        }

        tx_bit_index++;
        if (tx_bit_index >= (MSG_LENGTH * 2)) {
            tx_bit_index = 0;
            send_flag = 0;
            current_level = 0;
            sending = 0;
        }
        multiple_led_drive(led_pins, led_size, current_level);
        //#endif
    #else

        if(!reading) return;
        
        multiple_led_drive(led_pins, led_size, 0);
        
        if(reading){    
            adc_lib_read_all_logical(adc1_results, adc2_results, rx_buffer, SIG_THRESHOLD);
            for (int i = 0; i < CHANNEL_NUM; i++) rx_count[i]++;
        }


        //  This part is buggy

        // // If backoff is active, skip checking and transmitting
        // if (backoff_active) {
        //     if(backoff_countdown-- == 0) backoff_active = 0;  // Counting down, when reaching 0, check again
        //     return;
        // }
        
        // channel_occupied = 0;
        // for (int i = 0; i < adc1_size; i++) {
        //     if (adc1_results[i] > SIG_THRESHOLD) {
        //         channel_occupied = 1;
        //         break;
        //     }
        // }
        // for (int i = 0; i < adc2_size; i++) {
        //     if (adc2_results[i] > SIG_THRESHOLD) {
        //         channel_occupied = 1;
        //         break;
        //     }
        // }

        // // If channel is busy, start backoff timer
        // if (channel_occupied) {
        //     multiple_led_drive(led_pins, led_size, 0);
        //     backoff_active = 1;
        //     sending = 0;
        //     backoff_countdown = (rand() % (MAX_BACKOFF_CYCLE * CYCLE_BIT_COUNT)) + (MIN_BACKOFF_CYCLE * CYCLE_BIT_COUNT); // Random backoff
        //     return;  // Skip transmission
        // }

        

        if (!sending) return;

        if (sending && channel_occupied) {
            multiple_led_drive(led_pins, led_size, 0);
            sending = 0;        // Abort sending if the channel gets occupied
            backoff_active = 1;
            return;
        } 

        static uint8_t current_level = 0;
        static uint8_t send_flag = 0;
        uint8_t bit;

        if ((tx_bit_index >= START_SIG_LEN) && !send_flag) {
            send_flag = 1;
            current_level = 0;
            tx_bit_index = 0;
        }

        if (!send_flag) {
            current_level = 1;

            // make the last bit 0 ----> for START_SIG = 0b1110
            if (START_SIG == 0b1110){
                if (tx_bit_index >= START_SIG_LEN-1) current_level = 0;
            }

        } else {
            bit = (tx_buffer >> ((MSG_LENGTH - 1) - ((tx_bit_index / 2) % MSG_LENGTH))) & 1;
            if (!(tx_bit_index & 1)) {
                if (!bit) current_level = !current_level;
            } else {
                current_level = !current_level;
            }
        }

        tx_bit_index++;
        if (tx_bit_index >= (MSG_LENGTH * 2)) {
            tx_bit_index = 0;
            send_flag = 0;
            current_level = 0;
            sending = 0;
        }
        multiple_led_drive(led_pins, led_size, current_level);
    #endif
}


void dm_comm_init(adc1_channel_t *adc1_ch, int a1_size, adc2_channel_t *adc2_ch, int a2_size, gpio_num_t *leds, int l_size) {
    adc1_channels = adc1_ch;
    adc2_channels = adc2_ch;
    adc1_size = a1_size;
    adc2_size = a2_size;
    led_pins = leds;
    led_size = l_size;
    
    adc1_config_t adc1_config = {
        .adc1_channels = adc1_channels,
        .adc1_num_channels = adc1_size,
        .width = ADC_WIDTH_BIT_12,
        .atten = ADC_ATTEN_DB_0
    };
    adc2_config_t adc2_config = {
        .adc2_channels = adc2_channels,
        .adc2_num_channels = adc2_size,
        .width = ADC_WIDTH_BIT_12,
        .atten = ADC_ATTEN_DB_0
    };
    adc_lib_init_all(&adc1_config, &adc2_config);
    multiple_led_init(led_pins, led_size);
    hwtimer_init(timer_comm, 1000000, BIT_DURATION_US, timer1_callback);

}

void dm_comm_start() {
    hwtimer_start(timer_comm);
}

void dm_comm_stop() {
    hwtimer_stop(timer_comm);
}

void dm_comm_send(int message) {
    if (channel_occupied || backoff_active) return;

    tx_buffer = message;
    sending = 1;
}

void dm_comm_reading_stop(void){
    reading = 0;
    // rx_count = 0;
    read_flag = 0;
    for (int i = 0; i < CHANNEL_NUM; i++)
    {
        rx_buffer[i] = 0;
    }
}

void dm_comm_reading_start(void){
    reading = 1;
}

bool dm_comm_detect_start_sig() {
    bool any_start = false;

    for (int i = 0; i < CHANNEL_NUM; i++) {
        if (!start_detected[i] && rx_buffer[i] == START_SIG) {
            start_detected[i] = 1;
            rx_count[i] = 0;
            rx_buffer[i] = 0;
            any_start = true;
        } else if (!start_detected[i] &&
                   ((i < adc1_size && adc1_results[i] <= SIG_THRESHOLD) ||
                    (i >= adc1_size && adc2_results[i - adc1_size] <= SIG_THRESHOLD))) {
            rx_buffer[i] = 0;
        }
    }

    return any_start;
}


void decode_channel(int i) {
    if (rx_count[i] != 2 * MSG_LENGTH) return;

    uint8_t rx_bit, rx_bit_prev;
    msg[i] = 0;

    for (int j = 0; j < 2 * MSG_LENGTH; j += 2) {
        rx_bit = (rx_buffer[i] >> ((2 * MSG_LENGTH - 1) - j)) & 1;

        if (j == 0) {
            msg[i] = rx_bit ? 0 : 1;
        } else {
            rx_bit_prev = (rx_buffer[i] >> ((2 * MSG_LENGTH - 1) - (j - 1))) & 1;
            msg[i] = (msg[i] << 1) | (rx_bit == rx_bit_prev);
        }
    }

    start_detected[i] = 0;
    rx_count[i] = 0;
    rx_buffer[i] = 0;
}

bool dm_comm_process() {
    bool any_processed = false;
    
    dm_comm_detect_start_sig();

    for (int i = 0; i < CHANNEL_NUM; i++) {
        if (start_detected[i] && rx_count[i] == 2 * MSG_LENGTH) {
            decode_channel(i);
            any_processed = true;
        }
    }

    return any_processed;
}


bool dm_comm_detect_signals(void) {
    //printf("\ndm_comm reading");
    adc_lib_read_all(sig_adc1_results, sig_adc2_results);
    vTaskDelay(pdMS_TO_TICKS(10)); // there has to be delay at 10 ms, otherwise crashes
    for (int i = 0; i < adc1_size; i++)
    {
        if (sig_adc1_results[i] >= SIG_THRESHOLD) return true;
    }

    for (int i = 0; i < adc2_size; i++) {
        if (sig_adc2_results[i] >= SIG_THRESHOLD) return true;
    }
    return false;
}

void dm_comm_get_signals(int adc_results[CHANNEL_NUM]){
    adc_lib_read_all(sig_adc1_results, sig_adc2_results);
    for (int i = 0; i < adc1_size; i++)
    {
        adc_results[i] = sig_adc1_results[i];
    }

    for (int i = 0; i < adc2_size; i++) {
        adc_results[i + adc1_size] = sig_adc2_results[i];
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // there has to be delay at 10 ms, otherwise crashes
}

void dm_comm_get_messages(int rx_msg[CHANNEL_NUM]) {
    for (int i = 0; i < CHANNEL_NUM; i++)
    {
        rx_msg[i] = msg[i];
        msg[i] = 0;
    }
}

void dm_comm_get_msg_strength(int adc_results[CHANNEL_NUM]){
    for (int i = 0; i < adc1_size; i++)
    {
        adc_results[i] = adc1_results[i];
    }

    for (int i = 0; i < adc2_size; i++) {
        adc_results[i + adc1_size] = adc2_results[i];
    }
}

bool dm_comm_backoff(){
    return backoff_active;
}

void dm_comm_set_backoff(int backoff){
    backoff_active = 1; 
    backoff_countdown = backoff * CYCLE_BIT_COUNT;
}