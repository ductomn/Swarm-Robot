#include "coop.h"


// LED and ADC setup
static adc1_channel_t *adc_dis_channels;
static int adc_dis_size;
static gpio_num_t *led_dis_pins;
static int led_size;

int dis_results[DIS_CHANNEL_NUM];

static int adc_results[CHANNEL_NUM];
int coop_direction;

static bool obstacle_detected;

static void timer2_callback() {

    multiple_led_drive(led_dis_pins, led_size, 1);
    adc_lib_dis_read_all(dis_results);  // dummy read
    adc_lib_dis_read_all(dis_results);  // dummy read
    adc_lib_dis_read_all(dis_results);
    // vTaskDelay(pdMS_TO_TICKS(1));
    multiple_led_drive(led_dis_pins, led_size, 0);

    if ((dis_results[0] >= DIS_THRESHOLD) &&(dis_results[1] >= DIS_THRESHOLD))  obstacle_detected = 1;
    else obstacle_detected = 0;    

}

void coop_dis_init(adc1_channel_t *adc1_ch, int a1_size, gpio_num_t *leds, int l_size) {
    adc_dis_channels = adc1_ch;
    adc_dis_size = a1_size;
    led_dis_pins = leds;
    led_size = l_size;
    
    adc1_config_t adc_dis_config = {
        .adc1_channels = adc_dis_channels,
        .adc1_num_channels = adc_dis_size,
        .width = ADC_WIDTH_BIT_12,
        .atten = ADC_ATTEN_DB_0
    };

    adc_lib_dis_init(&adc_dis_config);

    multiple_led_init(led_dis_pins, led_size);
    hwtimer_init(TIMER_DIS, TIMER_DIS_RESOLUTION, DIS_PERIOD, timer2_callback);

}

bool coop_obstacle_detection(void){
    return obstacle_detected;
}


// Function to determine the direction of the strongest signal
int coop_signal_direction(int adc_values[CHANNEL_NUM]) {
    int max_index = 0;
    uint16_t max_value = adc_values[0];
    
    for (int i = 0; i < CHANNEL_NUM; i++) {
        //printf("\n%d: \t %d", i,adc_values[i]);
        if (adc_values[i] > max_value) {
            max_value = adc_values[i];
            max_index = i;
        }
    }
    //printf("\n\n");
    return max_index;
}

void coop_turn_to_signal(int direction){

    if (direction == 0) return;

    if (direction <= 3) servo_rotate_right(500);
    if (direction > 3) servo_rotate_left(500);
    vTaskDelay(pdMS_TO_TICKS(200));
    servo_stop();
}

void coop_turn_away(int direction){
    if (direction == 3) return;

    if (direction > 3) servo_rotate_right(500);
    if (direction < 3) servo_rotate_left(500);
    vTaskDelay(pdMS_TO_TICKS(200));
    servo_stop();
}



//----------    RANDOM WALK     ----------

static move_type_t previous_move = MOVE_FORWARD;
static move_type_t current_move = MOVE_FORWARD;
static uint32_t random_move_duration = 0;
static uint32_t random_move_start_time = 0;

void random_walk_start(){
    previous_move = MOVE_FORWARD;
    current_move = MOVE_FORWARD;
    random_move_duration = 0;
    random_move_start_time = 0;
    // Delay is needed before random_walk starts (or mby timers resets), or crashes
    vTaskDelay(pdMS_TO_TICKS(10));  
}

void apply_move(move_type_t move) {
    switch (move) {
        case MOVE_FORWARD:
            servo_move_forward(SERVO_MOVE_SPEED);
            break;
        case TURN_LEFT:
            servo_rotate_left(SERVO_MOVE_SPEED);
            break;
        case TURN_RIGHT:
            servo_rotate_right(SERVO_MOVE_SPEED);
            break;
    }
}

// Pick a new random move and duration
void random_walk_choose(uint32_t time_now, int state) {
    if(previous_move >= TURN_LEFT) current_move = MOVE_FORWARD;
    else current_move = esp_random() % 3;

    previous_move = current_move;
    if (current_move >= TURN_LEFT) random_move_duration = MIN_MOVE_TIME_US + (esp_random() % (MAX_ROTATE_TIME_US - MIN_MOVE_TIME_US));
    else if (state >= 5) random_move_duration = LEADER_MIN_MOVE_TIME_US + (esp_random() % (LEADER_MAX_MOVE_TIME_US - LEADER_MIN_MOVE_TIME_US));
    else random_move_duration = MIN_MOVE_TIME_US + (esp_random() % (MAX_MOVE_TIME_US - MIN_MOVE_TIME_US));
    random_move_start_time = time_now;
    apply_move(current_move);
}

void random_walk_loop(uint32_t time_now, int state){
    if (time_now - random_move_start_time >= random_move_duration) {
        random_walk_choose(time_now, state);
    }
}

void signal_correction(){
    // if(previous_move == MOVE_FORWARD) servo_move_backwards(SERVO_MOVE_SPEED);
    // else if(previous_move == TURN_LEFT) servo_rotate_right(SERVO_MOVE_SPEED);
    // else if(previous_move == TURN_RIGHT) servo_rotate_left(SERVO_MOVE_SPEED);
    // vTaskDelay(pdMS_TO_TICKS(100));

    int signal_found = 0;
    // servo_rotate_right(50);

    // Kick the robot out of signal range to slowly get back
    servo_rotate_left(500);
    vTaskDelay(pdMS_TO_TICKS(200));  
    
    servo_stop();
    vTaskDelay(pdMS_TO_TICKS(20));
    
    while (!signal_found){
        servo_rotate_right(60);
        
        dm_comm_get_signals(adc_results);

        printf("\nLooking for signal");

        for (int i = 0; i < CHANNEL_NUM; i++){
            if (adc_results[i] >= SIG_THRESHOLD) signal_found = 1;
        }

    }

    servo_stop();
}


// void leader_walk(uint32_t time_now) {
//     if (time_now - random_move_start_time >= random_move_duration) {
//         if(previous_move >= TURN_LEFT) current_move = MOVE_FORWARD;
//         else current_move = esp_random() % 3;

//         previous_move = current_move;
//         if (current_move >= TURN_LEFT) random_move_duration = MIN_MOVE_TIME_US + (esp_random() % (MAX_ROTATE_TIME_US - MIN_MOVE_TIME_US));
//         else random_move_duration = MIN_MOVE_TIME_US + (esp_random() % (MAX_MOVE_TIME_US - MIN_MOVE_TIME_US));
//         random_move_start_time = time_now;
//         apply_move(current_move);
//     }
// }



// ----------   CMD2 - SPREAD OUT   ------------

static spread_out_state_t current_state = STATE_TURN_AWAY;
static uint32_t state_start_time = 0;


void coop_start_spread(){
    current_state = STATE_TURN_AWAY;
    state_start_time = 0;
}

void coop_spread_out_loop(uint32_t time_now, bool flag_close) {
    switch (current_state) {
        case STATE_TURN_AWAY:
            dm_comm_get_signals(adc_results);
            coop_direction = coop_signal_direction(adc_results);
            //printf("\n%lld \tCOOP Direction: %d\n",time_now, direction);
            coop_turn_away(coop_direction);
            if (time_now - state_start_time >= TURN_AWAY_TIME_US) {
                current_state = STATE_FORWARD_1;
                state_start_time = time_now;
            }
            break;
        case STATE_FORWARD_1:
            servo_move_forward(SERVO_MOVE_SPEED);
            if (time_now - state_start_time >= FORWARD_TIME_US) {
                current_state = STATE_REVERSE;
                state_start_time = time_now;
            }
            break;

        case STATE_REVERSE:
            servo_rotate_right_91();
            servo_rotate_right_91();
            current_state = STATE_FORWARD_2;
            state_start_time = time_now;
            
            // servo_rotate_right(SERVO_ROTATE_RIGHT_SPEED);
            // if (time_now - state_start_time >= REVERSE_TIME_US) {
            //     current_state = STATE_FORWARD_2;
            //     state_start_time = time_now;
            // }
            break;

        case STATE_FORWARD_2:
            servo_move_forward(SERVO_MOVE_SPEED);
            if (time_now - state_start_time >= FORWARD_TIME_US) {
                current_state = STATE_DONE;
                servo_stop();
            }
            break;

        case STATE_DONE:
            // Look for signal
            dm_comm_get_signals(adc_results);
            coop_direction = coop_signal_direction(adc_results);
            //printf("\n%lld \tCOOP: Direction: %d\n",time_now, coop_direction);
            coop_turn_to_signal(coop_direction);

            if (flag_close) servo_stop();
            else servo_move_forward(SERVO_MOVE_SPEED);

            break;
    }
}

uint8_t coop_get_state(){
    return current_state;
}



// ----------   CHAIN FORMATION   ------------


void state_chain() {
    static uint8_t role_id = ID_UNKNOWN;
    static int rx_msg[CHANNEL_NUM];    // decoded message
    static int signal_front = 0, signal_back = 0;
    static bool cooldown_active = true;
    static int msg_count = 0;

    static uint32_t time_now = 0;
    static uint32_t timer_command = 0;


    time_now = hwtimer_get_time();
    timer_command = hwtimer_cmd_get_time();

    if ((role_id == ID_BACK) && (cooldown_active) && (time_now >= COOLDOWN_AFTER_MOVE)) cooldown_active = false;
    

    // Detect signal and direction
    if (dm_comm_process()) {
        dm_comm_get_messages(rx_msg);

        if (rx_msg[0] == MSG_PRESENCE_BEACON) signal_front++;
        if (rx_msg[3] == MSG_PRESENCE_BEACON) signal_back++;
        msg_count++;
        //printf("\nfront: %d    back: %d", signal_front, signal_back);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Update role based on presence
    if(msg_count >= SIGNAL_SAMPLE_COUNT){
        printf("\nfront: %d    back: %d", signal_front, signal_back);
        printf("\nMy ID (role): %d", role_id);
        if (signal_front >= 10 && signal_back >= 10) {
            if (role_id != ID_MIDDLE) {
                role_id = ID_MIDDLE;
                printf("\n→ Role: MIDDLE");
            }
        } else if (signal_front >= 10 && signal_back < 5) {
            if (role_id != ID_BACK) {
                role_id = ID_BACK;
                cooldown_active = true;
                hwtimer_reset_clock();
                time_now = 0;
                printf("\n→ Role: BACK");
            }
        } else if (signal_front < 5 && signal_back >= 10) {
            if (role_id != ID_FRONT) {
                role_id = ID_FRONT;
                printf("\n→ Role: FRONT");
            }
        } else {
            if (role_id != ID_UNKNOWN) {
                role_id = ID_UNKNOWN;
                printf("\n→ Role: UNKNOWN");
            }
        }

        signal_back = 0;
        signal_front = 0;
        msg_count = 0;

    }

    // Broadcast role
    if (timer_command >= MSG_INTERVAL) {
        dm_comm_send(MSG_PRESENCE_BEACON);
        hwtimer_cmd_reset_clock();
        timer_command = 0;
    }

    // Move if in the back
    if ((role_id == ID_BACK) && !cooldown_active) {

        printf("\nMoving up");

        // Get out of line
        servo_rotate_right_91();
        servo_move_forward(300);
        vTaskDelay(pdMS_TO_TICKS(CHAIN_FORWARD_ROTATE_MS));
        servo_rotate_left_91();

        // Moving in parallel of line
        servo_move_forward(300);
        vTaskDelay(pdMS_TO_TICKS(CHAIN_FORWARD_TIME_MS));
        
        // %%%%%%% Blindly align in front of line %%%%%%%%%

        // // Get in line
        // servo_rotate_left_91();
        // servo_move_forward(300);
        // vTaskDelay(pdMS_TO_TICKS(CHAIN_FORWARD_ROTATE_MS));
        // servo_rotate_right_91();

        // servo_stop();

        // role_id = ID_FRONT;
        // hwtimer_cmd_reset_clock();
        // timer_command = 0;
        // printf("\nMoved: BACK → FRONT");




        // %%%%%%% Dynamically align in front of line, but NOT WORKING %%%%%%%%%

        // Check if at the front of line
        int back_left_max = 0, front_left_max = 0;
        bool far_enough = 0;

        servo_move_forward(300);
        while (!far_enough){
            back_left_max = 0;
            front_left_max = 0;
            for (int i = 0; i <= SIGNAL_SAMPLE_COUNT; i++) {
                dm_comm_get_signals(adc_results);

                // Track max values over time
                if (adc_results[4] > back_left_max) back_left_max = adc_results[4];     // BACK_LEFT
                if (adc_results[5] > front_left_max) front_left_max = adc_results[5];   // FRONT_LEFT
            }

            if (back_left_max >= (front_left_max + 100)) far_enough = 1;
        }

        // not working properly

        // // Rotate to go back in line
        // bool rotate_back_enough = 0;

        // servo_rotate_left(100);
        // while (!rotate_back_enough){
        //     back_left_max = 0;
        //     front_left_max = 0;
        //     for (int i = 0; i <= SIGNAL_SAMPLE_COUNT; i++) {
        //         dm_comm_get_signals(adc_results);

        //         // Track max values over time
        //         if (adc_results[4] > back_left_max) back_left_max = adc_results[4];     // BACK_LEFT
        //         if (adc_results[5] > front_left_max) front_left_max = adc_results[5];   // FRONT_LEFT
        //     }           
            
        //     if (front_left_max >= (back_left_max + 10)) rotate_back_enough = 1;
        // }

        vTaskDelay(pdMS_TO_TICKS(500));
        servo_rotate_left_91();

        // Go in front of front robot
        far_enough = 0;
        
        servo_move_forward(300);
        hwtimer_reset_clock();
        time_now = 0;
        // vTaskDelay(pdMS_TO_TICKS(CHAIN_FORWARD_ROTATE_MS));
        while (!far_enough){
            time_now = hwtimer_get_time();

            back_left_max = 0;
            front_left_max = 0;
            for (int i = 0; i <= 20; i++) {
                dm_comm_get_signals(adc_results);

                // Track max values over time
                if (adc_results[4] > back_left_max) back_left_max = adc_results[4];     // BACK_LEFT
                if (adc_results[5] > front_left_max) front_left_max = adc_results[5];   // FRONT_LEFT
            }

            if ((abs(back_left_max - front_left_max) <= 50) && (time_now >= CHAIN_FORWARD_ROTATE_MS-500)) far_enough = 1;
            if (time_now >= CHAIN_FORWARD_ROTATE_MS+500) far_enough = 1;
        }

        // Align
        back_left_max = 0;
        int back_max = 0;
        bool rotate_enough = 0;
        hwtimer_reset_clock();
        time_now = 0;

        servo_rotate_right(100);
        while (!rotate_enough){
            time_now = hwtimer_get_time();

            back_left_max = 0;
            back_max = 0;
            for (int i = 0; i <= SIGNAL_SAMPLE_COUNT-30; i++) {
                dm_comm_get_signals(adc_results);

                // Track max values over time
                if (adc_results[4] > back_left_max) back_left_max = adc_results[4];     // BACK_LEFT
                if (adc_results[3] > back_max) back_max = adc_results[3];   // BACK
            }           
            
            if ((back_max >= (back_left_max + 1000)) && (time_now >= SERVO_ROTATE_RIGHT-100)) rotate_enough = 1;
            if (time_now >= SERVO_ROTATE_RIGHT+100) rotate_enough = 1;
        }

        servo_stop();

        role_id = ID_FRONT;
        hwtimer_cmd_reset_clock();
        timer_command = 0;
        printf("\nMoved: BACK → FRONT");
    }

    servo_stop();
}