#include "servo_driver.h"


void servo_init(ledc_channel_t channel, int gpio) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = SERVO_MODE,
        .timer_num  = SERVO_TIMER,
        .duty_resolution = SERVO_RESOLUTION,
        .freq_hz    = SERVO_FREQ,
        .clk_cfg    = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num   = gpio,
        .speed_mode = SERVO_MODE,
        .channel    = channel,
        .timer_sel  = SERVO_TIMER,
        .duty       = SERVO_DUTY(SERVO_NEUTRAL_US), // Start at stop position
        .hpoint     = 0
    };
    ledc_channel_config(&channel_conf);
}

// Function to set servo speed (-1000 to 1000, where 0 is stop)
void servo_set_speed(ledc_channel_t channel, int speed) {
    int pulse_width = SERVO_NEUTRAL_US + (speed * (SERVO_MAX_US - SERVO_NEUTRAL_US) / 1000);
    if (pulse_width < SERVO_MIN_US) pulse_width = SERVO_MIN_US;
    if (pulse_width > SERVO_MAX_US) pulse_width = SERVO_MAX_US;
    ledc_set_duty(SERVO_MODE, channel, SERVO_DUTY(pulse_width));
    ledc_update_duty(SERVO_MODE, channel);
}

void servo_move_forward(int speed){
    servo_set_speed(SERVO_LEFT_CHANNEL, speed + SERVO_FORWARD_LEFT_MOD); 
    //servo_set_speed(SERVO_RIGHT_CHANNEL, -(speed/abs(speed) * (abs(speed)-10))); 
    servo_set_speed(SERVO_RIGHT_CHANNEL, -speed);
}

void servo_move_backwards(int speed){
    servo_set_speed(SERVO_LEFT_CHANNEL, -speed);  
    servo_set_speed(SERVO_RIGHT_CHANNEL, speed + SERVO_BACKWARDS_RIGHT_MOD); 
}

void servo_rotate_right(int speed){
    servo_set_speed(SERVO_LEFT_CHANNEL, speed);  
    servo_set_speed(SERVO_RIGHT_CHANNEL, speed);
}
void servo_rotate_left(int speed){
    servo_set_speed(SERVO_LEFT_CHANNEL, -speed);  
    servo_set_speed(SERVO_RIGHT_CHANNEL, -speed); 
}

// Rotate right for a little more than 90°
void servo_rotate_right_91(void){
    servo_set_speed(SERVO_LEFT_CHANNEL, SERVO_ROTATE_RIGHT_SPEED);  
    servo_set_speed(SERVO_RIGHT_CHANNEL, SERVO_ROTATE_RIGHT_SPEED); 
    vTaskDelay(pdMS_TO_TICKS(SERVO_ROTATE_RIGHT));
    servo_stop();
}

// Rotate left for a little more than 90°
void servo_rotate_left_91(void){
    servo_set_speed(SERVO_LEFT_CHANNEL, -SERVO_ROTATE_LEFT_SPEED); 
    servo_set_speed(SERVO_RIGHT_CHANNEL, -SERVO_ROTATE_LEFT_SPEED);
    vTaskDelay(pdMS_TO_TICKS(SERVO_ROTATE_LEFT));
    servo_stop();
}



void servo_stop(void){
    servo_set_speed(SERVO_LEFT_CHANNEL, 0);  
    servo_set_speed(SERVO_RIGHT_CHANNEL, 0); 
}