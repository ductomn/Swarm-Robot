#include "hwtimer.h"

static const char* TAG = "hwtimer_lib";
static gptimer_handle_t gptimers[MAX_TIMERS] = { NULL };
static timer_callback_t user_callbacks[MAX_TIMERS] = { NULL };

static uint32_t clock_period = 0;
static uint32_t clock_command = 0;


static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) 
{
    int timer_id = (int)user_ctx;  // Retrieve timer ID from input
    if ((timer_id >= 0) && (timer_id < MAX_TIMERS) && (user_callbacks[timer_id])) {
        user_callbacks[timer_id]();  // Call user-defined function
    }
    return true;
}

esp_err_t hwtimer_init(int timer_id, uint32_t resolution, uint64_t timer_count, timer_callback_t callback)
{

    if (timer_id < 0 || timer_id >= MAX_TIMERS) {
        ESP_LOGE(TAG, "Invalid timer ID: %d", timer_id);
        return ESP_ERR_INVALID_ARG;
    }
    if (gptimers[timer_id] != NULL) {
        ESP_LOGW(TAG, "Timer %d already initialized", timer_id);
        return ESP_ERR_INVALID_ARG;
    }

    user_callbacks[timer_id] = callback;  // Store user callback
    
    //   Configure GPTimer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = resolution  // eg. 1 MHz = 1 / 1Mhz = 1 tick per µs
    };
    if (gptimer_new_timer(&timer_config, &gptimers[timer_id]) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer %d", timer_id);
        return true;
    }
    
    //   Configure timer alarm
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = timer_count,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true
    };
    if (gptimer_set_alarm_action(gptimers[timer_id], &alarm_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set alarm for timer %d", timer_id);
        return true;
    }

    // Attach ISR callback
    gptimer_event_callbacks_t cbs = { .on_alarm = timer_callback };
    if (gptimer_register_event_callbacks(gptimers[timer_id], &cbs, (void*)timer_id) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callbacks for timer %d", timer_id);
        return true;
    }

    // Calculate interval in microseconds
    long long int time_us = (long long int)timer_count * 1000000 / resolution;

    ESP_LOGI(TAG, "Timer initialized with %lld us interval", time_us);

    ESP_ERROR_CHECK(gptimer_enable(gptimers[timer_id]));
    ESP_ERROR_CHECK(gptimer_start(gptimers[timer_id]));
    ESP_LOGI(TAG, "Timer %d started", timer_id);

    return ESP_OK;
}

void hwtimer_start(int timer_id) 
{
    if ((timer_id < 0) || (timer_id >= MAX_TIMERS) || (gptimers[timer_id] == NULL)) 
    {
        ESP_LOGE(TAG, "Invalid timer %d", timer_id);
        return;
    }

    ESP_ERROR_CHECK(gptimer_start(gptimers[timer_id]));
    ESP_LOGI(TAG, "Timer %d started", timer_id);
}

void hwtimer_stop(int timer_id) 
{
    if ((timer_id < 0) || (timer_id >= MAX_TIMERS) || (gptimers[timer_id] == NULL)) 
    {
        ESP_LOGE(TAG, "Invalid timer %d", timer_id);
        return;
    }
    ESP_ERROR_CHECK(gptimer_stop(gptimers[timer_id]));
    ESP_LOGI(TAG, "Timer %d stopped", timer_id);
}

void hwtimer_deinit(int timer_id) 
{
    if (((timer_id < 0) || (timer_id >= MAX_TIMERS) || (gptimers[timer_id] == NULL))) return;
    
    gptimer_disable(gptimers[timer_id]);
    gptimer_del_timer(gptimers[timer_id]);
    gptimers[timer_id] = NULL;
    user_callbacks[timer_id] = NULL;
    ESP_LOGI(TAG, "Timer %d deinitialized", timer_id);
}


esp_err_t hwtimer_once_init(int timer_id, uint32_t resolution, uint64_t timer_count, timer_callback_t callback)
{

    if (timer_id < 0 || timer_id >= MAX_TIMERS) {
        ESP_LOGE(TAG, "Invalid timer ID: %d", timer_id);
        return ESP_ERR_INVALID_ARG;
    }
    if (gptimers[timer_id] != NULL) {
        ESP_LOGW(TAG, "Timer %d already initialized", timer_id);
        return ESP_ERR_INVALID_ARG;
    }

    user_callbacks[timer_id] = callback;  // Store user callback
    
    //   Configure GPTimer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = resolution  // eg. 1 MHz = 1 / 1Mhz = 1 tick per µs
    };
    if (gptimer_new_timer(&timer_config, &gptimers[timer_id]) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer %d", timer_id);
        return true;
    }
    
    //   Configure timer alarm
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = timer_count,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = false
    };
    if (gptimer_set_alarm_action(gptimers[timer_id], &alarm_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set alarm for timer %d", timer_id);
        return true;
    }

    // Attach ISR callback
    gptimer_event_callbacks_t cbs = { .on_alarm = timer_callback };
    if (gptimer_register_event_callbacks(gptimers[timer_id], &cbs, (void*)timer_id) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register callbacks for timer %d", timer_id);
        return true;
    }

    // Calculate interval in microseconds
    long long int time_us = (long long int)timer_count * 1000000 / resolution;

    ESP_LOGI(TAG, "Timer initialized, called after %lld us", time_us);

    ESP_ERROR_CHECK(gptimer_enable(gptimers[timer_id]));
    ESP_ERROR_CHECK(gptimer_start(gptimers[timer_id]));
    ESP_LOGI(TAG, "Timer %d started", timer_id);

    return ESP_OK;
}

void hwtimer_once_start(int timer_id) 
{
    if ((timer_id < 0) || (timer_id >= MAX_TIMERS) || (gptimers[timer_id] == NULL)) 
    {
        ESP_LOGE(TAG, "Invalid timer %d", timer_id);
        return;
    }


    ESP_ERROR_CHECK(gptimer_start(gptimers[timer_id]));
    ESP_LOGI(TAG, "Timer %d started", timer_id);
}


static void timer0_callback()
{
    clock_period++;
    clock_command++;
}


esp_err_t hwtimer_clock_init(uint32_t resolution, uint64_t timer_count)
{
    return hwtimer_init(CLOCK_TIMER, resolution, timer_count, timer0_callback);
}

uint32_t hwtimer_get_time()
{
    return clock_period;
}

void hwtimer_reset_clock()
{
    clock_period = 0;
}

uint32_t hwtimer_cmd_get_time()
{
    return clock_command;
}

void hwtimer_cmd_reset_clock()
{
    clock_command = 0;
}