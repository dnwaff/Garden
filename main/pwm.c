#include "driver/ledc.h"

typedef struct {
ledc_timer_config_t timer;
ledc_channel_config_t channel;
}Config;

Config config;



void init_pwm(uint8_t pin, int freq){

    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = freq,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    config.timer = ledc_timer;

	ledc_channel_config_t led_config =
		{
			.channel    = LEDC_CHANNEL_0,
			.duty       = 0,
			.gpio_num   = pin,
			.speed_mode = LEDC_HIGH_SPEED_MODE,
			.hpoint     = 0,
			.timer_sel  = LEDC_TIMER_0
		};
	ledc_channel_config(&led_config);
	config.channel = led_config;
};

void pwm_set_duty(int duty){
    ledc_set_duty(config.channel.speed_mode, config.channel.channel, duty);
}

void pwm_start(){
	ledc_update_duty(config.channel.speed_mode, config.channel.channel);
}

void pwm_stop(){
	ledc_stop(config.channel.speed_mode, config.channel.channel, 0);
}

