#ifndef _PWM_H_
#define _PWM_H_

void init_pwm(uint8_t pin, int freq);
void pwm_start();
void pwm_stop();
void pwm_set_duty(int duty);



#endif
