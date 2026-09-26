#include "tb6612.h"

#include <stddef.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define MOTOR_PWMA_PIN 2
#define MOTOR_AIN1_PIN 4
#define MOTOR_AIN2_PIN 3
#define MOTOR_PWMB_PIN 12
#define MOTOR_BIN1_PIN 10
#define MOTOR_BIN2_PIN 11
#define MOTOR_STBY_PIN 5

#define MOTOR_PWM_TOP 999
#define MOTOR_PWM_START 250

void tb6612_init(void) {
    const uint pins[] = {
        MOTOR_AIN1_PIN,
        MOTOR_AIN2_PIN,
        MOTOR_BIN1_PIN,
        MOTOR_BIN2_PIN,
        MOTOR_STBY_PIN,
    };
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], false);
    }

    gpio_set_function(MOTOR_PWMA_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_PWMB_PIN, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, MOTOR_PWM_TOP);
    pwm_init(pwm_gpio_to_slice_num(MOTOR_PWMA_PIN), &config, true);
    pwm_init(pwm_gpio_to_slice_num(MOTOR_PWMB_PIN), &config, true);
    pwm_set_gpio_level(MOTOR_PWMA_PIN, 0);
    pwm_set_gpio_level(MOTOR_PWMB_PIN, 0);
}

void tb6612_stop(void) {
    pwm_set_gpio_level(MOTOR_PWMA_PIN, 0);
    pwm_set_gpio_level(MOTOR_PWMB_PIN, 0);
    gpio_put(MOTOR_STBY_PIN, false);
}

void tb6612_drive(bool forward) {
    tb6612_stop();
    gpio_put(MOTOR_AIN1_PIN, forward);
    gpio_put(MOTOR_AIN2_PIN, !forward);
    gpio_put(MOTOR_BIN1_PIN, forward);
    gpio_put(MOTOR_BIN2_PIN, !forward);
    gpio_put(MOTOR_STBY_PIN, true);
    pwm_set_gpio_level(MOTOR_PWMA_PIN, MOTOR_PWM_START);
    pwm_set_gpio_level(MOTOR_PWMB_PIN, MOTOR_PWM_START);
}
