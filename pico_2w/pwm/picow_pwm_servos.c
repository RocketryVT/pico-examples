/**
 * Servo sweep example for Pico W using PWM on GPIO 0 and 1.
 */

#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"


#define PIN_SERVO_0 0  // PWM0 A
#define PIN_SERVO_1 1  // PWM0 B

#define MIN_PULSE_US 500
#define MAX_PULSE_US 2500

#define SERVO_FREQ_HZ 50

static uint16_t angle_to_pulse_us(float angle) {
    if (angle < 0.f) {
        angle = 0.f;
    }
    if (angle > 180.f) {
        angle = 180.f;
    }
    const float span = (float)(MAX_PULSE_US - MIN_PULSE_US);
    return (uint16_t)(MIN_PULSE_US + (angle / 180.f) * span);
}

static void set_servo_pwm(uint pin, uint16_t wrap, float angle) {
    const uint32_t period_us = 1000000u / SERVO_FREQ_HZ;  // 20,000 us
    const uint32_t pulse_us = angle_to_pulse_us(angle);
    uint32_t level = (pulse_us * (wrap + 1)) / period_us;
    if (level > wrap) {
        level = wrap;
    }
    pwm_set_gpio_level(pin, (uint16_t)level);
}

int main() {
    stdio_init_all();

    // For 5 seconds print "Hello, world!" every second
    while (time_us_64() < 5000000) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }


    const float clk_div = 125.0f;   // 125 MHz / 125 = 1 MHz -> 1 count per microsecond
    const uint16_t wrap = (1000000u / SERVO_FREQ_HZ) - 1;  // 20,000 us period -> wrap 19,999

    gpio_set_function(PIN_SERVO_0, GPIO_FUNC_PWM);
    gpio_set_function(PIN_SERVO_1, GPIO_FUNC_PWM);

    const uint slice_num = pwm_gpio_to_slice_num(PIN_SERVO_0);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clk_div);
    pwm_config_set_wrap(&config, wrap);
    pwm_init(slice_num, &config, true);

    const float total_time_s = 10.0f;
    const int steps = 180;
    const uint32_t step_delay_ms = (uint32_t)((total_time_s * 1000.f) / steps);

    printf("Moving both servos from 0 to 180 degrees in %.1f seconds...\n", total_time_s);

    set_servo_pwm(PIN_SERVO_0, wrap, 0.f);
    set_servo_pwm(PIN_SERVO_1, wrap, 0.f);
    sleep_ms(50);

    for (int a = 0; a <= steps; a++) {
        const float angle = (float)a;
        set_servo_pwm(PIN_SERVO_0, wrap, angle);
        set_servo_pwm(PIN_SERVO_1, wrap, angle);
        sleep_ms(step_delay_ms);
    }

    // Stop PWM and release the pins to a low level.
    pwm_set_gpio_level(PIN_SERVO_0, 0);
    pwm_set_gpio_level(PIN_SERVO_1, 0);
    pwm_set_enabled(slice_num, false);

    printf("Done.\n");
    return 0;
}
