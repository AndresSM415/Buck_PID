#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

//PID
#define KP 50
#define KI 80
#define KD 0.0
#define SET_POINT 5.1

//pines
#define LOAD_PIN 27
#define SHUNT_PIN 26
#define PWM_PIN 28
#define FREQUENCY 100E3
#define CLOCK_FREQUENCY 125E6


//funciones
double PID(void);
void set_pwm_duty_cycle(double, uint, uint);
clock_t millis();
double map(double, double, double, double, double);

//variables globales
uint slice, channel;
uint16_t WRAP = CLOCK_FREQUENCY/FREQUENCY - 1;

int main()
{
    stdio_init_all();
    
    //ADC
    adc_init();
    adc_gpio_init(LOAD_PIN);
    adc_select_input(1);

    //outputs
    gpio_init(PWM_PIN);
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(PWM_PIN);
    channel = pwm_gpio_to_channel(PWM_PIN);
    pwm_set_enabled(slice, true);//ACTIVA EL PWM DEL PIN
    pwm_set_wrap(slice, WRAP);//SELECCIONA LA FRECUENCIA DEL PWM

    while(true)
    {
        PID();
    }
}

clock_t LAST_TIME;
double INTEGRAL, LAST_ERROR;
double PID(void)
{
    clock_t TIME = millis();
    double DT = (double)(TIME-LAST_TIME)/1000.0;
    LAST_TIME = TIME;

    double VOLTAGE_READ = map(adc_read(), 0, 4096, 0, 3.3*12.61/2.61);
    double ERROR = SET_POINT - VOLTAGE_READ;

    double PROPORTIONAL = ERROR;
    INTEGRAL += ERROR*DT;
    double DERIVATIVE = (ERROR - LAST_ERROR)/DT;
    LAST_ERROR = ERROR;

    double OUTPUT = (KP*PROPORTIONAL + KI*INTEGRAL);
    //double OUTPUT = (0*PROPORTIONAL + 0*INTEGRAL + 0*DERIVATIVE);

    if(OUTPUT > 70.0)
        OUTPUT = 70;
    else if(OUTPUT < 5.0)
        OUTPUT = 5;

    set_pwm_duty_cycle(OUTPUT, slice, channel);
}

void set_pwm_duty_cycle(double DUTY_CYCLE, uint slice, uint channel)
{
    u_int32_t LEVEL = map(DUTY_CYCLE, 0, 100, 0, WRAP);
    pwm_set_chan_level(slice, channel, LEVEL);//configura el DC
}

double map(double x, double INPUT_MINIMUM, double INPUT_MAXIMUM, double OUTPUT_MINIMUM, double OUTPUT_MAXIMUM)
{
    return (x - INPUT_MINIMUM)*(OUTPUT_MAXIMUM - OUTPUT_MINIMUM)/(INPUT_MAXIMUM -INPUT_MINIMUM) + OUTPUT_MINIMUM;
}

clock_t millis()
{
    return (clock_t) time_us_64() / 1000;
}