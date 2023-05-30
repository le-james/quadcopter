#include "pico/stdlib.h"
#include "hardware/pwm.h"

/** \file motor_lib.hpp
 * Initializes a pin for PWM usage. Set the frequency and duty cycle of the GPIO pin.
 * f = 50 Hz for servo motors
 */
class PWM
{
    public:
        uint slice_num;
        uint pwm_chan;
        uint16_t wrap;

    public:
        /** \brief Divides the clock frequency 
         * \param pin Pico pin to output a PWM signal
         * \param pwm_chan PWM channel of the gpio pin - Channel A = 0, Channel B = 1
         * \param f Desired frequency - operational frequency of the device
         */
        PWM(uint pin, uint pwm_chan, uint32_t f)
        {
            // store pwm_chan as a member variable
            this->pwm_chan = pwm_chan;

            // set the mode of the pin - pin is pwm mode
            gpio_set_function(pin, GPIO_FUNC_PWM);

            // Find out which PWM slice is connected to the pin
            slice_num = pwm_gpio_to_slice_num(pin);

            // Set the PWM running - enable pwn
            pwm_set_enabled(slice_num, true);


            // system clock frequency of the pico
            uint32_t clock = 125000000;

            // rounds up - c++ always rounds down integers
            uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);

            // set divider16 to min value of 16 if the ratio is less than 1
            if(divider16 / 16 < 1) divider16 = 16;

            wrap = clock * 16 / divider16 / f - 1;

            // clock division
            pwm_set_clkdiv_int_frac(slice_num, divider16/16, divider16 & 0xF);

            // set the wrap counter
            pwm_set_wrap(slice_num, wrap);

            // initialize the duty to be zero
            pwm_set_chan_level(slice_num, pwm_chan, 0);
        }


        /** \brief Change the duty cycle of the motor pin 
         * \param duty PWM duty cycle percentage from 0% to 100% (0-100)
         */
        void change_duty(double duty)
        {
            pwm_set_chan_level(slice_num, pwm_chan, wrap * duty / 100.0);
            printf("level: %f \n", wrap * duty / 100.0);
        }
};
