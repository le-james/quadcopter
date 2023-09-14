#include <stdio.h>

// testing input from keyboard to pico via usb - not working for some reason
#include <stdio.h>
#include "pico/stdlib.h"

int main(){
    //Initialise I/O
    stdio_init_all(); 
    // stdio_usb_init();

    // Initialise GPIO (Green LED connected to pin 25)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    char userInput;

    // need to include "pico_enable_stdio_usb" function in cmakelist.txt
    while(!stdio_usb_connected()) {
        sleep_ms(500);
    }
    printf("usb connected to pc\n");

    //Main Loop 
    while(1){
        //Get User Input
        printf("Command (1 = on or 0 = off):\n");
        // userInput = getchar();
        userInput = getchar_timeout_us(10000000);

        printf("%c\n",userInput);


        if(userInput == '1'){
            // Turn On LED
            gpio_put(25, 1); // Set pin 25 to high
            printf("LED switched on!\n");
        }
        else if(userInput == '0'){
            // Turn Off LED
            gpio_put(25, 0); // Set pin 25 to high.
            printf("LED switched off!\n");
        }
        else{
            printf("Invalid Input!\n");
        }
    }
}