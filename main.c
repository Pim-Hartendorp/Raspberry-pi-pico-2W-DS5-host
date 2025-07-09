#include <stdio.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "DS5_hid_host.h"

int main()
{

    const char * remote_addr_string = "58:10:31:2B:C8:71";

    stdio_init_all();
    if (cyw43_arch_init()) {
        puts("cyw43 init error");
        return 1;

    }
    
    DS5_init(remote_addr_string, false);


    while(1) {
        
        //example implementation of using the DS5 controller code.
        volatile int8_t left_x = DS5_left_joystick_X_axis(0);
        volatile int8_t left_y = DS5_left_joystick_Y_axis(0);
        volatile int8_t right_x = DS5_right_joystick_X_axis(0);
        volatile int8_t right_y = DS5_right_joystick_Y_axis(0);

        volatile int8_t l2_axis = DS5_L2_axis();
        volatile int8_t r2_axis = DS5_R2_axis();

        volatile uint8_t Dpad_val = DS5_Dpad_value();


        // remove print statement for actual code, this is just for debugging. it significantly slows down the code as it is a blocking function.
        // for now it seems to work fast enough on the Pico 2W but printf statements are not recommended when making a robot with this code.
        printf("Left:  X= %d, Y=%d | Right:  X= %d, Y=%d | Pads:  L2= %d, R2=%d | Dpad: %d\n", left_x, left_y, right_x, right_y, l2_axis, r2_axis, Dpad_val);
    


        if (DS5_touchpad_button()){
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, true);
        }
        else{
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
        }
        
        //use this function when main loop is empty. this makes sure the controller backend runs. without it, the controller won't connect
        //tight_loop_contents();
    }


    return 0;
}