/* Includes */
#include <Control.h>

/* Constructor definition */
Claptrap::Claptrap(){

}

/* Method definitions */
void Claptrap::begin(){
    motors_init();
    MP3_init();
    encoders_init();
    radio_init();
    LEDs_init();
}