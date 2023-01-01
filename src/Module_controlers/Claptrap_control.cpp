/* Includes */
#include <Claptrap.h>

/* Constructor definition */
Claptrap::Claptrap(){}

/* Method definitions */
void Claptrap::begin(){
    motors_begin();
    MP3_begin();
    encoders_begin();
    radio_begin();
    LEDs_begin();
}