/* Inludes */
#include <Control.h>
#include <Adafruit_NeoPixel.h>

/* Global variables */
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(4, Leds_PIN, NEO_GRBW + NEO_KHZ800);

/* Functions */
void set_eye_color(int r, int g, int b){
    for(int i = 1; i < 4; i++){
       pixels.setPixelColor(i, pixels.Color(r,g,b));  
    }
}

void LEDs_init(void){
    pixels.begin(); 
    set_eye_color(0,0,255);
    pixels.setBrightness(255);
    pixels.show();
}
