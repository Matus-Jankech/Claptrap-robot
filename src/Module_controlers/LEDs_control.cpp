/* Inludes */
#include <Claptrap.h>

/* Methods definition */
void Claptrap::set_eye_color(uint8_t r, uint8_t g, uint8_t b){
    for(int i = 0; i < 4; i++){
       pixels.setPixelColor(i, pixels.Color(r,g,b));  
    }
    pixels.show();
}

void Claptrap::LEDs_begin(){
    pixels = Adafruit_NeoPixel(4, Leds_PIN, NEO_GRBW + NEO_KHZ800);
    pixels.begin(); 
    Claptrap::set_eye_color(0,0,255);
    pixels.setBrightness(255);
    pixels.show();
}
