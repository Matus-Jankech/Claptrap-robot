/* Inludes */
#include <Control.h>
#include <Adafruit_NeoPixel.h>

/* Defines */
#define NumPixels 4

/* Functions */
void LEDs_init(void){
    Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NumPixels, Leds_PIN, NEO_GRBW + NEO_KHZ800);

    pixels.begin(); 
    for(int i; i < 4; i++){
       pixels.setPixelColor(i, pixels.Color(0,0,255));  
    }
    pixels.setBrightness(255);
    pixels.show();
}
