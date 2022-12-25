/* Inludes */
#include <Control.h>
#include <DFPlayerMini_Fast.h>

void MP3_init(){
    DFPlayerMini_Fast mp3;
    Serial1.begin(9600);
    delay(10);
    mp3.begin(Serial1);
    mp3.volume(20);
}