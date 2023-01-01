/* Inludes */
#include <Claptrap.h>

/* Methods definition */
void Claptrap::MP3_play(uint8_t track_num){
    mp3.play(track_num);
}

void Claptrap::MP3_set_volume(uint8_t volume){
    mp3.volume(volume);
}

void Claptrap::MP3_begin(){  
    Serial1.begin(9600);
    mp3.begin(Serial1);
    MP3_set_volume(10);
    MP3_play(2);
}
