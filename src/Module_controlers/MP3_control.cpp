/* Inludes */
#include <Claptrap.h>

/* Global variables */
DFPlayerMini_Fast mp3;

/* Methods definition */
void Claptrap::MP3_play(int track_num){
    mp3.play(track_num);
}

void Claptrap::MP3_set_volume(int volume){
    mp3.volume(volume);
}

void Claptrap::MP3_init(){
    Serial1.begin(9600);
    mp3.begin(Serial1);
    MP3_set_volume(10);
    MP3_play(2);
}
