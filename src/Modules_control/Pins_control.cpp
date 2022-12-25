/* Inludes */
#include <Control.h>

/* Functions */
void pins_init()
{
  /* Define pinmodes */
  pinMode(M1_front_PIN,OUTPUT);
  pinMode(M1_back_PIN,OUTPUT); 
  pinMode(M2_front_PIN,OUTPUT); 
  pinMode(M2_back_PIN,OUTPUT); 
  pinMode(Encoder_A1_PIN,INPUT); 
  pinMode(Encoder_B1_PIN,INPUT);
  pinMode(Encoder_A2_PIN,INPUT);
  pinMode(Encoder_B2_PIN,INPUT);

  /* Attach hardware interrupts */
  attachInterrupt(digitalPinToInterrupt(Encoder_A1_PIN), read_encoder_1, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_A2_PIN), read_encoder_2, RISING);

  /* Make sure motors are turned off */
  analogWrite(M1_front_PIN,0);
  analogWrite(M2_front_PIN,0);
  analogWrite(M1_back_PIN,0);
  analogWrite(M2_back_PIN,0); 
}