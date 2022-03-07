#include <Wire.h>
#include <SparkFun_FS3000_Arduino_Library.h>

volatile bool int_flag = false;

FS3000 fs;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  noInterrupts();
  pinMode(2, INPUT); // enables int0 pin as input
  EICRA = 3; // Configures interupt 0 to trigger on rising edge on INT0 pin
  EIMSK = 1; // enables int0 interupt
  interrupts(); 

}

// ISRs vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
ISR(INT0_vect){
  int_flag = true; // sets int_flag
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void loop()
{
    if(int_flag){
      for(int i=0; i<=5; i++){
        Serial.println(fs.readMetersPerSecond()); // note, this returns a float from 0-7.23
        delay(1000); // note, repsone time on the sensor is 125ms        
      }
      int_flag = false; // resets int_flag      
    }
}
