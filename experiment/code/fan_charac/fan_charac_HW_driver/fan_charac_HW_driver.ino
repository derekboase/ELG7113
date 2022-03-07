#include <Wire.h>
#include <SparkFun_FS3000_Arduino_Library.h>
const byte pin_fan = 3;
volatile bool int_flag = false;

FS3000 fs;

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  pinMode(pin_fan, OUTPUT);
  analogWrite(pin_fan, 0);

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
      for(int i=180; i<=255; i += 5){
        analogWrite(pin_fan, i);
        Serial.println(map_float(i, 0, 255, 0, 100));
        Serial.println(fs.readMetersPerSecond()); // note, this returns a float from 0-7.23
        delay(2000); // note, repsone time on the sensor is 125ms        
      }
      analogWrite(pin_fan, 0);
      int_flag = false; // resets int_flag      
    }
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
