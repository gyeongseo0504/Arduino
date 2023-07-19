#include <Wire.h> 
#define EncoderAPin 2
#define EncoderBPin 3

const int pulse_check = 9;
float old_imu_angle = 0.0; 
int delta_imu_angle = 0; 


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  pinMode(pulse_check, OUTPUT);

  pinMode(EncoderAPin, INPUT_PULLUP);
  pinMode(EncoderBPin, INPUT_PULLUP);
}


void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
   {
     digitalWrite(pulse_check, HIGH);
     
     float current_imu_angle = readIMU();
     float delta_imu_angle = current_imu_angle - old_imu_angle;
     
     old_imu_angle = current_imu_angle;
     previousMillis = currentMillis;

     digitalWrite(pulse_check, LOW);
  

    }
}


 
