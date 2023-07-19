#include <MPU6050_tockn.h>
#include <Wire.h>

#define EncoderAPin 2
#define EncoderBPin 3
#define pulse 9
#define pulse2m 0.000488

const unsigned long width = 100; // 100 ms (10 Hz)

volatile int counter = 0;
volatile int encoderB;

double angle = 0.0;

unsigned long enc_time;
unsigned long enc_time_old;
unsigned long enc_time_diff;

unsigned long previousMillis = 0;

//pulse:410

MPU6050 mpu6050(Wire);

void Encoder()
{
  enc_time = millis();
  enc_time_diff = enc_time - enc_time_old;

  encoderB = digitalRead(EncoderBPin);

  if (encoderB == LOW) {    // ccw
    counter++;
  }
  else {                    // cw
    counter--;
  }
  enc_time_old = enc_time;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(EncoderAPin, INPUT_PULLUP);
  pinMode(EncoderBPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EncoderAPin), Encoder, RISING);

}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= width)
  {
    mpu6050.update();
    angle = mpu6050.getAngleZ();

    previousMillis = currentMillis;
  }
  Serial.print("Heading_Angle = "); Serial.println(mpu6050.getAngleZ());
  Serial.print("Pulse_counter: "); Serial.println(counter);
  Serial.print("Wheel_trick: "); Serial.println((counter * pulse2m));

}
