#include "sbus.h"
#include <Servo.h>
#include <MsTimer2.h>

#define Sbus_Speed_max  1800
#define Sbus_Speed_zero  983
#define Sbus_Speed_min   210

#define SPEED_SERVO_PIN  8

#define CONTROL 0
#define BRAKE 1

#define NERUAL_MOTOR_SPEED 90
#define MOTOR_SPEED_OFFSET 0

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1);
/* SBUS data */
bfs::SbusData data;

Servo servo_speed;
int control_mode =  0;
int DEBUG = 1;
int motor_speed = NERUAL_MOTOR_SPEED + MOTOR_SPEED_OFFSET;

void Ms2Timer_setup(void)
{
  //MsTimer2::set(10, MsTimerISR);
  //MsTimer2::start();
}

void setup()
{
  Serial.begin(115200);
  servo_speed.attach(SPEED_SERVO_PIN);
  servo_speed.write(NERUAL_MOTOR_SPEED + MOTOR_SPEED_OFFSET);

  while (!Serial) {}
  sbus_rx.Begin();
  sbus_tx.Begin();

  Ms2Timer_setup();
}

void sbus_control()
{
  if (sbus_rx.Read())
  {
    data = sbus_rx.data();

    /*
        for (int8_t i = 0; i < 20; i++)
        {
          Serial.print(data.ch[i]);
          Serial.print("\t");
        }
        Serial.println("");
    */

    sbus_tx.data(data);
    sbus_tx.Write();

    if (data.ch[2] >= Sbus_Speed_zero)
    {
      motor_speed = map(data.ch[2], Sbus_Speed_zero, Sbus_Speed_max, 90, 160);
    }
    else
    {
      motor_speed = map(data.ch[2], Sbus_Speed_min, Sbus_Speed_zero, 30, 90);
    }
  }
}

void joycontrol()
{
  if (data.ch[6] <= 300)
  {
    control_mode = CONTROL;
  }
  else
  {
    control_mode = BRAKE;
  }
}

void MsTimerISR()
{
  sbus_control();
  joycontrol();
}

void loop ()
{
  sbus_control();
  joycontrol();

  switch (control_mode)
  {
    case CONTROL :
      servo_speed.write(motor_speed);
      
      if (DEBUG)
      {
        Serial.println("CONTROL");
        Serial.print("speed : ");
        Serial.println(motor_speed - 90);
        Serial.println(" ");
      }
      break;

    case BRAKE :
      servo_speed.write(90);

      if (DEBUG)
      {
        Serial.println("BRAKE");
        Serial.println(" ");
      }
      break;
  }
}
