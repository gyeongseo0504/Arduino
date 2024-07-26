/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "sbus.h"
#include <Servo.h>
#include <MsTimer2.h>
#include <Wire.h>

#define Sbus_Speed_max  1796
#define Sbus_Speed_zero  976
#define Sbus_Speed_min   200

#define Sbus_Steer_max  1796
#define Sbus_Steer_zero  997
#define Sbus_Steer_min   200


#define SPEED_SERVO_PIN  8
#define STEER_SERVO_PIN  3

#define MAX_SERVO_ANGLE    27
#define MAX_SERVO_SPEED_FW 40 
#define MAX_SERVO_SPEED_RV 20 
// 이부분 보완해야 함

#define FULL_RC_CONTROL 0
#define SEMI_AUTONOMOUS 1
#define FULL_AUTONOMOUS 2


/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1);
/* SBUS data */
bfs::SbusData data;


/* Servo */
Servo servo_speed;
Servo servo_steer;
int   control_mode        =  0;
int   Auto_Steering_Angle = 90;
int   Auto_Motor_Speed    =  0;
int   DEBUG = 1;
union{
  short data; //rpm값
  char  bytedata[2]; //쪼개는 단위
}m_car_steer_int16;        //rpm값을 1바이트씩 쪼개기 위한 공용체

union
{
  short data ;        //pc로 받는 속도 값
  char  bytedata[2];  //쪼개는 단위
} m_car_speed_int16;  //pc로 받는 속도 데이터 값 공용체

void  MsTimerISR()
{



}

void requestEvent() 
{
  unsigned char s[2] = {0,};
  
  int temp= 30;
  
  s[0]= m_car_steer_int16.bytedata[0];
  s[1]= m_car_steer_int16.bytedata[1];
  Wire.write(s,2); // respond 
 // Serial.println("send");
}

void receiveEvent(int howMany)
{
  unsigned  char a[4];
  a[0] = Wire.read();
  a[1] = Wire.read();
  a[2] = Wire.read();
  a[3] = Wire.read();
 // a[4] = Wire.read();
 // a[5] = Wire.read();

  Auto_Steering_Angle =  a[0]*256 + a[1];
  Auto_Motor_Speed    =  a[2]*256 + a[3];
  
  m_car_steer_int16.bytedata[0] = a[0];
  m_car_steer_int16.bytedata[1] = a[1];
  //
  
  m_car_speed_int16.bytedata[0] = a[2];
  m_car_speed_int16.bytedata[1] = a[3];
  //m_car_speed_int16.data +=90;
  // Serial.println("receive");
/*  
  Serial.print(a[0]);   Serial.print("  ");  Serial.print(a[1]);  Serial.print("  ");
  Serial.print(a[2]);   Serial.print("  ");  Serial.print(a[4]);   Serial.print("  ");
  Serial.print(a[5]);   Serial.print("  ");
  Serial.print(Steering_Angle);   Serial.print("  ");
  Serial.println(Motor_Speed);
*/
}
void setup() 
{
  /* Serial to display data */
  Serial.begin(115200);
  servo_speed.attach(SPEED_SERVO_PIN);  
  servo_steer.attach(STEER_SERVO_PIN);  
  servo_steer.write(90);
  servo_speed.write(90);
   
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();

  /* MsTimer2 */
  MsTimer2::set(1000, MsTimerISR);
  //MsTimer2::start();


  /* I2C */
  Wire.begin(5);    // I2C bus  #5
  Wire.onRequest(requestEvent); // register events
  Wire.onReceive(receiveEvent);
}

void loop () 
{
  int motor_speed    = 90;
  int steering_angle = 90;
  if (sbus_rx.Read()) 
  {
    /* Grab the received data */
    data = sbus_rx.data();
    /* Display the received data */
   
      
    for (int8_t i = 0; i < 1/*data.NUM_CH*/; i++) {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
     Serial.println("");
    /* Display lost frames and failsafe data */
    //Serial.print(data.lost_frame);
    //Serial.print("\t");
    //Serial.println(data.failsafe);
    /* Set the SBUS TX data to the received data */
    sbus_tx.data(data);
    /* Write the data to the servos */
    sbus_tx.Write();

    /*mode 제어 */

        /* 모터 속도 제어*/
    if(data.ch[1] >=Sbus_Speed_zero)
    {
      motor_speed = map(data.ch[1], Sbus_Speed_zero, Sbus_Speed_max, 90,160);
    }
    else
    {
       motor_speed = map(data.ch[1], Sbus_Speed_min, Sbus_Speed_zero, 30 ,90);
    }
    
    /* 차량 방향 제어*/

    if(data.ch[0] >=Sbus_Steer_zero)
    {
      steering_angle = map(data.ch[0], Sbus_Steer_zero, Sbus_Steer_max, 90,160);
    }
    else
    {
       steering_angle = map(data.ch[0], Sbus_Steer_min, Sbus_Steer_zero, 30,90);
    }
    
    if(data.ch[6] >= 1600)
    {
      control_mode = FULL_AUTONOMOUS;
    }
    else if(data.ch[6] <= 300)
    {
      control_mode = FULL_RC_CONTROL;      
    }
    else
    {
      control_mode = SEMI_AUTONOMOUS;
    }
    if(steering_angle >= MAX_SERVO_ANGLE+90)   steering_angle  = 90 + MAX_SERVO_ANGLE;
    if(steering_angle <= 90-MAX_SERVO_ANGLE)   steering_angle  = 90 - MAX_SERVO_ANGLE;
    
    switch(control_mode)
    {
      case FULL_RC_CONTROL :
            servo_speed.write(motor_speed);
            servo_steer.write(steering_angle); 
            if(DEBUG)
            {
              Serial.print("Full RC: ");            
              Serial.print("speed : ");
              Serial.print(motor_speed);
              Serial.print("  steer : ");
              Serial.println(steering_angle);
            }
            break;
             
      case SEMI_AUTONOMOUS :
            servo_speed.write(motor_speed);
            servo_steer.write(steering_angle/*
++++++++++++++++++++++++++++++++++++++++++++++++.data+90*/);
            
            if(DEBUG)
            {
              Serial.print("Semi Auto: ");             
              Serial.print("speed : ");
              Serial.print(motor_speed);
              Serial.print("  steer : ");
              Serial.println(m_car_steer_int16.data); 
            } 
            break;

      case FULL_AUTONOMOUS:
             
            servo_steer.write(m_car_steer_int16.data+90);
            servo_speed.write(m_car_speed_int16.data+90);
            if(DEBUG)
            {
              Serial.print("Full Auto: ");            
              Serial.print("speed : ");                        
              Serial.print(m_car_speed_int16.data);
              Serial.print("  steer : ");
              Serial.println(m_car_steer_int16.data);
            }
            //m_car_speed_int16.data = 0;
            //m_car_steer_int16.data = 0;
            
            break;
    } 
   
  }
}

/*
  FULL_RC_CONTROL 0
#define SEMI_AUTONOMOUS 1
#define FULL_AUTONOMOUS 2
*/
