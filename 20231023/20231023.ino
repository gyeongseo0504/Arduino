#define A0pin  A0
#define SIpin  22
#define CLKpin  23
#define NPIXELS 128


double kp_vision  = 0.1;
double kd_vision  = 0.3;
double ki_vision  = 0.0;
double error      = 0.0;
double error_old    = 0.0;
double Target     = NPIXELS/2;


byte Pixel[NPIXELS];

int LineSensor_Data[NPIXELS];
byte LineSensor_threshold_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup()
{
  int i;
  for(i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i]          = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i]      = 1023;
    MIN_LineSensor_Data[i]      = 0;  
  }

  pinMode(SIpin, OUTPUT);  //출력
  pinMode(CLKpin, OUTPUT); //출력
  pinMode(A0pin, INPUT);   //입력

  digitalWrite(SIpin, LOW);
  digitalWrite(CLKpin, LOW);

  #if FASTADC

  sbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  #endif

  flag_line_adapation = 0;

  Serial.begin(115200);
  Serial.println("TSL1401");
}
void threshold_line_image(int threshold_value)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if(Pixel[i]>=threshold_value)
    {
      LineSensor_threshold_Data[i] = 255;
    }
    else
    {
      LineSensor_threshold_Data[i] = 0;
    }
  }
}

void read_line_camera(void)
{
  int i;
  delay(1);
  
  digitalWrite(CLKpin, LOW);
  digitalWrite(SIpin, HIGH);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(SIpin, LOW);
  delayMicroseconds(1);
  
  for(i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead (A0pin) * 0.25; //8-bit is enough
    digitalWrite(CLKpin, LOW);
    delayMicroseconds(1);
    digitalWrite(CLKpin, HIGH);
  }
  digitalWrite(CLKpin, LOW);
}

double line_centroid(void)
{
  double centroid = 0.0;
  double mass_sum = 0.0;

  for(int i = 0; i< NPIXELS ; i++)
  {
    mass_sum += LineSensor_threshold_Data[i];
    centroid += LineSensor_threshold_Data[i] * i;
  }
  centroid = centroid / mass_sum;

  return centroid;
}
double PID_control(double line_center)
{
  /*double kp_vision  = 0.1;
  double kd_vision  = 0.3;
  double ki_vision  = 0.0;
  double error      = 0.0;
  double error_old    = 0.0;
  double target     = NPIXELS/2;*/

  int pwm_value  = 0;
  double error_d =0;
  error = Target - line_center;
  error_d = error - error_old;
  pwm_value = int(kp_vision*error + kd_vision*error_old);
  if(pwm_value >= 200) pwm_value = 200;
  if(pwm_value <= -200) pwm_value = -200;
  error_old = error;
  return pwm_value;
}

void vision_line_control(int base_speed, double l_c)
{
  int pwm_control_value = PID_control(l_c);
  int motor_control_left = (base_speed + pwm_control_value);
  int motor_control_right = (base_speed - pwm_control_value);
  
}
void loop()
{
  double c_x = 0;
  int cx_int;
  read_line_camera();
  threshold_line_image(150);
  c_x = line_centroid();
  vision_line_control(50, c_x);
  
  /*for(int i = 0; i < NPIXELS; i++)
  {
    Serial.println((byte)Pixel[i] + 1);
    
  }*/
  
  for(int i = 0; i < NPIXELS; i++)
  {
    Serial.print(LineSensor_threshold_Data[i]+1);
    Serial.print(" ");
  }
  cx_int = (int)c_x;
  Serial.println(c_x);
  Serial.print(" ");
  Serial.println(" ");
  //Serial.println(c_x);
}
