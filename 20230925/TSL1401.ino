#define A0pin  A0
#define SIpin  22
#define CLKpin  23
#define NPIXELS 128

byte Pixel[NPIXELS];

int LineSensor_Data[NPIXELS];
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

void loop()
{
  read_line_camera();
  
  for(int i = 0; i < NPIXELS; i++)
  {
    Serial.println((byte)Pixel[i] + 1);
    
  }
}
