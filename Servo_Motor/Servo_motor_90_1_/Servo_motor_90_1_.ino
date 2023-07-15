const int encoderPinA = 2;
const int encoderPinB = 3;
int PinBValue;
int count = 0; 

void interrupt_encoder()
{
PinBValue = digitalRead(encoderPinB);
if (PinBValue == LOW) 
  {
    count++;
  }
  else
  {
    count--;
  }
}
void setup() 
{// 엔코더 핀을 입력으로 설정
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), interrupt_encoder, RISING);  
}
void loop() //시리얼 모니터만
{
  // 핀 A와 핀 B의 값 시리얼 모니터에 출력
  Serial.print("PinA: 1 ");
  Serial.print("PinB:   ");
  Serial.println(PinBValue);
  Serial.print(count);
  Serial.println(); 
}
