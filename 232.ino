void setup() 
{
  Serial.begin(115200);
  Serial.begin(115200);
  // put your setup code here, to run once:

}

void loop() 
{
  if(Serial.available())
  {
    char receivechar = Serial.read();
    Serial.write(receivechar);
  }
  if(Serial.available()
  {
    char receivechar = Serial.read();
    Serial.write(transmitchar);
  }
  // put your main code here, to run repeatedly:

}
