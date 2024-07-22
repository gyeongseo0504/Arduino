void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print("a");
  delay(1000);
}

void serialEvent()
{
  int recived_data_no = 0;
  unsigned char test = 0;
  recived_data_no = Serial.available();
  for(int i =0; i<recived_data_no; i++)
  {
    test = Serial.read();
    Serial.write(test);
    //Serial.print(test);
  }
  
  //Serial.println(recived_data_no);
  //Serial.println("Recived");
}
