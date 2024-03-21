const int capacitorPin = A0;
unsigned long startTime = 0;

void setup() 
{
  pinMode(capacitorPin, INPUT);
  Serial.begin(115200);
}

void loop()
{
  int rawValue = analogRead(capacitorPin);
  float voltage = (float)rawValue / 1023.0 * 5.0;

  Serial.print("Charging Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");

  if(voltage < 0.01)
  {
    unsigned long endTime = millis();
    unsigned long elapsedTime = endTime - startTime;

    Serial.print("elapsedTime: ");
    Serial.print(elapsedTime);
    Serial.println(" ms");
  }
  pinMode(capacitorPin, OUTPUT);
  digitalWrite(capacitorPin, LOW);
  pinMode(capacitorPin, INPUT);  
}
