union UNION
{
    int test;
    byte mbyte[2];
};

void setup()
{
    Serial.begin(115200); 
}

void loop()
{
    union UNION u;
    u.test = 30;

    for (int i = 0; i < 2; i++)
    {
        Serial.print("mbyte[");
        Serial.print(i);
        Serial.print("] = ");
        Serial.println(u.mbyte[i]);
    }
    Serial.println(); 

    Serial.println(u.test);

    delay(1000); // 1초 대기
}