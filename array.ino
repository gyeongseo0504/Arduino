#include <MsTimer2.h>

static int cnt = 0;
float sonar[100] = {0.0};
bool run_flag  = false;

void setup() 
{
  run_flag  = true;
  Serial.begin(115200);
  MsTimer2::set(10, Read_sonar); 
}

void loop() 
{
  
  if (run_flag  == false)
  {
    for (int i = 0; i < 100; i++)
    {
      Serial.println(sonar[i]);
    }
    
    run_flag  = true;
    cnt = 0;
    MsTimer2::start();
  }
}

void Read_sonar()
{
  cnt++;
  if (cnt >= 100)    
  {
    run_flag  = false;
    MsTimer2::stop();
  }
}
