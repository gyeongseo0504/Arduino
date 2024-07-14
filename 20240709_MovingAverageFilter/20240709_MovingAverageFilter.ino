#include <MsTimer2.h>

#define Pin A0          // 아날로그 핀 A0을 사용
#define NO_OF_DATA 15   // 이동 평균에 사용할 데이터 수를 정의

float data[NO_OF_DATA] = {0,};  // 이동 평균 계산에 사용할 데이터를 저장할 배열을 0으로 초기화

float ad_value;
float average;

int count = 0;

void MsTimer2_ISR()
{
  //float average = 0.0;
  //float ad_value = 0.0;
  ad_value = (float)analogRead(Pin);
  average = recursive_moving_average(ad_value);

  /*
    Serial.print("ad_value: ");
    Serial.print(ad_value);
    Serial.print("\t");


    Serial.print("average: ");
    Serial.println(average);

    Serial.print("\n");
  */
}

void setup()
{
  Serial.begin(115200);

  MsTimer2::set(100, MsTimer2_ISR);
  MsTimer2::start();
}

// 재귀 이동 평균을 계산하는 함수
float recursive_moving_average(float ad_value)
{
  static float avg = 0.0;  // 평균 값을 저장하는 static 변수

  // 데이터를 한 칸씩 앞으로 이동
  for (int i = 0; i < NO_OF_DATA - 1; i++)
  {
    data[i] = data[i + 1];
    //Serial.print(data[i]);
    //Serial.print("  ");
  }
  data[NO_OF_DATA - 1] = ad_value;  // 새로운 값을 배열의 마지막에 저장
  //Serial.println(data[NO_OF_DATA - 1]);

  //Serial.print("Average_old2: "); Serial.println(avg);  // 이전 평균 값을 출력

  if (count < NO_OF_DATA)
  {
    count++;
  }
  // 새로운 평균 값을 계산
  avg = avg + ((data[NO_OF_DATA - 1] - data[0]) / (float)count);

  return avg;  // 계산된 평균 값을 반환
}

// 일반적인 이동 평균을 계산하는 함수
float recursive_moving_average1(float ad_value)
{
  static float avg = 0.0;  // 평균 값을 저장하는 static 변수
  float sum = 0.0;  // 합계를 저장할 변수를 초기화

  // 데이터를 한 칸씩 앞으로 이동
  for (int i = 0; i < NO_OF_DATA - 1; i++)
  {
    data[i] = data[i + 1];
    //Serial.print(data[i]);
    //Serial.print("  ");
  }
  data[NO_OF_DATA - 1] = ad_value;  // 새로운 값을 배열의 마지막에 저장
  //Serial.println(data[NO_OF_DATA - 1]);

  // 배열의 모든 값을 더함
  for (int i = 0; i < NO_OF_DATA; i++)
  {
    sum += data[i];
  }
  avg = sum / NO_OF_DATA;  // 합계를 데이터 수로 나누어 평균을 계산

  return avg;  // 계산된 평균 값을 반환
}

void loop()
{
  /*
    float average = 0.0;
    float ad_value = 0.0;
    ad_value = (float)analogRead(Pin);
    static float average_old = 0.0;

    Serial.print("Average_old: "); Serial.println(average_old);
    Serial.print("ad_value: ");    Serial.println(ad_value);

    average = recursive_moving_average1(ad_value);
    Serial.print("Average: ");     Serial.println(average);
    average_old = average;
    Serial.print("\n");
  */

  Serial.print("ad_value: ");
  Serial.print(ad_value);
  Serial.print("\t");


  Serial.print("average: ");
  Serial.println(average);
  delay(100);
  //Serial.print("\n");
}
