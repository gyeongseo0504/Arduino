#include <MsTimer2.h>
#include <NewPing.h>
#include <Servo.h>

#define SONAR_NUM 4
#define MAX_DISTANCE 2000

#define MOTOR_DIR    4
#define MOTOR_PWM    5

#define SERVO_PIN 3

#define NEURAL_ANGLE 0
#define LEFT_STEER_ANGLE -25
#define RIGHT_STEER_ANGLE 25

unsigned long previousMillis = 0;
const long interval = 200;

union {
  float data;
  byte bytedata[4];
} Left_Front_Sonar, Left_Rear_Sonar, Right_Front_Sonar, Right_Rear_Sonar;

NewPing sonar[SONAR_NUM] = {
  NewPing(38, 39, MAX_DISTANCE), // 왼쪽 전방
  NewPing(34, 35, MAX_DISTANCE), // 왼쪽 후방
  NewPing(36, 37, MAX_DISTANCE), // 오른쪽 전방
  NewPing(32, 33, MAX_DISTANCE), // 오른쪽 후방
};

float car_angle = 0.0;

Servo Steeringservo;

void setup() {
#if FASTADC
  ADCSRA |= _BV(ADPS2);
  ADCSRA &= ~(_BV(ADPS1) | _BV(ADPS0));
#endif

  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Steeringservo.attach(SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);

  // MsTimer2::set(20, control_steering); // Uncomment if control_steering is implemented
  // MsTimer2::start(); // Uncomment if MsTimer2 is used

  Serial.begin(115200);  // Ensure this matches the serial monitor speed
}

void motor_control(int speed) {
  if (speed >= 0) {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, speed);
  } else {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, -speed);
  }
}

void steering_control() {
  // Implement steering control logic if needed
}

void read_ultrasonic_sensor(void) {
  Left_Front_Sonar.data = sonar[0].ping_cm() * 10.0;
  Left_Rear_Sonar.data = sonar[1].ping_cm() * 10.0;
  Right_Front_Sonar.data = sonar[2].ping_cm() * 10.0;
  Right_Rear_Sonar.data = sonar[3].ping_cm() * 10.0;

  if (Left_Front_Sonar.data == 0) Left_Front_Sonar.data = MAX_DISTANCE;
  if (Left_Rear_Sonar.data == 0) Left_Rear_Sonar.data = MAX_DISTANCE;
  if (Right_Front_Sonar.data == 0) Right_Front_Sonar.data = MAX_DISTANCE;
  if (Right_Rear_Sonar.data == 0) Right_Rear_Sonar.data = MAX_DISTANCE;
}

void calculate_car_angle(void) {
  float Left_Diff = Left_Front_Sonar.data - Left_Rear_Sonar.data;
  float Right_Diff = Right_Front_Sonar.data - Right_Rear_Sonar.data;
  float car_width = 182.0;

  float Left_Angle = atan2(Left_Diff, car_width) * 180.0 / PI;
  float Right_Angle = atan2(Right_Diff, car_width) * 180.0 / PI;
  car_angle = (Left_Angle + Right_Angle) / 2.0;

  if (car_angle > 90.0) car_angle = 90.0;
  if (car_angle < -90.0) car_angle = -90.0;

  car_angle = -car_angle;
}

void ultrasonic_sensor_serial_print(void) {
  read_ultrasonic_sensor();
  calculate_car_angle();

  Serial.print("Left_Front_Sonar : ");
  Serial.println(Left_Front_Sonar.data);
  Serial.print("Left_Rear_Sonar : ");
  Serial.println(Left_Rear_Sonar.data);

  Serial.print("Right_Front_Sonar : ");
  Serial.println(Right_Front_Sonar.data);
  Serial.print("Right_Rear_Sonar: ");
  Serial.println(Right_Rear_Sonar.data);

  Serial.print("Angle : ");
  Serial.print(car_angle);
  Serial.println(" degrees");
  Serial.println();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ultrasonic_sensor_serial_print();
  }
}
