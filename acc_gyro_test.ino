#include <Wire.h>
#include <Adafruit_LSM9DS1.h>

Adafruit_LSM9DS1 lsm;

const float gravity = 9.81; // 중력 가속도 (m/s^2)

float velocity = 0.0; // 초기 속도
float position = 0.0; // 초기 위치
float angle = 0.0;    // 초기 각도

unsigned long previousTime = 0;

void setup() {
  Serial.begin(9600);

  if (!lsm.begin()) {
    Serial.println("Could not find a valid LSM9DS1 sensor, check wiring!");
    while (1);
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void loop() {
  lsm.read();

  sensors_event_t accel_event, mag_event, gyro_event, temp_event;
  lsm.getEvent(&accel_event, &mag_event, &gyro_event, &temp_event);

  float accel_x = accel_event.acceleration.x;
  float accel_y = accel_event.acceleration.y;
  float accel_z = accel_event.acceleration.z;

  float gyro_x = gyro_event.gyro.x;
  float gyro_y = gyro_event.gyro.y;
  float gyro_z = gyro_event.gyro.z;

  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0; // 초 단위로 변환

  // 회전 각도 업데이트 (간단한 적분)
  angle += gyro_z * elapsedTime;

  // 중력 제거
  float accel_no_gravity_x = accel_x - gravity * sin(angle);
  float accel_no_gravity_y = accel_y + gravity * cos(angle);
  float accel_no_gravity_z = accel_z - gravity;

  // 가속도 데이터를 이용한 속도 및 변위 계산 (단순 적분)
  velocity += accel_no_gravity_z * elapsedTime;
  position += velocity * elapsedTime;

  previousTime = currentTime;

  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" meters, Velocity: ");
  Serial.print(velocity);
  Serial.print(" m/s, Angle: ");
  Serial.print(angle);
  Serial.println(" radians");

  delay(100); // 측정 주기에 따라 적절한 딜레이를 추가합니다.
}