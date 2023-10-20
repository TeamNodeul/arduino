#include <Arduino_LSM9DS1.h>
#include <cmath>
int count;
float xOffset = -0.002939;//해당값만큼 더해져서 측정이 됨
float yOffset = -0.009281;//Offset만큼 다시 빼줘야함
float zOffset = -0.026254;//
const float gravity = 9.8;
float MAX;
float MIN = 1000;
void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  //calibrateAccelOffsets(); // 초기 가속도 보정
  count = 0;
  MAX = 0;
  MIN =  1000;
}



float dt = 0.01;
// float time, prev_velocity;
float velocity = 0;
float speed;
float displacement = 0;

int timing = 0;
int phase; // [0,1,2,3] :[시작전, 동작시작, 동작중, 동작끝]

void loop() {
  float x, y, z;
  float aaaa, aa, a;//가속도제곱
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    x -= xOffset;
    y -= yOffset;
    z -= zOffset;

    aa = x*x + y*y + z*z;
    aaaa = aa*aa;


    //a = round(sqrt(x*x + y*y + z*z)*20)/20; // 0.05단위로 양자화
    //a = round(sqrt(x*x + y*y + z*z)*25)/25; // 0.04단위로 양자화
    a = sqrt(x*x + y*y + z*z);
    MAX = max(MAX, a);
    MIN = min(MIN, a);
    //if(-0.02<= z&&z<= 0.02) z = 0;
    //if(z < 0 ) z *= -1;

        //if(-0.1 <= velocity_z && velocity_z<= 0.1) velocity_z = 0;
    //if(0.9<=a&&a<=1.1) a = 1;
    // velocity += (a-1.0) * dt;
    // speed += abs(a-1.0) * dt;

    // displacement += velocity * dt;

    // switch(phase){
    //   case 0:
    //     if(timing >= 10 && aa > 4) {
    //       phase = 1;
    //       count++;
    //       timing = 0;
    //     }
    //     //start = time();
    //     break;
    //   case 1:
    //     if(aa <= 0.8) phase = 2;
    //     break;
    //   case 2:
    //     if(aa > 2) phase = 0;
    //     //if(time() - prev_time > 0.5) phase = 3; //적당한 시간이 흘렀으면 운동했다고 판단
    //     break;
    //   // case 3:
    //   //   if (timing >= 50){
    //   //     //speed = 0;
    //   //     phase = 0;
    //   //     timing = 0;
    //   //   }
    // }


    
    switch(phase){
      case 0://덤벨 들어올리기 전
        if(timing >= 50 && aa > 2){ //위로 들어올림을 감지시
          phase = 1;
          timing = 0;
        }
        break;
      case 1://덤벨 들어올리고 있는 상태
        if(timing >= 20 && aa <= 0.95){
          phase = 2;
          timing = 0;
          break;
        }
      case 2://내리고 있는 중
        if(timing >= 20 && aa > 1.2) { //덤벨 내리다가 가속도감지(=최저점에서 정지)
          phase = 0;
          timing = 0;
          count++;
        }
    }
    
    if(timing >= 100) phase = 0;//한 자세로 1초 이상 유지하면, 준비중 상태로 이동
    

    //Serial.print(zOffset);
    // Serial.print('\t');
    // Serial.print(xOffset,6);
    // Serial.print('\t');
    // Serial.print(yOffset,6);
    // Serial.print('\t');
    // Serial.print(zOffset,6);
    // Serial.print('\t');

    Serial.print(MAX);
    Serial.print('\t');
    Serial.print(MIN);
    Serial.print('\t');


    Serial.print(x,6);
    Serial.print('\t');

    Serial.print(y,6);
    Serial.print('\t');
    Serial.print(z,6);
    Serial.print('\t');
    Serial.print(aa,6);
    Serial.print('\t');

    // Serial.print(z);
    // Serial.print('\t');
    //Serial.print(timing);
    //Serial.print('\t');
    Serial.print(speed);
    Serial.print('\t');
    Serial.print(count);
    Serial.print('\t');
    Serial.println(phase);
  }
  delay(10);
   timing += 1;
}


// 가속도계의 오프셋 보정
void calibrateAccelOffsets() {
  float sumX, sumY, sumZ, accelX, accelY, accelZ;
  sumX = sumY = sumZ = accelX = accelY = accelZ = 0;

  int numSamples = 1000; // 샘플링 횟수

  for (int i = 0; i < numSamples; i++) {
    // sensors_event_t event;
    // IMU.acceleration.getEvent(&event);
    // sumX += event.acceleration.x;
    // sumY += event.acceleration.y;
    // sumZ += event.acceleration.z;
    if(IMU.accelerationAvailable()){
      IMU.readAcceleration(accelX, accelY, accelZ);
      sumX += accelX;
      sumY += accelY;
      sumZ += accelZ;
      
    }
    delay(10);
  }

   xOffset = sumX / numSamples;
   yOffset = sumY / numSamples;
   zOffset = sumZ / numSamples;
}
