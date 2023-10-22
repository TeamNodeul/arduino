#include <Arduino_LSM9DS1.h>
#include <cmath>
#define ANNO if(false)
#define N 40
int count;
float xOffset = -0.002939;//해당값만큼 더해져서 측정이 됨
float yOffset = -0.009281;//Offset만큼 다시 빼줘야함
float zOffset = -0.026254;//
const float gravity = 9.8;
float MAX;
float MIN = 1000;

float now, prev[N];
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
  now = 1;
  for(int i=0;i<N;i++) prev[i] = 1;
}


float dt = 0.01;
// float time, prev_velocity;
float velocity = 0;
float speed;
float displacement = 0;



float sum = 0;
int cnt = 0;

int timing = 0;
int phase; // [0,1,2,3] :[시작전, 동작시작, 동작중, 동작끝]

void loop() {
  float x, y, z;
  float aaaa, aa, a;//가속도제곱
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    //x -= xOffset;
    //y -= yOffset;
    z -= zOffset;

    a = sqrt(x*x + y*y + z*z);
    aa = x*x + y*y + z*z;
    aaaa = aa*aa;
    
    // prevvvv = prevvv;
    // prevvv = prevv;
    // prevv = prev;
    // prev = aa;
    now = 0;
    
    for(int i=0;i<N-1;i++){
      now += prev[i];
      prev[i] = prev[i+1];
    }
    prev[N-1] = a;
    now += prev[N-1];
    now = now / N;
    now *= now;
    //a = round(sqrt(x*x + y*y + z*z)*20)/20; // 0.05단위로 양자화
    //a = round(sqrt(x*x + y*y + z*z)*25)/25; // 0.04단위로 양자화
    

    MAX = max(MAX, now);
    MIN = min(MIN, now);
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
        if(timing >= 35 && now > 1.6){ //위로 들어올림을 감지시
          phase = 1;
          timing = 0;
        }
        break;
      case 1://덤벨 들어올리고 있는 상태
        if(timing >= 20 && now <= 0.7){
          phase = 2;
          timing = 0;
          
        }
        break;
      case 2://내리고 있는 중
        if(timing >= 20 && aa > 1.2) { //덤벨 내리다가 가속도감지(=최저점에서 정지)
          phase = 0;
          timing = 0;
          count++;
        }
      }

      if(timing >= 75) phase = 0; //한 자세로 오래 있으면 오류로 간주 -> 시작 전으로 이동


    /*
    {if(now > 1.1 && timing >= 100 && MIN <= 0.95) {
      sum += MIN;
      cnt++;
      Serial.print(MIN);
      Serial.print('\t');
      //Serial.print(sum / cnt);

      MIN = 1;
      timing = 0;
      
    }
    if(now < 0.85 && timing >= 100 && MAX >= 1.05) {
      //sum += MIN;
      cnt++;
      Serial.println(MAX);
      //Serial.print('\t');
      //Serial.println(sum / cnt);

      MAX = 1;
      timing = 0;
      
    }
    }*/


    //Serial.print(zOffset);
    // Serial.print('\t');
    // Serial.print(xOffset,6);
    // Serial.print('\t');
    // Serial.print(yOffset,6);
    // Serial.print('\t');
    // Serial.print(zOffset,6);
    // Serial.print('\t');

    // Serial.print(MAX);
    // Serial.print('\t');
    // Serial.print(MIN);
    // Serial.print('\t');


    // Serial.print(x,6);
    // Serial.print('\t');

    // Serial.print(y,6);
    // Serial.print('\t');
    // Serial.print(z,6);
    // Serial.print('\t');



    // Serial.print(z);
    // Serial.print('\t');
    //Serial.print(timing);
    //Serial.print('\t');
    //Serial.print(speed);
    //Serial.print('\t');
    Serial.print(count);
    Serial.print('\t');
    Serial.print(phase);
    Serial.print('\t');
    Serial.print(now,6);
    Serial.print('\t');
    Serial.print(aa,6);
     Serial.print('\t');
    Serial.println(10);
    
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
