#include <Arduino_LSM9DS1.h>
#include <cmath>
#define N 40
int count;
const float xOffset = -0.002939;//해당값만큼 더해져서 측정이 됨
const float yOffset = -0.009281;//Offset만큼 다시 빼줘야함
const float zOffset = -0.026254;//

float aa, a;//가속도제곱, 가속도
float avg_acc, prev_acc[N]; //평균 가속도

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

  //calibrateAccelOffsets(); // 초기 가속도 보정
  count = 0;
  avg_acc = 1;
  for(int i=0;i<N;i++) prev_acc[i] = 1;
}


int timing = 0;
int phase; // phase[0,1,2] : [시작전, 상승동작, 최고점 도달]

void loop() {
  float x, y, z;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    x -= xOffset;
    y -= yOffset;
    z -= zOffset;

    a = sqrt(x*x + y*y + z*z);
    aa = x*x + y*y + z*z;

    avg_acc = 0; //최근 N틱(10틱 = 0.2초)동안의 평균 가속도 크기
    for(int i=0;i<N-1;i++){
      avg_acc += prev_acc[i];
      prev_acc[i] = prev_acc[i+1];
    }
    prev_acc[N-1] = a;
    avg_acc += prev_acc[N-1];

    avg_acc /= N;
    avg_acc *= avg_acc;


    switch(phase){
      case 0://덤벨 들어올리기 전
        if(timing >= 35 && avg_acc > 1.6){ //위로 들어올림을 감지시
          phase = 1;
          timing = 0;
        }
        break;
      case 1://덤벨 들어올리고 있는 상태
        if(timing >= 20 && avg_acc <= 0.75){
          phase = 2;
          timing = 0;
        }
        break;
      case 2://내리고 있는 중
        if(timing >= 20 && avg_acc > 1.2) { //덤벨 내리다가 가속도감지(=최저점에서 정지)
          phase = 0;
          timing = 0;
          count++;
        }
      }

      if(timing >= 75) phase = 0; //한 자세로 오래 있으면 오류로 간주 -> 시작 전으로 이동
    // Serial.print(x,6);
    // Serial.print('\t');
    // Serial.print(y,6);
    // Serial.print('\t');
    // Serial.print(z,6);
    // Serial.print('\t');
    //Serial.print(timing);
    //Serial.print('\t');

    Serial.print(count);
    Serial.print('\t');
    Serial.print(phase);
    Serial.print('\t');
    Serial.print(avg_acc,6);
    Serial.print('\t');
    Serial.print(aa,6);
    Serial.print('\t');
    Serial.println(10);
  }
  delay(10);
   ++timing;
}

// 가속도계의 오프셋 보정
/*
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
*/
