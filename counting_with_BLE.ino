#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <cmath>
#define N 30
int count;
const float xOffset = -0.000151;//해당값만큼 더해져서 측정이 됨
const float yOffset = -0.027868;//Offset만큼 다시 빼줘야함
const float zOffset = -0.023135;

float aa, a;//가속도제곱, 가속도
float avg_acc, prev_acc[N]; //평균 가속도

// BLE 서비스와 정보를 담을 BLE Characteristic 3개
BLEService sensorService("369f109d-f77e-48f4-8f36-8ec381c6abf2");
BLEStringCharacteristic xSensorLevel("30429e6d-c93f-411d-b35c-381b95f64aba", BLERead | BLENotify, 15);
BLEStringCharacteristic ySensorLevel("de137a85-4c76-4b5a-b70d-a9267ef94774", BLERead | BLENotify, 15);
BLEStringCharacteristic zSensorLevel("66e17cea-0395-4646-9c65-bf6af45eb37f", BLERead | BLENotify, 15);
BLEStringCharacteristic printPhase("921a82c4-02cc-4665-acc5-a979ace621f2", BLERead | BLENotify, 15);
BLEStringCharacteristic printCount("52923a50-8dcc-4452-a92b-d3245c7f6652", BLERead | BLENotify, 15);

void setup() {
  Serial.begin(9600);
  // while (!Serial); //실제 구동시엔 주석처리 - 시리얼이 연결될때만 실행되도록 하기 때문
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  //BLE 부분-------------------------
  pinMode(LED_BUILTIN, OUTPUT);
  if(!BLE.begin()) { 
    Serial.println("starting BLE failed!");
    while(1);
  }

  BLE.setLocalName("불나방");
  BLE.setAdvertisedService(sensorService);
  // sensorService.addCharacteristic(xSensorLevel);
  // sensorService.addCharacteristic(ySensorLevel);
  // sensorService.addCharacteristic(zSensorLevel);
  sensorService.addCharacteristic(printCount);
  sensorService.addCharacteristic(printPhase);
  BLE.addService(sensorService);

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  // --------------------------------


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

  //BLE 연결
  BLEDevice central = BLE.central();
  if(central) {
    Serial.print("Connected to central: ");
    count = 0;
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while(central.connected()) {
      updateCount();
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }  
}

void updateCount() {
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
        if(timing >= 25 && avg_acc > 1.5){ //위로 들어올림을 감지시
          phase = 1;
          timing = 0;
        }
        break;
      case 1://덤벨 들어올리고 있는 상태
        if(timing >= 15 && avg_acc <= 0.8){
          phase = 2;
          timing = 0;
        }
        break;
      case 2://내리고 있는 중
        if(timing >= 12 && avg_acc > 1.3) { //덤벨 내리다가 가속도감지(=최저점에서 정지)
          phase = 0;
          timing = 0;
          count++;
        }
      }

      if(timing >= 100) phase = 0; //한 자세로 오래 있으면 오류로 간주 -> 시작 전으로 이동
    // Serial.print(x,6);
    // Serial.print('\t');
    // Serial.print(y,6);
    // Serial.print('\t');
    // Serial.print(z,6);
    // Serial.print('\t');
    //Serial.print(timing);
    //Serial.print('\t');

    Serial.print("count: ");
    Serial.print(count);
    Serial.print('\t');
    Serial.print("phase: ");
    Serial.print(phase);
    Serial.print("\t");
    Serial.print("acc^2: ");
    Serial.print(aa,6);
    Serial.print("\t\t");
    Serial.print("filtered_acc^2: ");
    Serial.print(avg_acc,6);
    Serial.print('\t');
    Serial.println();

  }
  delay(5);
   ++timing;


  String phaseMessage = "Phase : " + String(phase);
  String countMessage = "Count : " + String(count);

  printPhase.writeValue(phaseMessage);
  printCount.writeValue(countMessage);
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
