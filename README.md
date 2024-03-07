# arduino ino파일(cpp기반) Repository - 주의점

여러 시행착오로 인해, 소스코드를 여러개 업로드하게 되었으니 지정된 **최신화된 코드**를 사용할수있도록 합니다.
### 최신화된 코드(23.12.05. ver)
counting_with_BLE.ino -> timing조절 필수, 측정 오차가 존재할 수있음

<br>

![image](https://github.com/TeamNodeul/arduino/assets/69042677/61198534-4450-4f77-97a9-dc6552928dc4)
# 구현된 것
1. 아두이노 IMU(여기선 가속도센서만 이용)를 이용한 운동 측정 로직
2. BLE통신을 통해, 스마트폰으로 측정데이터 전달
3. 리액트네이티브와 BLE 통신
   
# 구현해야될 것
1. NFC태그 시 블루투스 연결
