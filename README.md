# SELF-BALANCING ROBOT
## Hardware
- BLUEPILL (STM32F103C8T6)
- Motor diver: L298N module
- IMU Sensor: MPU6050 module
- DC Motor: JGA25 (without encoder)
- Power Supply:
  - 4× 18650
  - Buck converter -> 12V
  - 7805 in L298N module -> 5V to Bluepill & MPU650 module 
- Pinout:
  - SDA/SCL: MPU6050 <-> I2C1 (PB6 & PB7)
  - ENA: PA1
  - ENB: PA0
  - IN1: PA5
  - IN2: PA4
  - IN3: PA3
  - IN4: PA2
## Software
- Language: C
- IDE: KeilC
## Reference
[PID Controller](https://wired.chillibasket.com/2015/03/pid-controller/)

## Demo
[![Watch the video](https://img.youtube.com/vi/5QwzpXIb9FE/maxresdefault.jpg)](https://www.youtube.com/watch?v=5QwzpXIb9FE&ab_channel=TZ)

