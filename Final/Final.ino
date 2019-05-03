
#include "BluetoothSerial.h"
#include "MPU6050_tockn.h"
#include <Wire.h>

BluetoothSerial SerialBT;
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

MPU6050 mpu6050(Wire);

const int endstop1_Pin = 2;
const int endstop2_Pin = 15;
const int motor_Pin1 = 18;
const int motor_Pin2 = 19;
const int Led1_4_Pin = 0;
const int Led2_5_Pin = 4;
const int Led3_6_Pin = 16;
bool button1_state = false;
bool button2_state = false;
bool button3_state = false;
unsigned long timePressButton3 = 0;
unsigned long timePressButton2 = 0;
unsigned long timePressButton1 = 0;
#define STOP 0
#define RUN_FORWARD 1
#define RUN_BACKWARD 2
int motorState = STOP;
bool accelCheck = false;
long timeAccel = 0;
float accelAverage;
void setup()
{
  Serial.begin(9600);
  SerialBT.begin("ESP32");
  pinMode(endstop1_Pin, INPUT);
  pinMode(endstop2_Pin, INPUT);
  pinMode(motor_Pin1, OUTPUT);
  pinMode(motor_Pin2, OUTPUT);
  pinMode(Led1_4_Pin, OUTPUT);
  pinMode(Led2_5_Pin, OUTPUT);
  pinMode(Led3_6_Pin, OUTPUT);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("START" );
}
void loop()
{
  accelAverage = calAccelerator(50);
  Serial.println(accelAverage);
  if (accelAverage > 1.5 && !accelCheck) {
    accelCheck = true;
  }
  if (accelCheck)
  {
    if (accelAverage < 1) {
      turnOnLed(Led1_4_Pin);
      turnOnLed(Led2_5_Pin);
      turnOnLed(Led3_6_Pin);
      delay(2000);
      turnOffLed(Led1_4_Pin);
      turnOffLed(Led2_5_Pin);
      turnOffLed(Led3_6_Pin);
      accelCheck = false;
    }
  }

    readBLE();
    if (button2_state)
    {
      if (motorState == STOP)
      {
        Serial.println("RUN FORWARD" );
        motor_FORWARD();
      }
      else if (motorState == RUN_FORWARD)
      {
        if (readStatusEndstop(endstop1_Pin) == HIGH)
        {
          Serial.println("RUN BACKWARD" );
          motor_BACKWARD();
        }
      }
      else if (motorState == RUN_BACKWARD)
      {
        if (readStatusEndstop(endstop2_Pin) == HIGH)
        {
          Serial.println("Stop" );
          motor_Stop();
          button2_state = false;
        }
      }
    }
    if (button3_state)
    {
      if ((unsigned long)(millis() - timePressButton3) < 200)
      {
        turnOnLed(Led1_4_Pin);
        turnOnLed(Led2_5_Pin);
        turnOnLed(Led3_6_Pin);
      }
      else if ((unsigned long)(millis() - timePressButton3) < 400)
      {
        turnOffLed(Led1_4_Pin);
        turnOffLed(Led2_5_Pin);
        turnOffLed(Led3_6_Pin);
      }
      else
      {
        timePressButton3 = millis();
      }
    }
    if (button1_state)
    {
      if ((unsigned long)(millis() - timePressButton1) < (150 * 1))
      {
        turnOnLed(Led1_4_Pin);
      }
      else if ((unsigned long)(millis() - timePressButton1) < (150 * 2))
      {
        turnOffLed(Led1_4_Pin);
      }
      else if ((unsigned long)(millis() - timePressButton1) < (150 * 3))
      {
        turnOnLed(Led2_5_Pin);
      }
      else if ((unsigned long)(millis() - timePressButton1) < (150 * 4))
      {
        turnOffLed(Led2_5_Pin);
      }
      else if ((unsigned long)(millis() - timePressButton1) < (150 * 5))
      {
        turnOnLed(Led3_6_Pin);
      }
      else if ((unsigned long)(millis() - timePressButton1) < (150 * 6))
      {
        turnOffLed(Led3_6_Pin);
      }
      else
      {
        timePressButton1 = millis();
      }
    }
}
void motor_FORWARD()
{
  motorState = RUN_FORWARD;
  digitalWrite(motor_Pin1, HIGH);
  digitalWrite(motor_Pin2, LOW);
}
void motor_BACKWARD()
{
  motorState = RUN_BACKWARD;
  digitalWrite(motor_Pin1, LOW);
  digitalWrite(motor_Pin2, HIGH);
}
void motor_Stop()
{
  motorState = STOP;
  digitalWrite(motor_Pin1, LOW);
  digitalWrite(motor_Pin2, LOW);
}
int readStatusEndstop(int pin)
{
  delay(50);
  return digitalRead(pin);
}
void turnOnLed(int pin)
{
  digitalWrite(pin, HIGH);
}
void turnOffLed(int pin)
{
  digitalWrite(pin, LOW);
}
int readBLE() {
  int a = SerialBT.read();
  if (a == 49)
  {
    //while(digitalRead(RF_D0) == HIGH){};
    button1_state = !(button1_state);
    timePressButton1 = millis();
    {
      turnOffLed(Led1_4_Pin);
      turnOffLed(Led2_5_Pin);
      turnOffLed(Led3_6_Pin);
    }
    button3_state = false;
    return 0;
  }
  if (a == 50)
  {
    button2_state = !(button2_state);
    timePressButton2 = millis();
    return 2;
  }
  if (a == 51)
  {
    Serial.println("////////////");
    button3_state = !(button3_state);
    {
      turnOffLed(Led1_4_Pin);
      turnOffLed(Led2_5_Pin);
      turnOffLed(Led3_6_Pin);
    }
    timePressButton3 = millis();
    button1_state = false;
    return 3;
  }

  //}
}

float calAccelerator(int N) {
  float accelTotal[100], vAverage = 0;
   unsigned long timeCheck = millis();
  for (int i = 0; i < N; i++) {
      mpu6050.update();
      accelTotal[i] = sqrt(pow(mpu6050.getAccX(), 2) + pow(mpu6050.getAccY(), 2) + pow(mpu6050.getAccZ(), 2));
  }
  for (int i = 0; i < N; i++) {
    vAverage += accelTotal[i] / N;
  }
   Serial.print("Times: ");
   Serial.println(millis() - timeCheck);
  return vAverage;
}


















