/*
 Example sketch for the PS4 Bluetooth library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <PS4BT.h>
#include <usbhub.h>
#include <Servo.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#define AIR1 22
#define AIR2 24
#define AIR3 26
#define MOTA1 13
#define MOTA2 11
#define MOTA3 12
#define MOTA4 5
#define MOTA5 3
#define MOTA6 33
#define DIR1 9
#define DIR2 7
#define DIR3 8
#define DIR4 6
#define DIR5 4
#define DIR6 43


Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
//PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
PS4BT PS4(&Btd);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

bool air1State = false;
bool air2State = false;
bool air3State = false;
static int servoState = 0;
static int motaState = 0;
static int bollState = 0;

void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));
  pinMode(AIR1,OUTPUT);
  pinMode(AIR2,OUTPUT);
  pinMode(AIR3,OUTPUT);
  pinMode(MOTA1,OUTPUT);
  pinMode(MOTA2,OUTPUT);
  pinMode(MOTA3,OUTPUT);
  pinMode(MOTA4,OUTPUT);
  pinMode(MOTA5,OUTPUT);
  pinMode(MOTA6,OUTPUT);
  pinMode(DIR1,OUTPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(DIR3,OUTPUT);
  pinMode(DIR4,OUTPUT);
  pinMode(DIR5,OUTPUT);
  pinMode(DIR6,OUTPUT);
  servo1.attach(28);
  servo2.attach(34);
  servo3.attach(47);
  servo4.attach(41);


}

void stopMotors() {
  analogWrite(MOTA1, 0);
  analogWrite(MOTA2, 0);
  analogWrite(MOTA3, 0);
  analogWrite(MOTA4, 0);

  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  digitalWrite(DIR3, LOW);
  digitalWrite(DIR4, LOW);

  Serial.print(F("\r\nMotors stopped"));
}

void loop() {
  Usb.Task();

  if (PS4.connected()) {

    if (PS4.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS4.disconnect();
    }
    else {
      bool isButtonPressed = false;
      delay(2);

      if (PS4.getButtonPress(UP)) {
      Serial.print(F("\r\nFront"));
      analogWrite(MOTA1, 100);
      analogWrite(MOTA2, 100);
      analogWrite(MOTA3, 100);
      analogWrite(MOTA4, 100);
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, LOW);
      digitalWrite(DIR3, LOW);
      digitalWrite(DIR4, LOW);
      isButtonPressed = true;
    } 
    else if (PS4.getButtonPress(RIGHT)) {
      Serial.print(F("\r\nRight"));
      analogWrite(MOTA1, 50);
      analogWrite(MOTA2, 50);
      analogWrite(MOTA3, 50);
      analogWrite(MOTA4, 50);
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, LOW);
      digitalWrite(DIR3, HIGH);
      digitalWrite(DIR4, LOW);
      isButtonPressed = true;
    } 
    else if (PS4.getButtonPress(DOWN)) {
      Serial.print(F("\r\nBack"));
      analogWrite(MOTA1, 100);
      analogWrite(MOTA2, 100);
      analogWrite(MOTA3, 100);
      analogWrite(MOTA4, 100);
      digitalWrite(DIR1, HIGH);
      digitalWrite(DIR2, HIGH);
      digitalWrite(DIR3, HIGH);
      digitalWrite(DIR4, HIGH);
      isButtonPressed = true;
    } 
    else if (PS4.getButtonPress(LEFT)) {
      Serial.print(F("\r\nLeft"));
      analogWrite(MOTA1, 50);
      analogWrite(MOTA2, 50);
      analogWrite(MOTA3, 50);
      analogWrite(MOTA4, 50);
      digitalWrite(DIR1, LOW);
      digitalWrite(DIR2, HIGH);
      digitalWrite(DIR3, LOW);
      digitalWrite(DIR4, HIGH);
      isButtonPressed = true;
    }
          else if(PS4.getButtonPress(L1)){
         Serial.print(F("\r\nRotate Left"));
        PS4.setLed(Lightblue);
        analogWrite(MOTA1,25);
        analogWrite(MOTA2,25);
        analogWrite(MOTA3,25);
        analogWrite(MOTA4,25);
        digitalWrite(DIR1,0);
        digitalWrite(DIR2,1);
        digitalWrite(DIR3,1);
        digitalWrite(DIR4,0);
        isButtonPressed = true;
      }
      else if(PS4.getButtonPress(R1)){
         Serial.print(F("\r\nRotate Right"));
        PS4.setLed(Lightblue);
        analogWrite(MOTA1,25);
        analogWrite(MOTA2,25);
        analogWrite(MOTA3,25);
        analogWrite(MOTA4,25);
        digitalWrite(DIR1,1);
        digitalWrite(DIR2,0);
        digitalWrite(DIR3,0);
        digitalWrite(DIR4,1);
        isButtonPressed = true;
      }

    if (!isButtonPressed) {
      stopMotors();


    /*  if (PS4.getButtonClick(TRIANGLE)) {
        air1State = !air1State;
        Serial.print(F("\r\nAIR1 toggled: "));
        Serial.println(air1State ? "ON" : "OFF");
        digitalWrite(AIR1, air1State ? HIGH : LOW);
      }

      if (PS4.getButtonClick(CIRCLE)) {
        air2State = !air2State;
        Serial.print(F("\r\nAIR2 toggled: "));
        Serial.println(air2State ? "ON" : "OFF");
        digitalWrite(AIR2, air2State ? HIGH : LOW);
      }

      if (PS4.getButtonClick(SQUARE)) {
        air3State = !air3State;
        Serial.print(F("\r\nAIR3 toggled: "));
        Serial.println(air3State ? "ON" : "OFF");
        digitalWrite(AIR3, air3State ? HIGH : LOW);
      }
    */

    if (PS4.getButtonPress(TRIANGLE)) {
        Serial.print(F("\r\nTriangle"));
        if (PS4.getButtonClick(TRIANGLE)) {
      if(motaState == 0){
        analogWrite(MOTA5,50);
        digitalWrite(DIR5,0);
        motaState = 1;
      }
      else if (motaState == 1){
        analogWrite(MOTA5,66);
        digitalWrite(DIR5,0);
        motaState = 2;
      }
      else {
        analogWrite(MOTA5,0);
        digitalWrite(DIR5,0);
        motaState = 0;
       }
      }
    }
    if (PS4.getButtonPress(CIRCLE)) {
        Serial.print(F("\r\nCircle"));
        if (PS4.getButtonClick(CIRCLE)) {
     if (servoState == 0) {
        servo1.write(120); // サーボ1を 0° → 90°
        servo2.write(0);
        Serial.println("Servo1: 0 -> 90");
        servoState = 1;
     } 
     else if (servoState == 1) {
        servo1.write(0);
        servo2.write(0);  // サーボ2を 90° → 0°
        Serial.println("Servo2: 90 -> 0");
        servoState = 2;
     } 
     else if (servoState == 2) {
        servo1.write(0);  // サーボ1を 90° → 0°
        servo2.write(180);
        Serial.println("Servo1: 90 -> 0");
        servoState = 3;
     } 
     else {
        servo1.write(120);
        servo2.write(150); // サーボ2を 0° → 90°（最初の状態に戻る）
        Serial.println("Servo2: 0 -> 90");
        servoState = 0; // 初期状態に戻す
     }
     }
    }
        if (PS4.getButtonPress(CROSS)) {
        Serial.print(F("\r\nCross"));
        if (PS4.getButtonClick(CROSS)) {
     if (bollState == 0) {
        servo3.write(180); // サーボ1を 0° → 90°
        servo4.write(180);

 
        bollState = 1;
     } 
     else if (bollState == 1) {
        servo3.write(0);
        servo4.write(0);  // サーボ2を 90° → 0°

        bollState = 2;
     } 
     else if (bollState == 2) {
        servo3.write(0);  // サーボ1を 90° → 0°
        servo4.write(180);
   
        bollState = 3;
     } 
      else if (bollState == 3) {
        servo3.write(180);  // サーボ1を 90° → 0°
        servo4.write(180);
   
        bollState = 4;
      }
     else if(bollState == 4){
      digitalWrite(MOTA6,HIGH);
      digitalWrite(DIR6,LOW);
      
        bollState = 5; // 初期状態に戻す
     }
     else {
      digitalWrite(MOTA6,LOW);
      digitalWrite(DIR6,LOW);
      bollState =0;
     }
     
    }
      
      

      



    }
  }
}
  }
}


