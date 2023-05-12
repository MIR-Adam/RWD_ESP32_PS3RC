#include <Ps3Controller.h>
#include "driver/gpio.h"

#define FORWARD 0
#define BACKWARD 1

const char* ps3ControllerMAC = "YOUR:MAC:ADDRESS:WILL:GO:HERE";
//MAC can be obtained with sixaxis tool

int enableRightMotor=22; 
int rightMotorPin1=16;
int rightMotorPin2=17;

int enableLeftMotor=23;
int leftMotorPin1=18;
int leftMotorPin2=19;

int rearMotorSpeedMax = 255; 

const int PWMFreq = 1000; 
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

void disable_onboard_buttons() {
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);
}

void rotateMotors(int rearMotorSpeed, int frontMotorSpeed)
{

  if (rearMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);    
  }
  else if (rearMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);      
  }

  ledcWrite(leftMotorPWMSpeedChannel, frontMotorSpeed);

  ledcWrite(rightMotorPWMSpeedChannel, abs(rearMotorSpeed)); 
}

void notify()
{
  int yAxisValue = Ps3.data.analog.stick.ly;  
  int xAxisValue = Ps3.data.analog.stick.rx;  

  int rearMotorSpeed = map(yAxisValue, 127, -127, -255, 255);
  int motorDirection = 1;

  if (rearMotorSpeed < 0)
  {
    motorDirection = -1;
  }

  rearMotorSpeed = abs(rearMotorSpeed);
  rearMotorSpeed = constrain(rearMotorSpeed, 0, rearMotorSpeedMax);

  int frontMotorSpeed = map(xAxisValue, -127, 127, -255, 255);
  frontMotorSpeed = abs(frontMotorSpeed);
  frontMotorSpeed = constrain(frontMotorSpeed, 0, 255);

  if (xAxisValue < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);      
  }
  else if (xAxisValue > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);      
  }

  if (Ps3.event.button_down.up) {
    rearMotorSpeedMax = min(rearMotorSpeedMax + 13, 255);
    Serial.println("Throttle +5%!");
  } else if (Ps3.event.button_down.down) {
    rearMotorSpeedMax = max(rearMotorSpeedMax - 13, 0);
    Serial.println("Throttle -5%!");
  }

  rotateMotors(rearMotorSpeed * motorDirection, frontMotorSpeed);
}

void onConnect()
{
  Serial.println("Connected!");
}

void onDisConnect()
{
  rotateMotors(0, 0);
  Serial.println("Disconnected!");    
}

void setMotorSpeed(int leftSpeed, int leftDir, int rightSpeed, int rightDir) {

  digitalWrite(leftMotorPin1, leftDir);
  digitalWrite(leftMotorPin2, !leftDir);
  ledcWrite(leftMotorPWMSpeedChannel, leftSpeed);

  digitalWrite(rightMotorPin1, rightDir);
  digitalWrite(rightMotorPin2, !rightDir);
  ledcWrite(rightMotorPWMSpeedChannel, rightSpeed);
}

void setup() {

  disable_onboard_buttons();

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

  setMotorSpeed(0, FORWARD, 0, FORWARD);

  Ps3.begin(ps3ControllerMAC);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);

  Serial.begin(115200);
  Serial.println("Ready.");
}

void loop() {

  if (!Ps3.isConnected()) {
    Serial.println("PS3 controller disconnected. Reconnecting...");
    Ps3.end();
    delay(500); 

    while (!Ps3.isConnected()) {
      Ps3.begin(ps3ControllerMAC);
      delay(500); 

      if (Ps3.isConnected()) {
        Serial.println("PS3 controller reconnected.");
        break;
      }
    }
  }
}
