#include <Arduino.h>

#include <RH_ASK.h>
#include <SPI.h>
#include <ServoTimer2.h>

#include "main.h"
#include "flags.h"

#define MOTOR_PWM_PIN 9
#define MOTOR_FORWARD_PIN 7
#define MOTOR_BACKWARD_PIN 8

Control motor, steering;
Buttons btn;

ServoTimer2 servo;

RH_ASK remote(2000, 6, 5);

void setup()
{
  Serial.begin(9600);
  initMotor();
  initSteering();

  if (!remote.init())
  {
    Serial.println("Failed to init remote communication");
    exit(1);
  }
}

void loop()
{
  receiveData();
}

void updateSteering()
{
  uint32_t angle = 1500;

  if (steering.direction == STEERING_LEFT)
  {
    angle = map(steering.speed, 0, 255, 1500, 2300);
  }

  if (steering.direction == STEERING_RIGHT)
  {
    angle = map(steering.speed, 0, 255, 1500, 800);
  }

  servo.write(angle);
}

void updateMotor()
{
  analogWrite(MOTOR_PWM_PIN, 255);

  if (motor.direction == DIRECTION_FORWARD)
  {
    digitalWrite(MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
  }
  else if (motor.direction == DIRECTION_BACKWARD)
  {
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, HIGH);
  }
  else
  {
    digitalWrite(MOTOR_FORWARD_PIN, LOW);
    digitalWrite(MOTOR_BACKWARD_PIN, LOW);
  }
}

void receiveData()
{
  uint8_t data[3];
  uint8_t len = sizeof(uint8_t) * 3;

  memset(data, 0, len);

  bool dataReceived = remote.recv(data, &len);
  if (dataReceived == false || data[0] == 0x0)
  {
    //emergencyStop();
    return;
  }

  debugDataSent(data);

  motor.direction = DIRECTION_STOP;
  if (data[0] & REMOTE_FORWARD)
  {
    motor.direction = DIRECTION_FORWARD;
  }

  if (data[0] & REMOTE_BACKWARD)
  {
    motor.direction = DIRECTION_BACKWARD;
  }

  if (data[0] & REMOTE_RIGHT)
  {
    steering.direction = STEERING_RIGHT;
  }

  if (data[0] & REMOTE_LEFT)
  {
    steering.direction = STEERING_LEFT;
  }

  motor.speed = data[1];
  steering.speed = data[2];

  updateSteering();
  updateMotor();
}

void emergencyStop()
{
  motor.direction = DIRECTION_STOP;
  motor.speed = 0;

  steering.direction = STEERING_STRAIGHT;
  steering.speed = 0;

  btn.c = false;
  btn.z = false;
}

void debugDataSent(uint8_t *data)
{
  Serial.print(data[0], BIN);
  Serial.print(" ");
  Serial.print(data[1], BIN);
  Serial.print(" ");
  Serial.print(data[2], BIN);
  Serial.println();
}

void debugControls()
{
  Serial.print("Steering : ");
  Serial.print(steering.direction, DEC);
  Serial.print(" / ");
  Serial.print(steering.speed, DEC);
  Serial.print(" | ");
  Serial.print("Motor : ");
  Serial.print(motor.direction, DEC);
  Serial.print(" / ");
  Serial.print(motor.speed, DEC);
  Serial.println("");
}

void initMotor()
{
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_BACKWARD_PIN, OUTPUT);

  digitalWrite(MOTOR_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_BACKWARD_PIN, LOW);

  digitalWrite(MOTOR_FORWARD_PIN, HIGH);
  digitalWrite(MOTOR_BACKWARD_PIN, LOW);
}

void initSteering()
{
  servo.attach(11);
}