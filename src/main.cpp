#include <constants.h>
#include <differential_car.h>
#include <SoftwareSerial.h>
#include <utils.h>

DifferentialCar car(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_EN, 
                    LEFT_MOTOR_ENC_A, LEFT_MOTOR_ENC_B, LEFT_MOTOR_KP, LEFT_MOTOR_KI, 
                    RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_EN, 
                    RIGHT_MOTOR_ENC_A, RIGHT_MOTOR_ENC_B, RIGHT_MOTOR_KP, RIGHT_MOTOR_KI);
//RS485 rs485(&Serial2, 9600, READ_WRITE_ENABLER);

void setup(void) {
  Serial.begin(115200);
  car.init();
}

void loop(void) {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    String command = data.substring(0, 2);
    String parameter = data.substring(3, data.length());
    Serial.println(command);
    Serial.println(parameter);
  }
}