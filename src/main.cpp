#include <differential_car.h>
#include <constants.h>
#include <utils.h>

DifferentialCar car (LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_EN, 
                      LEFT_MOTOR_ENC_A, LEFT_MOTOR_ENC_B, LEFT_MOTOR_KP, LEFT_MOTOR_KI, 
                      RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_EN, 
                      RIGHT_MOTOR_ENC_A, RIGHT_MOTOR_ENC_B, RIGHT_MOTOR_KP, RIGHT_MOTOR_KI);

RS485 rs485(&Serial, READ_WRITE_ENABLER);

void setup(void) {
  Serial.begin(115200);
  car.init();
  rs485.write("OP 1");
}

void loop(void) {
  Serial.println(rs485.read());
}