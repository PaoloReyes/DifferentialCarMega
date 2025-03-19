#include <constants.h>
#include <differential_car.h>
#include <SoftwareSerial.h>
#include <utils.h>

DifferentialCar car(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_EN, 
                    LEFT_MOTOR_ENC_A, LEFT_MOTOR_ENC_B, LEFT_MOTOR_KP, LEFT_MOTOR_KI, 
                    RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_EN, 
                    RIGHT_MOTOR_ENC_A, RIGHT_MOTOR_ENC_B, RIGHT_MOTOR_KP, RIGHT_MOTOR_KI);
RS485 rs485(&Serial2, 9600, READ_WRITE_ENABLER);

void move(DifferentialCar& car, double speed, double time);

void setup(void) {
  Serial.begin(115200);
  car.init();
  car.set_speed(linear_speed, linear_speed/CURVE_FACTOR);
  delay(7000);
  car.set_speed(0, 0);
  // rs485.write("OP 1\r\n");
  // Serial.println(rs485.read());
}

const double linear_speed = 0.3; //m/s
void loop(void) {
  // rs485.write("GG\r\n");
  // Serial.println(rs485.read());
  // delay(1000);
}