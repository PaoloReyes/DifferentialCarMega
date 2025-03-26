#include <constants.h>
#include <differential_car.h>
#include <SoftwareSerial.h>
#include <utils.h>

#define CAR_ID 2

DifferentialCar car(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_EN, 
                    LEFT_MOTOR_ENC_A, LEFT_MOTOR_ENC_B, LEFT_MOTOR_KP, LEFT_MOTOR_KI, 
                    RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_EN, 
                    RIGHT_MOTOR_ENC_A, RIGHT_MOTOR_ENC_B, RIGHT_MOTOR_KP, RIGHT_MOTOR_KI);

typedef enum {
  IDLE,
  WAITING_COMMAND,
} car_state_t;

String get_next_parameter(String *parameters) {
  int index = parameters->indexOf(' ');
  if (index == -1) index = parameters->length();
  String immediate_parameter = parameters->substring(0, index);
  *parameters = parameters->substring(index+1, parameters->length());
  return immediate_parameter;
}

void setup(void) {
  Serial.begin(115200);
  car.init();
}

car_state_t state = IDLE;

void loop(void) {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    String command = data.substring(0, 2);
    String parameters = data.substring(3, data.length());
    switch (state) {
      case IDLE:
        if (command == "OP") {
          String id = get_next_parameter(&parameters);
          if (id.toInt() == CAR_ID) {
            state = WAITING_COMMAND;
            Serial.println("OK");
          } else {
            Serial.println("ERR");
          }
        } else {
          Serial.println("ERR");
        }
        break;
      case WAITING_COMMAND:
        if (command == "MV") {
          String container = get_next_parameter(&parameters);
          if (container.toInt() >= 0 && container.toInt() < CONTAINERS_NUM) {
            car.set_target_container(container.toInt());
            Serial.println(container.toInt());
            Serial.println("OK");
          } else {
            Serial.println("ERR");
          }
        } else if (command == "CL") {
          state = IDLE;
          Serial.println("OK");
        } else {
          Serial.println("ERR");
        }
        break;
      default:
        break;
    }
  }
}