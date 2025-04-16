#include <constants.h>
#include <differential_car.h>
#include <SoftwareSerial.h>
#include <utils.h>

#define CAR_ID 2 //Car ID for RS485 communication

//Create car object
DifferentialCar car(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_EN, 
                    LEFT_MOTOR_ENC_A, LEFT_MOTOR_ENC_B, LEFT_MOTOR_KP, LEFT_MOTOR_KI, 
                    RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_EN, 
                    RIGHT_MOTOR_ENC_A, RIGHT_MOTOR_ENC_B, RIGHT_MOTOR_KP, RIGHT_MOTOR_KI);

typedef enum {
  IDLE,
  WAITING_COMMAND,
} car_state_t; //State machine for the car

/// @brief Get the next parameter from the parameters string separated by space
/// @param parameters String containing the parameters
/// @return String containing the next parameter
String get_next_parameter(String *parameters) {
  int index = parameters->indexOf(' ');
  if (index == -1) index = parameters->length();
  String immediate_parameter = parameters->substring(0, index);
  *parameters = parameters->substring(index+1, parameters->length());
  return immediate_parameter;
}

void setup(void) {
  Serial.begin(115200); //Serial communication with the computer
  car.init();           //Initialize the car (Timer interrupt for speed control)
}

car_state_t state = IDLE; //Initial state of the car

void loop(void) {
  //Check if a command is received from the computer
  if (Serial.available() > 0) { 
    String data = Serial.readStringUntil('\n');
    String command = data.substring(0, 2); //Extract the command from the data
    String parameters = data.substring(3, data.length()); //Extract the parameters from the data
    //Switch case for the state machine
    switch (state) {
      //IDLE state
      case IDLE:
        //Check if the command is "OP" (Open) to open the car
        if (command == "OP") {
          String id = get_next_parameter(&parameters);
          if (id.toInt() == CAR_ID) {
            state = WAITING_COMMAND;
            Serial.println("OK");
          }
        }
        break;
      //WAITING_COMMAND state
      case WAITING_COMMAND:
        //Check if the command is "MV" (Move) to move the car
        if (command == "MV") {
          String container = get_next_parameter(&parameters); //Get the container number
          //Check if the container number is valid
          if (container.toInt() >= 0 && container.toInt() < CONTAINERS_NUM) {
            car.set_target_container(container.toInt()); //Set the target container
            car.wait_until_on_target(); //Wait until the car is on the target container
            Serial.println("OK"); //Send OK to the computer
          } else {
            Serial.println("ERR"); //Send ERR to the computer
          }
        //Check if the comman is "CL" (Close) to close the car
        } else if (command == "CL") {
          state = IDLE;         //Set the state to IDLE
          Serial.println("OK"); //Send OK to the computer
        } else {
          Serial.println("ERR"); //Send ERR to the computer
        }
        break;
      default:
        break;
    }
  }
}