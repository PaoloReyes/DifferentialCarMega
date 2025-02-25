#include <constants.h>
#include <differential_car.h>
#include <SoftwareSerial.h>
#include <utils.h>

#define BUZZER_PIN 11
double grams[5] = {3000.0, 8.5, 76.9, 7.2, 36.5};

DifferentialCar car(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, LEFT_MOTOR_EN, 
                    LEFT_MOTOR_ENC_A, LEFT_MOTOR_ENC_B, LEFT_MOTOR_KP, LEFT_MOTOR_KI, 
                    RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, RIGHT_MOTOR_EN, 
                    RIGHT_MOTOR_ENC_A, RIGHT_MOTOR_ENC_B, RIGHT_MOTOR_KP, RIGHT_MOTOR_KI);
RS485 rs485(&Serial2, 9600, READ_WRITE_ENABLER);

void move(DifferentialCar& car, double speed, double time);

void setup(void) {
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.begin(115200);
  car.init();
  rs485.write("OP 1\r\n");
  move(car, 0.3, 1300);
  delay(3000);
  move(car, 0.3, 7250);
  delay(3000);
  move(car, 0.3, 8050);
  delay(3000);
  move(car, 0.3, 5850);
  delay(3000);
  move(car, 0.3, 4000);
}

void loop(void) {
}

void move(DifferentialCar& car, double speed, double time) {
  car.set_curve_linear_speed(speed);
  delay(time);
  car.set_curve_linear_speed(0);
}

// void wait_until_filled(RS485& rs485, uint32_t target) {
//   rs485.write("GG\r\n");
//   String response = rs485.read();
//   while (!response.startsWith("G")) {
//     rs485.write("GG\r\n");
//     response = rs485.read();
//     delay(50);
//   }
//   for (int i = 0; i < 10; i++) {
//     rs485.write("GG\r\n");
//     response = rs485.read();
//     delay(50);
//   }
//   response = response.substring(2, 8);
//   response.replace(".", "");
//   uint32_t offset = response.toInt();
  
//   uint32_t current_grams = 0;
//   while (current_grams < target) {
//     rs485.write("GG\r\n");
//     response = rs485.read();
//     while (!response.startsWith("G")) {
//       rs485.write("GG\r\n");
//       response = rs485.read();
//       delay(50);
//     }
//     response = response.substring(2, 8);
//     response.replace(".", "");
//     if (response.toInt() - offset < 4000000000) {
//       current_grams = response.toInt() - offset;
//     }
//     Serial.println(current_grams);
//     delay(50);
//   }

//   digitalWrite(BUZZER_PIN, HIGH);
//   delay(1000);
//   digitalWrite(BUZZER_PIN, LOW);
//   delay(2000);
// }