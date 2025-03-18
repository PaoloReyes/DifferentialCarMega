#include "differential_car.h"

// Initialize static variables
JohnsonMotor *DifferentialCar::left_motor, *DifferentialCar::right_motor;
uint32_t DifferentialCar::last_update = millis();
double DifferentialCar::real_linear_velocity = 0.0, DifferentialCar::real_angular_velocity = 0.0;

/// @brief Wrapper function to update the speed of the individual motors
/// @param void
void DifferentialCar::update_speed(void) {
    if (micros() > DifferentialCar::last_update) {
        DifferentialCar::left_motor->update_speed((double)(micros()-DifferentialCar::last_update)/1000000.0);
        DifferentialCar::right_motor->update_speed((double)(micros()-DifferentialCar::last_update)/1000000.0);
    }
    // DifferentialCar::real_linear_velocity = (DifferentialCar::right_motor->read_speed()+DifferentialCar::left_motor->read_speed())/2.0;
    // DifferentialCar::real_angular_velocity = (DifferentialCar::right_motor->read_speed()-DifferentialCar::left_motor->read_speed())/WHEELS_DISTANCE;
    // Serial.print("Right Motor: ");
    // Serial.print(DifferentialCar::right_motor->read_speed());
    // Serial.print(" Left Motor: ");
    // Serial.println(DifferentialCar::left_motor->read_speed());
}

/// @brief Create a DifferentialCar object
/// @param left_in1 Input 1 pin for the left motor
/// @param left_in2 Input 2 pin for the left motor
/// @param left_pwm Enable pin for the left motor
/// @param left_enc_a Encoder A pin for the left motor
/// @param left_enc_b Encoder B pin for the left motor
/// @param left_kp Proportional gain for the left motor
/// @param left_ki Integral gain for the left motor
/// @param right_in1 Input 1 pin for the right motor
/// @param right_in2 Input 2 pin for the right motor
/// @param right_pwm Enable pin for the right motor
/// @param right_enc_a Encoder A pin for the right motor
/// @param right_enc_b Encoder B pin for the right motor
/// @param right_kp Proportional gain for the right motor
/// @param right_ki Proportional gain for the right motor
DifferentialCar::DifferentialCar(uint8_t left_in1, uint8_t left_in2, uint8_t left_pwm, uint8_t left_enc_a, uint8_t left_enc_b, double left_kp, double left_ki, uint8_t right_in1, uint8_t right_in2, uint8_t right_pwm, uint8_t right_enc_a, uint8_t right_enc_b, double right_kp, double right_ki) {
    this->left_motor = new JohnsonMotor(left_in1, left_in2, left_pwm, left_enc_a, left_enc_b, left_kp, left_ki);
    this->right_motor = new JohnsonMotor(right_in1, right_in2, right_pwm, right_enc_a, right_enc_b, right_kp, right_ki);
    this->left_motor->attach_interrupt(this->left_motor_isr);
    this->right_motor->attach_interrupt(this->right_motor_isr);
}

/// @brief Initialize the Timer1 for recurrent speed update
/// @param void
void DifferentialCar::init(void) {
    TCCR1A = 0;              // Initialize Timer1 Control Registers
    TCCR1B = 0;              // Initialize Timer1 Control Registers
    TCCR1B |= (TIMER_CAR_PRESCALAR == 64)? 0b00000011: (TIMER_CAR_PRESCALAR == 8)? 0b00000010 : 0b00000001; // Set Prescalar
    TCNT1 = TIMER_CAR_PRELOAD; // Timer Preloading
    TIMSK1 |= B00000001;     // Enable Timer Overflow Interrupt
}

/// @brief Kinematic model to set the speed of the car
/// @param linear_speed Linear speed in m/s
/// @param angular_speed Angular speed in rad/s
void DifferentialCar::set_speed(double linear_speed, double angular_speed) {
    const double left_speed = (linear_speed - angular_speed*WHEELS_DISTANCE/2.0)/WHEEL_CIRCUMFERENCE*60.0;
    const double right_speed = (linear_speed + angular_speed*WHEELS_DISTANCE/2.0)/WHEEL_CIRCUMFERENCE*60.0;
    Serial.print("Left Speed: ");
    Serial.print(left_speed);
    Serial.print(" Right Speed: ");
    Serial.println(right_speed);
    this->set_motors_speed(left_speed, right_speed);
}

void DifferentialCar::move(double setpoint_distance) {
    //double error = setpoint_distance-this->distance;
}

/// @brief Wrapper function for the left motor encoder ISR
/// @param void
void DifferentialCar::left_motor_isr(void) {
    left_motor->enc_isr();
}

/// @brief Wrapper function for the right motor encoder ISR
/// @param void
void DifferentialCar::right_motor_isr(void) {
    right_motor->enc_isr();
}

/// @brief Update the speed setpoint of the individual motors
/// @param left_speed RPM setpoint for the left motor
/// @param right_speed RPM setpoint for the right motor
void DifferentialCar::set_motors_speed(double left_speed, double right_speed) {
    this->left_motor->set_speed(left_speed);
    this->right_motor->set_speed(right_speed);
}

/// @brief Timer1 Overflow Interrupt Service Routine, reload the timer and update the car speed speed
/// @param VECTOR(TIMER1_OVF_vect)
ISR(TIMER1_OVF_vect) {
    TCNT1 = TIMER_CAR_PRELOAD;
    DifferentialCar::update_speed();
    DifferentialCar::last_update = micros();
}