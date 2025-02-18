#include "differential_car.h"

// Initialize static variables
JohnsonMotor *DifferentialCar::left_motor, *DifferentialCar::right_motor;

/// @brief Wrapper function to update the speed of the individual motors
/// @param void
void DifferentialCar::update_speed(void) {
    left_motor->update_speed();
    right_motor->update_speed();
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
    this->left_motor = new JohnsonMotor(left_in1, left_in2, left_pwm, left_enc_a, left_enc_b, DifferentialCarNS::TIMER_CAR_S, left_kp, left_ki);
    this->right_motor = new JohnsonMotor(right_in1, right_in2, right_pwm, right_enc_a, right_enc_b, DifferentialCarNS::TIMER_CAR_S, right_kp, right_ki);
    this->left_motor->attach_interrupt(this->left_motor_isr);
    this->right_motor->attach_interrupt(this->right_motor_isr);
}

/// @brief Initialize the Timer1 for recurrent speed update
/// @param void
void DifferentialCar::init(void) {
    TCCR1A = 0;              // Initialize Timer1 Control Registers
    TCCR1B = 0;              // Initialize Timer1 Control Registers
    TCCR1B |= (DifferentialCarNS::TIMER_CAR_PRESCALAR == 64)? 0b00000011: (DifferentialCarNS::TIMER_CAR_PRESCALAR == 8)? 0b00000010 : 0b00000001; // Set Prescalar
    TCNT1 = DifferentialCarNS::TIMER_CAR_PRELOAD; // Timer Preloading
    TIMSK1 |= B00000001;     // Enable Timer Overflow Interrupt
}

/// @brief Kinematic model to set the linear speed of the car
/// @param speed Linear speed in m/s
void DifferentialCar::set_curve_linear_speed(double speed) {
    const double car_speed_rpm = speed*60/DifferentialCarNS::wheel_circumference;
    const double car_delta_by_side = car_speed_rpm*DifferentialCarNS::curve_factor;
    this->set_speed(car_speed_rpm-car_delta_by_side, car_speed_rpm+car_delta_by_side);
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
void DifferentialCar::set_speed(double left_speed, double right_speed) {
    this->left_motor->set_speed(left_speed);
    this->right_motor->set_speed(right_speed);
}

/// @brief Timer1 Overflow Interrupt Service Routine, reload the timer and update the car speed speed
/// @param VECTOR(TIMER1_OVF_vect)
ISR(TIMER1_OVF_vect) {
    TCNT1 = DifferentialCarNS::TIMER_CAR_PRELOAD;
    DifferentialCar::update_speed();
}