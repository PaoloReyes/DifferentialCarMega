#include "differential_car.h"

// Initialize static variables
JohnsonMotor *DifferentialCar::left_motor, *DifferentialCar::right_motor;
uint32_t DifferentialCar::last_update = micros();
uint32_t DifferentialCar::new_profile_time = millis();
pose_t DifferentialCar::car_pose = {0, 0, 0};
double DifferentialCar::t1, DifferentialCar::t2, DifferentialCar::t3, DifferentialCar::vtop, DifferentialCar::b;
bool DifferentialCar::on_target = true;

/// @brief Wrapper function to update the speed of the individual motors
/// @param void
void DifferentialCar::update_speed(double delta) {
    DifferentialCar::left_motor->update_speed(delta);
    DifferentialCar::right_motor->update_speed(delta);
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
    const double left_speed = (linear_speed - angular_speed*WHEELS_DISTANCE/2.0)/(WHEEL_CIRCUMFERENCE)*60.0;
    const double right_speed = (linear_speed + angular_speed*WHEELS_DISTANCE/2.0)/(WHEEL_CIRCUMFERENCE)*60.0;
    DifferentialCar::set_motors_speed(left_speed, right_speed);
}

void DifferentialCar::update_position(double delta) {
    double real_linear_velocity = (DifferentialCar::right_motor->read_speed()+DifferentialCar::left_motor->read_speed())*(WHEEL_CIRCUMFERENCE)/120.0;
    double real_angular_velocity = (DifferentialCar::right_motor->read_speed()-DifferentialCar::left_motor->read_speed())*(WHEEL_CIRCUMFERENCE)/(60*WHEELS_DISTANCE);
    
    DifferentialCar::car_pose.theta += real_angular_velocity*delta;
    DifferentialCar::car_pose.x += real_linear_velocity*cos(DifferentialCar::car_pose.theta)*delta;
    DifferentialCar::car_pose.y += real_linear_velocity*sin(DifferentialCar::car_pose.theta)*delta;

    Serial.print("X: ");
    Serial.print(DifferentialCar::car_pose.x);
    Serial.print(" Y: ");
    Serial.print(DifferentialCar::car_pose.y);
    Serial.print(" Theta: ");
    Serial.println(DifferentialCar::car_pose.theta);

    // DifferentialCar::set_speed(controller_output, controller_output/CURVE_RADIUS);
    // DifferentialCar::vel((millis()-DifferentialCar::new_profile_time)/1000.0);

    if (!DifferentialCar::on_target) {
        double trapezoidal_speed = DifferentialCar::vel((millis()-DifferentialCar::new_profile_time)/1000.0);
        DifferentialCar::set_speed(trapezoidal_speed, trapezoidal_speed/CURVE_RADIUS);
    }

    // Serial.print("Error: ");
    // Serial.print(error);
    // Serial.print(" Controller Output: ");
    // Serial.println(controller_output);
}

void DifferentialCar::set_target_container(uint8_t container) {
    double distance = DifferentialCar::get_euclidean_distance_to_container(&DifferentialCar::car_pose, 
                                                                            &containers_possition[container]);

    DifferentialCar::generate_profile(distance);
    DifferentialCar::on_target = false;
}

void DifferentialCar::generate_profile(double target) {
    bool vmax_reached = target*AMAX/(2.0*VMAX)-(VMAX/2.0) >= 0;

    if (vmax_reached) {
        DifferentialCar::t1 = VMAX/AMAX;
        DifferentialCar::t2 = -DifferentialCar::t1+target/VMAX+VMAX/AMAX;
        DifferentialCar::t3 = DifferentialCar::t1+DifferentialCar::t2;
        DifferentialCar::vtop = VMAX;
    } else {
        DifferentialCar::t1 = sqrt(target/AMAX);
        DifferentialCar::t2 = DifferentialCar::t1;
        DifferentialCar::t3 = DifferentialCar::t1+DifferentialCar::t2;
        DifferentialCar::vtop = target/DifferentialCar::t1;
    }
    DifferentialCar::b = AMAX*DifferentialCar::t3;
    DifferentialCar::new_profile_time = millis();
}

double DifferentialCar::vel(double t) {
    if (t >= 0 && t < DifferentialCar::t1) return AMAX*t;
    if (t >= DifferentialCar::t1 && t <= DifferentialCar::t2) return DifferentialCar::vtop;
    if (t > DifferentialCar::t2 && t <= DifferentialCar::t3) return -AMAX*(t)+DifferentialCar::b;
    return 0;
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
    DifferentialCar::left_motor->set_speed(left_speed);
    DifferentialCar::right_motor->set_speed(right_speed);
}

double DifferentialCar::get_euclidean_distance_to_container(pose_t *car_pose, const container_position_t *target) {
    return sqrt(pow(car_pose->x - target->x, 2) + pow(car_pose->y - target->y, 2));
}

/// @brief Timer1 Overflow Interrupt Service Routine, reload the timer and update the car speed speed
/// @param VECTOR(TIMER1_OVF_vect)
ISR(TIMER1_OVF_vect) {
    TCNT1 = TIMER_CAR_PRELOAD;
    if (micros() > DifferentialCar::last_update) {
        double delta = (double)(micros()-DifferentialCar::last_update)/1000000.0;
        DifferentialCar::update_speed(delta);
        DifferentialCar::update_position(delta);
        DifferentialCar::last_update = micros();
    }
}