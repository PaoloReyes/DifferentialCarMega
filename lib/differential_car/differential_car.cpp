#include "differential_car.h"

// Initialize static variables
JohnsonMotor *DifferentialCar::left_motor, *DifferentialCar::right_motor;
uint32_t DifferentialCar::last_update = micros();
uint32_t DifferentialCar::new_profile_time = millis();
pose_t DifferentialCar::car_pose = {0, 0, 0};
double DifferentialCar::t1, DifferentialCar::t2, DifferentialCar::t3, DifferentialCar::vtop, DifferentialCar::b, DifferentialCar::c, DifferentialCar::d;
double DifferentialCar::target_distance = 0;
uint16_t DifferentialCar::container_target = 0;
bool DifferentialCar::on_target = true;
bool DifferentialCar::vmax_reached = false;
double DifferentialCar::last_position_error = 0;

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

/// @brief Perform the kinematic model to set the speed of the car using the expected speed from the trapezoidal profile and a proportional controller for the position error
/// @param delta Time since the last update in seconds
void DifferentialCar::update_position(double delta) {
    double real_linear_velocity = (DifferentialCar::right_motor->read_speed()+DifferentialCar::left_motor->read_speed())*(WHEEL_CIRCUMFERENCE)/120.0; //Car speed in m/s
    double real_angular_velocity = (DifferentialCar::right_motor->read_speed()-DifferentialCar::left_motor->read_speed())*(WHEEL_CIRCUMFERENCE)/(60*WHEELS_DISTANCE); //Car angular speed in rad/s
    
    //Update the car pose using euler integration
    DifferentialCar::car_pose.theta += real_angular_velocity*delta;
    DifferentialCar::car_pose.x += real_linear_velocity*cos(DifferentialCar::car_pose.theta)*delta;
    DifferentialCar::car_pose.y += real_linear_velocity*sin(DifferentialCar::car_pose.theta)*delta;

    //Update the car speed using the trapezoidal profile if the car is not on target
    if (!DifferentialCar::on_target) {
        double trapezoidal_speed = DifferentialCar::vel((millis()-DifferentialCar::new_profile_time)/1000.0); //Expected speed from the trapezoidal profile
        double trapezoidal_position = DifferentialCar::pos((millis()-DifferentialCar::new_profile_time)/1000.0); //Expected position from the trapezoidal profile
        double trapezoidal_error = DifferentialCar::target_distance - trapezoidal_position; //Expected position error from the trapezoidal profile
        double car_error = DifferentialCar::get_euclidean_distance_to_container(&DifferentialCar::car_pose, &containers_position[DifferentialCar::container_target]); //Car position error
        double delta_error = car_error - trapezoidal_error; //Position error between the car and the trapezoidal profile expected position
        DifferentialCar::set_speed(trapezoidal_speed+CAR_KP*delta_error, (trapezoidal_speed+CAR_KP*delta_error)/CURVE_RADIUS); //Set the speed of the car using the trapezoidal profile and a proportional controller
        
        //Check if the car is on the least position error
        if (DifferentialCar::last_position_error < car_error) {
            DifferentialCar::on_target = true; //Set the car to on target
            DifferentialCar::set_speed(0, 0);  //Stop the car
        }
        DifferentialCar::last_position_error = car_error; //Update the last position error
    }
}

/// @brief Set a target euclidean distance to a container
/// @param container Target container number
void DifferentialCar::set_target_container(uint8_t container) {
    DifferentialCar::last_position_error = 10000000;
    DifferentialCar::container_target = container;
    // Set the target distance to the container
    DifferentialCar::target_distance = DifferentialCar::get_euclidean_distance_to_container(&DifferentialCar::car_pose, 
                                                                                            &containers_position[DifferentialCar::container_target]);

    DifferentialCar::generate_profile_parameters(); //Generate the trapezoidal profile parameters
    DifferentialCar::on_target = false;             //Set the car to not on target
}

/// @brief Generate the trapezoidal profile parameters based on the trapezoidal motion model, the target distance, max speed and acceleration
void DifferentialCar::generate_profile_parameters(void) {
    DifferentialCar::vmax_reached = DifferentialCar::target_distance*AMAX/(2.0*VMAX)-(VMAX/2.0) >= 0; //Check if the max speed is reached

    //If max speed is reached, generate a trapezoidal profile, if not generate a triangular profile
    if (DifferentialCar::vmax_reached) {
        DifferentialCar::t1 = VMAX/AMAX; //Time to reach max speed
        DifferentialCar::t2 = -DifferentialCar::t1+ DifferentialCar::target_distance/VMAX+VMAX/AMAX; //Time to start decelerating
        DifferentialCar::t3 = DifferentialCar::t1+DifferentialCar::t2; //Expected time to reach the target distance
        DifferentialCar::vtop = VMAX; //Max speed
    } else {
        DifferentialCar::t1 = sqrt(DifferentialCar::target_distance/AMAX); //Time to reach top speed
        DifferentialCar::t2 = DifferentialCar::t1;                         //Time to start decelerating
        DifferentialCar::t3 = DifferentialCar::t1+DifferentialCar::t2;     //Expected time to reach the target distance
        DifferentialCar::vtop = DifferentialCar::target_distance/DifferentialCar::t1; //Max speed reached
    }

    DifferentialCar::b = AMAX*DifferentialCar::t3;    //b parameter from the trapezoidal profile equation
    DifferentialCar::c = -VMAX*DifferentialCar::t1/2; //c parameter from the trapezoidal profile equation
    DifferentialCar::d = DifferentialCar::target_distance-AMAX/2*pow(DifferentialCar::t3, 2); //d parameter from the trapezoidal profile equation
    DifferentialCar::new_profile_time = millis();     //Set the new profile start time
}

/// @brief Get the expected velocity of the car at a given time based on the parameters of the trapezoidal profile
/// @param t Current time
/// @return Expected velocity of the car at time t
double DifferentialCar::vel(double t) {
    if (t >= 0 && t < DifferentialCar::t1) return AMAX*t;
    if (t >= DifferentialCar::t1 && t <= DifferentialCar::t2) return DifferentialCar::vtop;
    if (t > DifferentialCar::t2 && t <= DifferentialCar::t3) return -AMAX*(t)+DifferentialCar::b;
    return 0;
}

/// @brief Get the expected position of the car at a given time based on the parameters of the trapezoidal profile
/// @param t Current time
/// @return Expected position of the car at time t
double DifferentialCar::pos(double t) {
    if (t >= 0 && t < DifferentialCar::t1) return AMAX/2*pow(t, 2);
    if (t >= DifferentialCar::t1 && t <= DifferentialCar::t2 && DifferentialCar::vmax_reached) return VMAX*t+DifferentialCar::c;
    if (t >= DifferentialCar::t1 && t <= DifferentialCar::t2 && !DifferentialCar::vmax_reached) return DifferentialCar::target_distance/2;
    if (t > DifferentialCar::t2 && t <= DifferentialCar::t3) return -AMAX/2*pow(t, 2)+DifferentialCar::b*t+DifferentialCar::d;
    if (t > DifferentialCar::t3) return DifferentialCar::target_distance;
    return 0;
}

/// @brief Wait until the car is on target to break the loop
void DifferentialCar::wait_until_on_target(void) {
    while (!DifferentialCar::on_target) {
        asm("nop");
    }
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

/// @brief Timer1 Overflow Interrupt Service Routine, reload the timer and update the car speed and position
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