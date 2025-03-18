#include "johnson_motor.h"

/// @brief Initialize a JohnsonMotor object with encoder and PID controller
/// @param in1 Input 1 pin
/// @param in2 Input 2 pin
/// @param pwm Enable pin
/// @param enc_a Encoder A pin
/// @param enc_b Encoder B pin
/// @param TIMER_S Delta time in seconds for speed calculation
/// @param kp Proportional gain
/// @param ki Integral gain
JohnsonMotor::JohnsonMotor(uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t enc_a, uint8_t enc_b, double kp, double ki) {
    this->in1 = in1;
    this->in2 = in2;
    this->pwm = pwm;
    if (!(enc_a == 2 || enc_a == 3 || enc_a == 18 || enc_a == 19 || enc_a == 20 || enc_a == 21))
        while (true) {Serial.println("Invalid encoder pin A");} 
    if (!(enc_b == 2 || enc_b == 3 || enc_b == 18 || enc_b == 19 || enc_b == 20 || enc_b == 21))
        while (true) {Serial.println("Invalid encoder pin B");} 
    this->enc_a = enc_a;
    this->enc_b = enc_b;
    this->kp = kp;
    this->ki = ki;
    pinMode(enc_a, INPUT_PULLUP);
    pinMode(enc_b, INPUT_PULLUP);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pwm, OUTPUT);
}

/// @brief Set the interrupt service routine for the motor encoder
/// @param isr A pointer to the interrupt service routine
void JohnsonMotor::attach_interrupt(void (*isr)(void)) {
    this->prev_enc_bin = this->read_enc_bin();
    attachInterrupt(digitalPinToInterrupt(this->enc_a), isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->enc_b), isr, CHANGE);
}

/// @brief Static method to be called by the encoder interrupt service routine
/// @param void
void JohnsonMotor::enc_isr(void) {
    uint8_t enc_bin = this->read_enc_bin();
    switch (enc_bin) {
        case 0b00:
            if (this->prev_enc_bin == 0b01) {
                this->encoder_count--;
            } else if (this->prev_enc_bin == 0b10) {
                this->encoder_count++;
            }
            break;
        case 0b01:
            if (this->prev_enc_bin == 0b11) {
                this->encoder_count--;
            } else if (this->prev_enc_bin == 0b00) {
                this->encoder_count++;
            }
            break;
        case 0b10:
            if (this->prev_enc_bin == 0b00) {
                this->encoder_count--;
            } else if (this->prev_enc_bin == 0b11) {
                this->encoder_count++;
            }
            break;
        case 0b11:
            if (this->prev_enc_bin == 0b10) {
                this->encoder_count--;
            } else if (this->prev_enc_bin == 0b01) {
                this->encoder_count++;
            }
            break;
        default:
            break;
    }
    this->prev_enc_bin = enc_bin;
}

/// @brief Set the PWM signal for the motor disregarding the PID controller
/// @param speed [-1, 1] Speed of the motor
void JohnsonMotor::set_pwm(double speed) {
    this->speed_control_flag = false;
    this->set_pwm_internal(speed);
}

/// @brief Set the speed setpoint for the motor enabling the PID controller
/// @param speed Speed setpoint of the motor
void JohnsonMotor::set_speed(double speed) {
    this->speed_control_flag = true;
    this->speed_setpoint = speed;
}

/// @brief Read the current speed of the motor
/// @param void
/// @return RPM of the motor
double JohnsonMotor::read_speed(void) {
    return this->speed;
}

/// @brief Actuates the PID controller and updates the speed of the motor
/// @param delta Time in seconds since the last update
void JohnsonMotor::update_speed(double delta) {
    this->speed = ((this->encoder_count - this->prev_encoder_count) * 60) / (177.6 * delta);
    this->prev_encoder_count = this->encoder_count;
    
    if (this->speed_control_flag) {
        double error = this->speed_setpoint - this->speed;
        this->error_sum += error * delta;

        double pid_output = this->kp * error + this->ki * this->error_sum;
        if (abs(pid_output) > 1) {
            this->error_sum -= error * delta;
        }
        
        this->set_pwm_internal(pid_output);
    }
}

/// @brief Translate encoder pins to binary
/// @param  void
/// @return Binary representation of the encoder pins
uint8_t JohnsonMotor::read_enc_bin(void) {
    return (digitalRead(this->enc_a) << 1) | digitalRead(this->enc_b);
}

/// @brief Translate speed in range [-1, 1] to PWM signal and input pins direction
/// @param speed [-1, 1] Speed of the motor
void JohnsonMotor::set_pwm_internal(double speed) {
    speed = constrain(speed, -1, 1);
    if (speed > 0) {
        digitalWrite(this->in1, HIGH);
        digitalWrite(this->in2, LOW);
    } else if (speed < 0) {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, HIGH);
    } else {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, LOW);
    }
    analogWrite(this->pwm, abs(speed) * 255);
}