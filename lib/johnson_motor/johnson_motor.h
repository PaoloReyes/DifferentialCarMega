#ifndef JOHNSON_MOTOR_H
    #define JOHNSON_MOTOR_H
    
    #include <Arduino.h>

    class JohnsonMotor {
        public:
            JohnsonMotor(uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t enc_a, uint8_t enc_b, double kp, double ki);

            void attach_interrupt(void (*isr)(void));
            void enc_isr(void);

            void set_pwm(double speed);

            void set_speed(double speed);
            double read_speed(void);
            void update_speed(uint32_t delta);

        private:
            uint8_t in1, in2, pwm, enc_a, enc_b;

            uint8_t prev_enc_bin = 0;
            double kp = 0.0, ki = 0.0, speed = 0.0, error_sum = 0.0;

            volatile double speed_setpoint = 0.0;
            volatile bool speed_control_flag = false;
            volatile int32_t encoder_count = 0, prev_encoder_count = 0;

            uint8_t read_enc_bin(void);
            void set_pwm_internal(double speed);
    };

#endif