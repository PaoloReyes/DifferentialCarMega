#ifndef DIFFERENTIAL_CAR_H
    #define DIFFERENTIAL_CAR_H
    
    #include <Arduino.h>
    #include <johnson_motor.h>

    #define TIMER_CAR_MAX 65535
    #define TIMER_CAR_S 0.15
    #define TIMER_CAR_PRESCALAR 64
    #define TIMER_CAR_PRELOAD TIMER_CAR_MAX-(TIMER_CAR_S*F_CPU/TIMER_CAR_PRESCALAR)

    #define WHEEL_CIRCUMFERENCE 2.5*2.54*PI/100
    #define WHEELS_DISTANCE 0.2
    #define CURVE_FACTOR 0.932

    class DifferentialCar {
        public:
            static uint32_t last_update;
            static double real_linear_velocity, real_angular_velocity;
            static JohnsonMotor *left_motor, *right_motor;
            static void update_speed(void);

            DifferentialCar(uint8_t left_in1, 
                            uint8_t left_in2, 
                            uint8_t left_pwm, 
                            uint8_t left_enc_a, 
                            uint8_t left_enc_b, 
                            double left_kp, 
                            double left_ki,
                            uint8_t right_in1, 
                            uint8_t right_in2, 
                            uint8_t right_pwm, 
                            uint8_t right_enc_a, 
                            uint8_t right_enc_b,
                            double right_kp, 
                            double right_ki);
            void init(void);

            void set_speed(double linear_speed, double angular_speed);
            void move(double setpoint_distance);

        private:
            static void left_motor_isr(void);
            static void right_motor_isr(void);
            
            void set_motors_speed(double left_speed, double right_speed);
            double get_distance(void);
    };

    ISR(TIMER1_OVF_vect);
#endif