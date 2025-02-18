#ifndef DIFFERENTIAL_CAR_H
    #define DIFFERENTIAL_CAR_H
    
    #include <Arduino.h>
    #include <johnson_motor.h>

    namespace DifferentialCarNS {
        const uint16_t TIMER_CAR_MAX = 65535;   // MAX value for 16-bit timer
        const double TIMER_CAR_S = 0.15;        // Delta time in seconds for speed calculation
        const uint8_t TIMER_CAR_PRESCALAR = 64; // Timer prescalar
        const uint16_t TIMER_CAR_PRELOAD = DifferentialCarNS::TIMER_CAR_MAX-(DifferentialCarNS::TIMER_CAR_S*F_CPU/DifferentialCarNS::TIMER_CAR_PRESCALAR); // Timer preload value

        const double wheel_circumference = 2.5*2.54*PI/100; // Wheel circumference in meters
        const double curve_factor = 0.0932;                 // Curve factor for difference in diameter in the curve rails
    }

    class DifferentialCar {
        public:
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

            void set_curve_linear_speed(double speed);

        private:
            static void left_motor_isr(void);
            static void right_motor_isr(void);
            
            void set_speed(double left_speed, double right_speed);
    };

    ISR(TIMER1_OVF_vect);
#endif