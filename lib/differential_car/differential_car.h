#ifndef DIFFERENTIAL_CAR_H
    #define DIFFERENTIAL_CAR_H
    
    #include <Arduino.h>
    #include <johnson_motor.h>
    #include <../../include/containers.h>

    #define TIMER_CAR_MAX 65535
    #define TIMER_CAR_S 0.15
    #define TIMER_CAR_PRESCALAR 64
    #define TIMER_CAR_PRELOAD TIMER_CAR_MAX-(TIMER_CAR_S*F_CPU/TIMER_CAR_PRESCALAR)

    #define WHEEL_CIRCUMFERENCE 2.37*2.54*PI/100
    #define WHEELS_DISTANCE 0.2
    #define CURVE_RADIUS 1.173

    typedef struct {
        double x, y, theta;
    } pose_t;

    class DifferentialCar {
        public:
            static uint32_t last_update;
            static JohnsonMotor *left_motor, *right_motor;
            static pose_t car_pose;
            static uint8_t target_container;

            static void update_position(double delta);
            static void update_speed(double delta);
            static void set_speed(double linear_speed, double angular_speed);
            static void set_motors_speed(double left_speed, double right_speed);
            static double get_euclidean_distance_to_container(pose_t *current, const container_position_t *target);

            uint8_t current_container = 0;  

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

            void set_target_container(uint8_t container);

        private:
            static void left_motor_isr(void);
            static void right_motor_isr(void);
    };

    ISR(TIMER1_OVF_vect);
#endif