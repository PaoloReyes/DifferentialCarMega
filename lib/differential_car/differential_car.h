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
            static uint32_t last_update, new_profile_time;
            static JohnsonMotor *left_motor, *right_motor;
            static pose_t car_pose;
            static double t1, t2, t3, vtop, b, c, d, target_distance;
            static uint16_t container_target;
            static bool on_target, vmax_reached;
            static double last_position_error;

            static void update_position(double delta);
            static void update_speed(double delta);
            static void set_speed(double linear_speed, double angular_speed);
            static void set_motors_speed(double left_speed, double right_speed);
            static double get_euclidean_distance_to_container(pose_t *current, const container_position_t *target);
            static double vel(double t);
            static double pos(double t);

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

            static void set_target_container(uint8_t container);
            static void generate_profile_parameters(void);

        private:
            static void left_motor_isr(void);
            static void right_motor_isr(void);
    };

    ISR(TIMER1_OVF_vect);
#endif