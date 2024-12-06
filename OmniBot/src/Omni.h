#ifndef RIP_OMNI_H
#define RIP_OMNI_H

#include <chrono>
#include <functional>
#include <cmath>

#include <spdlog/spdlog.h>
#include <riplib/json.hpp>

#include <riplib/Motor.h>
#include <riplib/Gyro.h>
#include <riplib/Sensor.h>

namespace rip {
    class Omni {
    public:
        static Omni &get();

        Omni &operator=(const Omni &) = delete;
        //
        // void calibrate_light_sensors();

        /** Call reset for the default Gyro and the motor encoders */
        void reset_hardware();

        // [[nodiscard]] int get_left_sensor();
        //
        // [[nodiscard]] int get_right_sensor();
        //
        // [[nodiscard]] int get_middle_sensor();
        //
        // [[nodiscard]] bool is_black(int sensor_value) const;
        //
        // void drive(double distance_cm,
        //            double offset_cm = 0,
        //            double angle = 0,
        //            double target_speed = 1,
        //            bool accelerate = true,
        //            bool decelerate = true);

        void drive_raw(double distance_cm, double angle = 0, double speed = 1);

        // void drive_until(bool forwards,
        //                  const std::function<bool()> &condition,
        //                  double target_speed = 1,
        //                  bool accelerate = true);

        void turn(double angle,
                  double target_speed = 1,
                  bool accelerate = true,
                  bool decelerate = true);

        // void turn_until(bool clockwise,
        //                 const std::function<bool()> &condition,
        //                 double target_speed = 1,
        //                 bool accelerate = true);
        //
        // void follow_line(bool right_sensor,
        //                  bool right_edge,
        //                  double distance_cm,
        //                  double target_speed = 1,
        //                  bool accelerate = true,
        //                  bool decelerate = true);
        //
        // void follow_line_until(bool right_sensor,
        //                        bool right_edge,
        //                        const std::function<bool()> &condition,
        //                        double target_speed = 1,
        //                        bool accelerate = true);
        //
        // void align_at_next_line(bool forwards = true, double speed = 0.5);
        //
        void freeze(int time_ms = 0);

        void set_speed(double l_speed, double r_speed, double t_speed);

        void drive_straight(double distance_cm,
                    double angle,
                    double target_speed = 0.9,
                    bool accelerate = true,
                    bool decelerate = true);
        void drive_with_turn(double distance_cm,
                           double drive_angle,
                           double turn_angle,
                           double target_speed = 0.9,
                           bool accelerate = true,
                           bool decelerate = true);

    private:
        std::shared_ptr<spdlog::logger> log_;
        Gyro &gyro_ = Gyro::get();
        Motor *left_motor_;
        Motor *right_motor_;
        Motor *top_motor_;
        Sensor *left_sensor_;
        Sensor *right_sensor_;
        Sensor *top_sensor_;
        double start_speed_;
        double end_speed_;
        int encoder_accel_ticks_;
        int encoder_decel_ticks_;
        double encoder_correction_factor_;
        double gyro_accel_deg_;
        double gyro_decel_deg_;
        double gyro_correction_factor_;
        double gyro_curve_p_;
        double gyro_curve_i_;
        double gyro_curve_d_;
        double line_follow_correction_factor_;
        double drive_overshoot_ticks_;
        double turn_overshoot_degrees_;
        double ticks_per_cm_;
        double align_reverse_factor_;
        int white_value_;
        int black_value_;

        Omni();

        ~Omni();



        [[nodiscard]] int get_ticks_from_distance(double centimeters) const;

        [[nodiscard]] double clamp_speed(double target_speed) const;
    };
}

#endif
