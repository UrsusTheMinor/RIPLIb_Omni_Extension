//
// Created by reini on 06.09.24.
//

#include <cmath>

#include <spdlog/spdlog.h>
#include <wallaby/wombat.h>

#include <riplib/Util.h>
#include <riplib/RIP.h>
#include <riplib/PID.h>
#include "Omni.h"
#include <iostream>

namespace rip {
    Omni &Omni::get() {
        static Omni instance;
        return instance;
    }

    Omni::Omni() {
        log_ = create_logger("Omni");
        log_->trace("Initialising from config");

        left_motor_ = new Motor("omni/left_motor");
        right_motor_ = new Motor("omni/right_motor");
        top_motor_ = new Motor("omni/top_motor");

        try {
            left_sensor_ = new Sensor("omni/left_sensor");
        } catch (std::invalid_argument &e) {
            log_->error("No left sensor specified, some methods will not work");
            left_sensor_ = nullptr;
        }

        try {
            right_sensor_ = new Sensor("omni/right_sensor");
        } catch (std::invalid_argument &e) {
            log_->error("No right sensor specified, some methods will not work");
            right_sensor_ = nullptr;
        }

        try {
            top_sensor_ = new Sensor("omni/top_sensor");
        } catch (std::exception &e) {
            log_->error("No top sensor specified, some methods will not work");
            top_sensor_ = nullptr;
        }

        auto json_object = RIP::get_config("omni");

        try {
            start_speed_ = json_object.at("start_speed").get<double>();
            end_speed_ = json_object.at("end_speed").get<double>();
            encoder_correction_factor_ = json_object.at("encoder_correction_factor").get<double>();
            gyro_accel_deg_ = json_object.at("gyro_accel_deg").get<double>();
            gyro_decel_deg_ = json_object.at("gyro_decel_deg").get<double>();
            gyro_correction_factor_ = json_object.at("gyro_correction_factor").get<double>();
            gyro_curve_p_ = json_object.at("gyro_curve_p").get<double>();
            gyro_curve_i_ = json_object.at("gyro_curve_i").get<double>();
            gyro_curve_d_ = json_object.at("gyro_curve_d").get<double>();
            line_follow_correction_factor_ = json_object.at("line_follow_correction_factor").get<double>();
            turn_overshoot_degrees_ = json_object.at("turn_overshoot_degrees").get<double>();
            drive_overshoot_ticks_ = json_object.at("drive_overshoot_ticks").get<double>();
            ticks_per_cm_ = json_object.at("ticks_per_cm").get<double>();
            align_reverse_factor_ = json_object.at("align_reverse_factor").get<double>();
            white_value_ = json_object.at("white_value").get<int>();
            black_value_ = json_object.at("black_value").get<int>();

            // depends on ticks_per_cm_ and cm_to_ticks_d_
            encoder_accel_ticks_ = get_ticks_from_distance(json_object.at("encoder_accel_cm").get<int>());
            encoder_decel_ticks_ = get_ticks_from_distance(json_object.at("encoder_decel_cm").get<int>());
        } catch (nlohmann::json::exception &e) {
            throw std::invalid_argument(fmt::format("Invalid data for Omni in the config: {}", e.what()));
        }

        reset_hardware();
    }

    Omni::~Omni() {
        delete left_motor_;
        delete right_motor_;
        delete top_motor_;
    }

    void Omni::drive_raw(double distance_cm, double angle, double speed) {
        if (!RIP::is_running()) return;

        speed = clamp_speed(speed);
        log_->info("Driving {}cm at {:0.1f}% speed without correction at angle {}", distance_cm, speed * 100, angle);

        reset_hardware();

        int goal_ticks = get_ticks_from_distance(distance_cm);

        log_->info("Goal Ticks: {}", goal_ticks);


        if (goal_ticks < 0) {
            speed *= -1;
        }

        double vx = sin(angle * M_PI / 180);
        double vy = cos(angle * M_PI / 180);

        double lo = 14;

        double turn = 0;

        double v_t = -0.5 * vx + (sqrt(3) / 2) * vy + lo * turn;
        double v_l = -0.5 * vx - (sqrt(3) / 2) * vy + lo * turn;
        double v_r = vx + lo * turn;

        int current_ticks = 0;
        while (
            RIP::is_running() &&
            std::abs(current_ticks) + drive_overshoot_ticks_ < std::abs(goal_ticks)
            // check if current ticks match the required ticks
        ) {
            current_ticks = (int) ((
                                       abs(left_motor_->get_encoder()) +
                                       abs(right_motor_->get_encoder()) +
                                       abs(top_motor_->get_encoder())
                                   ) * 0.5); // calculate the current ticks

            set_speed(v_l, v_r, v_t);
        }

        freeze();
        double left = left_motor_->get_encoder();
        double right = right_motor_->get_encoder();
        double top = top_motor_->get_encoder();

        log_->debug("Distances after drive_raw: L: {} | R: {} | T: {} | AVG: {:0.1f}",
                    left, right, top, (left + right + top) / 3.0);
    }

    void Omni::set_speed(double l_speed, double r_speed, double t_speed) {
        left_motor_->set_speed(l_speed);
        right_motor_->set_speed(r_speed);
        top_motor_->set_speed(t_speed);
    }

    void Omni::freeze(int time_ms) {
        left_motor_->freeze();
        right_motor_->freeze();
        top_motor_->freeze();
        sleep(time_ms); // ensure breaking
    }

    void Omni::reset_hardware() {
        left_motor_->reset_encoder();
        right_motor_->reset_encoder();
        top_motor_->reset_encoder();
        gyro_.reset();
    }

    int Omni::get_ticks_from_distance(double centimeters) const {
        int ticks = static_cast<int>(centimeters * ticks_per_cm_);

        log_->debug("Conversion: {} cm are {} ticks.", centimeters, ticks);
        return ticks;
    }

    double Omni::clamp_speed(double target_speed) const {
        double min_speed = std::min(start_speed_, end_speed_);
        if (target_speed < min_speed) {
            log_->error("Cannot drive with {:0.1f}% because it is less than the start_speed and end_speed. Clamping!",
                        target_speed * 100);
            target_speed = min_speed;
        }
        return target_speed;
    }

    void Omni::drive_straight(double distance_cm,
                              double angle,
                              double target_speed,
                              bool accelerate,
                              bool decelerate) {
        if (!RIP::is_running()) return;
        target_speed = clamp_speed(target_speed);
        log_->info("Driving {}cm at {:0.1f}% speed", distance_cm, target_speed * 100);

        int goal_ticks = get_ticks_from_distance(distance_cm);

        reset_hardware();

        double vx = sin(angle * M_PI / 180);
        double vy = cos(angle * M_PI / 180);

        double lo = 14;

        double turn = 0;

        double v_t = -0.5 * vx + (sqrt(3) / 2) * vy + lo * turn;
        double v_l = -0.5 * vx - (sqrt(3) / 2) * vy + lo * turn;
        double v_r = vx + lo * turn;

        double current_ticks = 0;
        while (RIP::is_running() && std::abs(current_ticks) + drive_overshoot_ticks_ < std::abs(goal_ticks)) {
            sleep(2);


            double speed = get_goal_speed(
                target_speed,
                start_speed_,
                end_speed_,
                current_ticks,
                goal_ticks,
                encoder_accel_ticks_,
                encoder_decel_ticks_,
                accelerate,
                decelerate
            );

            double gyro_correction = gyro_.read() * gyro_correction_factor_ * std::abs(speed);

            if (goal_ticks < 0) {
                speed *= -1;
            }

            set_speed(v_l - gyro_correction, v_r + gyro_correction, v_t - gyro_correction);

            current_ticks = (int) ((
                                       abs(left_motor_->get_encoder()) +
                                       abs(right_motor_->get_encoder()) +
                                       abs(top_motor_->get_encoder())
                                   ) * 0.5); // calculate the current ticks
        }

        freeze();
        double left = left_motor_->get_encoder();
        double right = right_motor_->get_encoder();
        double top = top_motor_->get_encoder();

        log_->debug("Distances after drive_straight: L: {} | R: {} | T: {} | AVG: {:0.1f}",
                    left, right, top, (left + right + top) / 3.0);
    }

    void Omni::turn(double angle, double target_speed, bool accelerate, bool decelerate) {
        if (!RIP::is_running()) return;
        target_speed = clamp_speed(target_speed);
        log_->info("Turning {:0.2f}° at {:0.1f}% speed", angle, target_speed * 100);
        if (std::abs(angle) < 10) {
            log_->warn("Low accuracy for small angle {:0.2f}°", angle);
        }

        reset_hardware();

        double current_angle = 0;
        while (RIP::is_running() && std::abs(current_angle) + turn_overshoot_degrees_ < std::abs(angle)) {
            sleep(2);

            double speed = get_goal_speed(
                target_speed,
                start_speed_,
                end_speed_,
                current_angle,
                angle,
                gyro_accel_deg_,
                gyro_decel_deg_,
                accelerate,
                decelerate
            );

            if (angle < 0) {
                speed *= -1;
            }

            double encoder_diff = (-abs(left_motor_->get_encoder()) + abs(right_motor_->get_encoder()) + -abs(
                                       top_motor_->get_encoder()))
                                  * encoder_correction_factor_;

            set_speed(speed - encoder_diff, speed - encoder_diff, speed - encoder_diff);
            current_angle = gyro_.read();
        }

        freeze();
    }

    void Omni::drive_with_turn(double distance_cm,
                               double drive_angle,
                               double target_speed,
                               double turn_angle_per_second,
                               bool accelerate,
                               bool decelerate) {

        // Not Implemented Exception
        throw std::runtime_error("Omni::drive_with_turn: not implemented");

        if (!RIP::is_running()) return;
        target_speed = clamp_speed(target_speed);
        log_->info("Driving {}cm at {:0.1f}% speed", distance_cm, target_speed * 100);

        int goal_ticks = get_ticks_from_distance(distance_cm);

        reset_hardware();

        double vx = sin(drive_angle * M_PI / 180);
        double vy = cos(drive_angle * M_PI / 180);

        double lo = 14;

        double turn = 0;

        double v_t = -0.5 * vx + (sqrt(3) / 2) * vy + lo * turn;
        double v_l = -0.5 * vx - (sqrt(3) / 2) * vy + lo * turn;
        double v_r = vx + lo * turn;

        double current_ticks = 0;
        while (RIP::is_running() && std::abs(current_ticks) + drive_overshoot_ticks_ < std::abs(goal_ticks)) {
            sleep(2);


            double speed = get_goal_speed(
                target_speed,
                start_speed_,
                end_speed_,
                current_ticks,
                goal_ticks,
                encoder_accel_ticks_,
                encoder_decel_ticks_,
                accelerate,
                decelerate
            );

            double gyro_correction = gyro_.read() * gyro_correction_factor_ * std::abs(speed);

            if (goal_ticks < 0) {
                speed *= -1;
            }

            set_speed(v_l - gyro_correction, v_r + gyro_correction, v_t - gyro_correction);

            current_ticks = (int) ((
                                       abs(left_motor_->get_encoder()) +
                                       abs(right_motor_->get_encoder()) +
                                       abs(top_motor_->get_encoder())
                                   ) * 0.5); // calculate the current ticks
        }

        freeze();
        double left = left_motor_->get_encoder();
        double right = right_motor_->get_encoder();
        double top = top_motor_->get_encoder();

        log_->debug("Distances after drive_straight: L: {} | R: {} | T: {} | AVG: {:0.1f}",
                    left, right, top, (left + right + top) / 3.0);
    }
}
