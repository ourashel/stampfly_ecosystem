/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.cpp
 * @brief See app_controller.hpp for what this class is for.
 *        このクラスの目的は app_controller.hpp を参照。
 *
 * @design controller.hpp — IController interface (12 methods)   [OK]
 * @design docs/plans/smc-rate-loop-plan.md §7.11 — STA trial for motor-delay [--]
 */

#include "app_controller.hpp"
#include "params.hpp"

namespace sf::app {

// X-quad spec inertia Ixx/Iyy/Izz [kg*m^2] -- same value and same caveat as
// firmware/apps/smc_rate/app_controller.cpp's kInertia.
// X-quad仕様慣性 Ixx/Iyy/Izz [kg*m^2] -- firmware/apps/smc_rate/
// app_controller.cppのkInertiaと同じ値・同じ注意点。
static const float kInertia[3] = {9.16e-6f, 13.3e-6f, 20.4e-6f};  // roll, pitch, yaw

void AppController::init()
{
    pid_.init();
    sta_roll_.inertia  = kInertia[0];
    sta_pitch_.inertia = kInertia[1];
    sta_yaw_.inertia   = kInertia[2];
    loadSmcParams();
}

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    // Run the full PID cascade unchanged -- see firmware/apps/smc_rate/
    // app_controller.cpp's compute() for the full rationale (identical
    // here, only the rate law differs).
    // PIDカスケードを丸ごと実行する（無改造）-- 詳細な根拠は
    // firmware/apps/smc_rate/app_controller.cppのcompute()参照（ここでは
    // レート則だけが異なる）。
    sf::ControlOutput output = pid_.compute(state, setpoint, dt);

    output.torque[0] = sta_roll_.compute(output.rate_ref[0], state.angular_rate[0], dt);
    output.torque[1] = sta_pitch_.compute(output.rate_ref[1], state.angular_rate[1], dt);
    output.torque[2] = sta_yaw_.compute(output.rate_ref[2], state.angular_rate[2], dt);

    return output;
}

void AppController::loadSmcParams()
{
    sf::params::get_float("smc_sta.roll.k1",        sta_roll_.k1);
    sf::params::get_float("smc_sta.roll.k2",        sta_roll_.k2);
    sf::params::get_float("smc_sta.roll.phi",       sta_roll_.phi);
    sf::params::get_float("smc_sta.roll.lambda_i",  sta_roll_.lambda_i);
    sf::params::get_float("smc_sta.roll.e_reset",   sta_roll_.e_reset);
    sf::params::get_float("smc_sta.pitch.k1",       sta_pitch_.k1);
    sf::params::get_float("smc_sta.pitch.k2",       sta_pitch_.k2);
    sf::params::get_float("smc_sta.pitch.phi",      sta_pitch_.phi);
    sf::params::get_float("smc_sta.pitch.lambda_i", sta_pitch_.lambda_i);
    sf::params::get_float("smc_sta.pitch.e_reset",  sta_pitch_.e_reset);
    sf::params::get_float("smc_sta.yaw.k1",         sta_yaw_.k1);
    sf::params::get_float("smc_sta.yaw.k2",         sta_yaw_.k2);
    sf::params::get_float("smc_sta.yaw.phi",        sta_yaw_.phi);
    sf::params::get_float("smc_sta.yaw.lambda_i",   sta_yaw_.lambda_i);
    sf::params::get_float("smc_sta.yaw.e_reset",    sta_yaw_.e_reset);

    // Same physical torque ceiling the PID rate loop uses -- see
    // firmware/apps/smc_rate/app_controller.cpp's loadSmcParams() for the
    // provenance comment (unchanged here).
    // レートPIDと同じ物理トルク上限 -- 出典コメントは
    // firmware/apps/smc_rate/app_controller.cppのloadSmcParams()参照
    // （ここでは無変更）。
    constexpr float kMaxRollPitchTorque = 5.2e-3f;  // [Nm], mirrors pid_controller.hpp
    sta_roll_.output_limit  = kMaxRollPitchTorque;
    sta_pitch_.output_limit = kMaxRollPitchTorque;
    sf::params::get_float("rate.yaw.max_torque", sta_yaw_.output_limit);
}

void AppController::reset()
{
    pid_.reset();
    sta_roll_.reset();
    sta_pitch_.reset();
    sta_yaw_.reset();
}

void AppController::onModeChange(sf::FlightMode new_mode)
{
    pid_.onModeChange(new_mode);
}

void AppController::onLanding()
{
    pid_.onLanding();
}

void AppController::onTakeoff()
{
    pid_.onTakeoff();
}

void AppController::onTakeoffComplete()
{
    pid_.onTakeoffComplete();
}

bool AppController::isTakeoffComplete() const
{
    return pid_.isTakeoffComplete();
}

void AppController::setGuidanceTarget(const sf::GuidanceTarget& target,
                                       const sf::CommandSetpoint& current_sticks)
{
    pid_.setGuidanceTarget(target, current_sticks);
}

bool AppController::isGuidanceActive() const
{
    return pid_.isGuidanceActive();
}

void AppController::startExcitation(const sf::SysidCommand& cmd)
{
    // Forwarded unchanged -- see firmware/apps/smc_rate/app_controller.cpp's
    // startExcitation() for the full rationale.
    // 無改造で転送 -- 詳細な根拠はfirmware/apps/smc_rate/app_controller.cpp
    // のstartExcitation()参照。
    pid_.startExcitation(cmd);
}

bool AppController::fetchSysidResult(sf::SysidFreqResult& out)
{
    return pid_.fetchSysidResult(out);
}

void AppController::reloadParams()
{
    pid_.reloadParams();
    loadSmcParams();
}

}  // namespace sf::app
