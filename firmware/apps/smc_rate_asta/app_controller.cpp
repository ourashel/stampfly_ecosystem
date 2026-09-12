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
 * @design docs/plans/smc-rate-loop-plan.md §7.31 — adaptive STA trial [--]
 */

#include "app_controller.hpp"
#include "params.hpp"

namespace sf::app {

// X-quad spec inertia Ixx/Iyy/Izz [kg*m^2] -- same value and same caveat as
// firmware/apps/smc_rate_sta/app_controller.cpp's kInertia.
// X-quad仕様慣性 Ixx/Iyy/Izz [kg*m^2] -- firmware/apps/smc_rate_sta/
// app_controller.cppのkInertiaと同じ値・同じ注意点。
static const float kInertia[3] = {9.16e-6f, 13.3e-6f, 20.4e-6f};  // roll, pitch, yaw

void AppController::init()
{
    pid_.init();
    sta_roll_.inertia  = kInertia[0];
    sta_pitch_.inertia = kInertia[1];
    sta_yaw_.inertia   = kInertia[2];
    loadSmcParams();
    // reset() seeds k1 from k1_init -- must run AFTER loadSmcParams() so
    // k1_init is the freshly-loaded value, not the struct's compile-time
    // default.
    // reset()はk1_initからk1をシードする——loadSmcParams()の後に実行し、
    // k1_initが構造体のコンパイル時既定値でなく読み込み済みの値になるようにする。
    sta_roll_.reset();
    sta_pitch_.reset();
    sta_yaw_.reset();
}

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    // Run the full PID cascade unchanged -- see firmware/apps/smc_rate_sta/
    // app_controller.cpp's compute() for the full rationale (identical
    // here, only the rate law differs).
    // PIDカスケードを丸ごと実行する（無改造）-- 詳細な根拠は
    // firmware/apps/smc_rate_sta/app_controller.cppのcompute()参照
    // （ここではレート則だけが異なる）。
    sf::ControlOutput output = pid_.compute(state, setpoint, dt);

    output.torque[0] = sta_roll_.compute(output.rate_ref[0], state.angular_rate[0], dt);
    output.torque[1] = sta_pitch_.compute(output.rate_ref[1], state.angular_rate[1], dt);
    output.torque[2] = sta_yaw_.compute(output.rate_ref[2], state.angular_rate[2], dt);

    return output;
}

void AppController::loadSmcParams()
{
    sf::params::get_float("smc_asta.roll.k1_init",    sta_roll_.k1_init);
    sf::params::get_float("smc_asta.roll.k1_min",     sta_roll_.k1_min);
    sf::params::get_float("smc_asta.roll.k1_max",     sta_roll_.k1_max);
    sf::params::get_float("smc_asta.roll.k2_ratio",   sta_roll_.k2_ratio);
    sf::params::get_float("smc_asta.roll.adapt_rate", sta_roll_.adapt_rate);
    sf::params::get_float("smc_asta.roll.leak_ratio", sta_roll_.leak_ratio);
    sf::params::get_float("smc_asta.roll.dead_band",  sta_roll_.dead_band);
    sf::params::get_float("smc_asta.roll.filter_tau", sta_roll_.filter_tau);
    sf::params::get_float("smc_asta.roll.mref_tau",          sta_roll_.mref_tau);
    sf::params::get_float("smc_asta.roll.mref_fast_tau",     sta_roll_.mref_fast_tau);
    sf::params::get_float("smc_asta.roll.mref_slow_tau",     sta_roll_.mref_slow_tau);
    sf::params::get_float("smc_asta.roll.mref_growth_ratio", sta_roll_.mref_growth_ratio);
    sf::params::get_float("smc_asta.roll.mref_abs_floor",    sta_roll_.mref_abs_floor);
    sf::params::get_float("smc_asta.roll.mref_shrink_ratio", sta_roll_.mref_shrink_ratio);
    sf::params::get_float("smc_asta.roll.mref_dwell_time",   sta_roll_.mref_dwell_time);
    sf::params::get_float("smc_asta.roll.phi",        sta_roll_.phi);
    sf::params::get_float("smc_asta.roll.lambda_i",   sta_roll_.lambda_i);
    sf::params::get_float("smc_asta.roll.e_reset",    sta_roll_.e_reset);

    sf::params::get_float("smc_asta.pitch.k1_init",    sta_pitch_.k1_init);
    sf::params::get_float("smc_asta.pitch.k1_min",     sta_pitch_.k1_min);
    sf::params::get_float("smc_asta.pitch.k1_max",     sta_pitch_.k1_max);
    sf::params::get_float("smc_asta.pitch.k2_ratio",   sta_pitch_.k2_ratio);
    sf::params::get_float("smc_asta.pitch.adapt_rate", sta_pitch_.adapt_rate);
    sf::params::get_float("smc_asta.pitch.leak_ratio", sta_pitch_.leak_ratio);
    sf::params::get_float("smc_asta.pitch.dead_band",  sta_pitch_.dead_band);
    sf::params::get_float("smc_asta.pitch.filter_tau", sta_pitch_.filter_tau);
    sf::params::get_float("smc_asta.pitch.mref_tau",          sta_pitch_.mref_tau);
    sf::params::get_float("smc_asta.pitch.mref_fast_tau",     sta_pitch_.mref_fast_tau);
    sf::params::get_float("smc_asta.pitch.mref_slow_tau",     sta_pitch_.mref_slow_tau);
    sf::params::get_float("smc_asta.pitch.mref_growth_ratio", sta_pitch_.mref_growth_ratio);
    sf::params::get_float("smc_asta.pitch.mref_abs_floor",    sta_pitch_.mref_abs_floor);
    sf::params::get_float("smc_asta.pitch.mref_shrink_ratio", sta_pitch_.mref_shrink_ratio);
    sf::params::get_float("smc_asta.pitch.mref_dwell_time",   sta_pitch_.mref_dwell_time);
    sf::params::get_float("smc_asta.pitch.phi",        sta_pitch_.phi);
    sf::params::get_float("smc_asta.pitch.lambda_i",   sta_pitch_.lambda_i);
    sf::params::get_float("smc_asta.pitch.e_reset",    sta_pitch_.e_reset);

    sf::params::get_float("smc_asta.yaw.k1_init",    sta_yaw_.k1_init);
    sf::params::get_float("smc_asta.yaw.k1_min",     sta_yaw_.k1_min);
    sf::params::get_float("smc_asta.yaw.k1_max",     sta_yaw_.k1_max);
    sf::params::get_float("smc_asta.yaw.k2_ratio",   sta_yaw_.k2_ratio);
    sf::params::get_float("smc_asta.yaw.adapt_rate", sta_yaw_.adapt_rate);
    sf::params::get_float("smc_asta.yaw.leak_ratio", sta_yaw_.leak_ratio);
    sf::params::get_float("smc_asta.yaw.dead_band",  sta_yaw_.dead_band);
    sf::params::get_float("smc_asta.yaw.filter_tau", sta_yaw_.filter_tau);
    sf::params::get_float("smc_asta.yaw.mref_tau",          sta_yaw_.mref_tau);
    sf::params::get_float("smc_asta.yaw.mref_fast_tau",     sta_yaw_.mref_fast_tau);
    sf::params::get_float("smc_asta.yaw.mref_slow_tau",     sta_yaw_.mref_slow_tau);
    sf::params::get_float("smc_asta.yaw.mref_growth_ratio", sta_yaw_.mref_growth_ratio);
    sf::params::get_float("smc_asta.yaw.mref_abs_floor",    sta_yaw_.mref_abs_floor);
    sf::params::get_float("smc_asta.yaw.mref_shrink_ratio", sta_yaw_.mref_shrink_ratio);
    sf::params::get_float("smc_asta.yaw.mref_dwell_time",   sta_yaw_.mref_dwell_time);
    sf::params::get_float("smc_asta.yaw.phi",        sta_yaw_.phi);
    sf::params::get_float("smc_asta.yaw.lambda_i",   sta_yaw_.lambda_i);
    sf::params::get_float("smc_asta.yaw.e_reset",    sta_yaw_.e_reset);

    // z leaky-integration safety mechanism -- see smc_rate_asta.hpp's
    // z_leak_tau field comment (same mechanism as smc_rate_sta.hpp).
    // zの漏れ積分安全機構 -- smc_rate_asta.hppのz_leak_tauフィールドコメント
    // 参照（smc_rate_sta.hppと同一機構）。
    sf::params::get_float("smc_asta.roll.z_leak_tau",  sta_roll_.z_leak_tau);
    sf::params::get_float("smc_asta.pitch.z_leak_tau", sta_pitch_.z_leak_tau);
    sf::params::get_float("smc_asta.yaw.z_leak_tau",   sta_yaw_.z_leak_tau);

    // Same physical torque ceiling the PID rate loop and smc_rate_sta use --
    // see firmware/apps/smc_rate/app_controller.cpp's loadSmcParams() for
    // the provenance comment (unchanged here).
    // レートPID・smc_rate_staと同じ物理トルク上限 -- 出典コメントは
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
