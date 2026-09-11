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
#include "esp_log.h"

namespace sf::app {

// TEMPORARY diagnostic instrumentation (docs/plans/smc-rate-loop-plan.md
// §7.17): log the roll axis's hyperplane integral (lambda_i*integral term)
// and the STA's own integral z every kDiagPeriodCycles, to measure the
// REAL saturation timescale under a sustained disturbance rather than
// relying on the theoretical estimate alone. Remove after the measurement.
// 一時的な診断用計装（docs/plans/smc-rate-loop-plan.md §7.17）: roll軸の
// 超平面積分（lambda_i*integral項）とSTA自身の積分zをkDiagPeriodCyclesごとに
// ログ出力し、理論見積もりだけに頼らず持続外乱下での実際の飽和時間を測る。
// 測定後に削除する。
static const char* kDiagTag = "STA_DIAG";
static constexpr int kDiagPeriodCycles = 100;  // 0.25 s @ 400 Hz
static int diag_counter_ = 0;

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

    // TEMPORARY diagnostic -- see the file-header comment above.
    // 一時的な診断 -- ファイル冒頭のコメント参照。
    if (++diag_counter_ >= kDiagPeriodCycles) {
        diag_counter_ = 0;
        const float e = output.rate_ref[0] - state.angular_rate[0];
        const float s = e + sta_roll_.lambda_i * sta_roll_.integral;
        ESP_LOGI(kDiagTag, "roll e=%.4f integ=%.4f z=%.4f s=%.4f torque=%.6f",
                 e, sta_roll_.integral, sta_roll_.z, s, output.torque[0]);
    }

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

    // z leaky-integration safety mechanism -- see smc_rate_sta.hpp's
    // z_leak_tau field comment and docs/plans/smc-rate-loop-plan.md §7.18.
    // zの漏れ積分安全機構 -- smc_rate_sta.hppのz_leak_tauフィールドコメントと
    // docs/plans/smc-rate-loop-plan.md §7.18参照。
    sf::params::get_float("smc_sta.roll.z_leak_tau",  sta_roll_.z_leak_tau);
    sf::params::get_float("smc_sta.pitch.z_leak_tau", sta_pitch_.z_leak_tau);
    sf::params::get_float("smc_sta.yaw.z_leak_tau",   sta_yaw_.z_leak_tau);

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
