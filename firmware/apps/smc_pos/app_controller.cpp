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
 * @design docs/plans/smc-rate-loop-plan.md §7 — rate+velocity-loop SMC design [--]
 */

#include "app_controller.hpp"
#include "params.hpp"
#include "sf_math.hpp"

namespace sf::app {

// X-quad spec inertia Ixx/Iyy/Izz [kg*m^2] -- same value and same caveat as
// firmware/apps/smc_rate/app_controller.cpp's kInertia (no inertia SSOT
// header exists to alias). Used to scale the rate-loop reaching law.
// X-quad仕様慣性 Ixx/Iyy/Izz [kg*m^2] -- firmware/apps/smc_rate/
// app_controller.cppのkInertiaと同じ値・同じ注意点。レートループ到達則の
// スケールに使う。
static const float kInertia[3] = {9.16e-6f, 13.3e-6f, 20.4e-6f};  // roll, pitch, yaw

// POS_HOLD tilt limit [rad] -- same value as PidController's private
// max_pos_tilt_ (pid_controller.hpp), duplicated here for the same reason
// kInertia is: PidController does not expose it as a param, and this app
// needs it (with kGravity) to set the velocity-loop SMC's output_limit to
// the SAME physical ceiling vel_x_/vel_y_ used before being overridden.
// POS_HOLDの傾き上限 [rad] -- PidControllerの非公開max_pos_tilt_
// （pid_controller.hpp）と同じ値。kInertiaと同じ理由でここに複製する:
// PidControllerがparam化しておらず、本appが差し替え前のvel_x_/vel_y_と
// 同じ物理上限をSMC速度ループのoutput_limitに設定するのに（kGravityと共に）
// 必要なため。
static constexpr float kMaxPosTilt = 0.1745f;  // [rad] (10 deg)

void AppController::init()
{
    pid_.init();
    smc_roll_.inertia  = kInertia[0];
    smc_pitch_.inertia = kInertia[1];
    smc_yaw_.inertia   = kInertia[2];
    loadRateSmcParams();
    loadVelSmcParams();

    // Plug the velocity-loop SMC into pid_'s position cascade (see
    // pid_controller.hpp PidController::setVelocityLawOverride()). pid_'s
    // OWN position-loop stage (pos_x_/pos_y_) stays untouched -- only the
    // vx/vy -> ax/ay stage is replaced, for BOTH the stick-reposition and
    // hold paths (they share this one stage, pid_controller.cpp:1300-1333).
    // pid_の位置カスケードへ速度SMCを注入する（PidController::
    // setVelocityLawOverride()参照）。pid_自身の位置ループ段（pos_x_/pos_y_）
    // は無改造 -- vx/vy→ax/ay段だけを差し替える。スティック再配置・保持の
    // 両経路がこの1段を共有する（pid_controller.cpp:1300-1333）。
    pid_.setVelocityLawOverride(
        [this](float vx_sp, float vx, float vy_sp, float vy, float dt,
               float& ax_ned, float& ay_ned) {
            ax_ned = smc_vel_x_.compute(vx_sp, vx, dt);
            ay_ned = smc_vel_y_.compute(vy_sp, vy, dt);
        });
}

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    // Run the full cascade: thrust, altitude, POSITION loop (pos_x_/pos_y_,
    // unchanged), takeoff/landing phase state machine, guidance, trim/hover-
    // thrust learning, DOB, and heading hold are all computed exactly as the
    // default vehicle build would. The velocity loop INSIDE this call
    // already routed through smc_vel_x_/smc_vel_y_ via the override
    // registered in init() -- see docs/plans/smc-rate-loop-plan.md §7. We
    // only discard torque[0..2] below (same as smc_rate).
    // カスケードを丸ごと実行する: 推力・高度・位置ループ（pos_x_/pos_y_、
    // 無改造）・離着陸フェーズステートマシン・誘導・トリム/ホバー推力学習・
    // DOB・ヘディングホールドは全て既定vehicleビルドと全く同じように計算
    // される。この呼び出し内部の速度ループは、init()で登録した差し替え口
    // 経由で既にsmc_vel_x_/smc_vel_y_を通っている
    // （docs/plans/smc-rate-loop-plan.md §7参照）。以下で破棄するのは
    // torque[0..2]だけ（smc_rateと同じ）。
    sf::ControlOutput output = pid_.compute(state, setpoint, dt);

    // output.rate_ref[R,P,Y] is PidController's OWN final rate-loop target,
    // already reflecting whatever roll/pitch tilt the (possibly SMC-driven)
    // velocity loop commanded this cycle -- see firmware/apps/smc_rate's
    // app_controller.cpp for the full rationale (unchanged here).
    // output.rate_ref[R,P,Y]はPidController自身の最終レートループ目標で、
    // （SMC駆動かもしれない）速度ループが今サイクル指令したroll/pitch傾きを
    // 既に反映済み -- 詳細な根拠はfirmware/apps/smc_rateのapp_controller.cpp
    // 参照（ここでは無変更）。
    output.torque[0] = smc_roll_.compute(output.rate_ref[0], state.angular_rate[0], dt);
    output.torque[1] = smc_pitch_.compute(output.rate_ref[1], state.angular_rate[1], dt);
    output.torque[2] = smc_yaw_.compute(output.rate_ref[2], state.angular_rate[2], dt);

    return output;
}

void AppController::loadRateSmcParams()
{
    sf::params::get_float("smc.roll.k",        smc_roll_.k);
    sf::params::get_float("smc.roll.eta",      smc_roll_.eta);
    sf::params::get_float("smc.roll.phi",      smc_roll_.phi);
    sf::params::get_float("smc.roll.lambda_i", smc_roll_.lambda_i);
    sf::params::get_float("smc.roll.e_reset",  smc_roll_.e_reset);
    sf::params::get_float("smc.pitch.k",        smc_pitch_.k);
    sf::params::get_float("smc.pitch.eta",      smc_pitch_.eta);
    sf::params::get_float("smc.pitch.phi",      smc_pitch_.phi);
    sf::params::get_float("smc.pitch.lambda_i", smc_pitch_.lambda_i);
    sf::params::get_float("smc.pitch.e_reset",  smc_pitch_.e_reset);
    sf::params::get_float("smc.yaw.k",        smc_yaw_.k);
    sf::params::get_float("smc.yaw.eta",      smc_yaw_.eta);
    sf::params::get_float("smc.yaw.phi",      smc_yaw_.phi);
    sf::params::get_float("smc.yaw.lambda_i", smc_yaw_.lambda_i);
    sf::params::get_float("smc.yaw.e_reset",  smc_yaw_.e_reset);

    // Dead-time predictor -- params are [ms] for readability, SlidingModeRate
    // wants [s]. See firmware/apps/smc_rate/app_controller.cpp (same pattern).
    // 無駄時間予測補償器 -- paramは[ms]、SlidingModeRateは[s]。
    // firmware/apps/smc_rate/app_controller.cppと同じパターン。
    float roll_delay_ms = 0.0f, pitch_delay_ms = 0.0f, yaw_delay_ms = 0.0f;
    sf::params::get_float("smc.roll.delay_comp_ms",  roll_delay_ms);
    sf::params::get_float("smc.pitch.delay_comp_ms", pitch_delay_ms);
    sf::params::get_float("smc.yaw.delay_comp_ms",   yaw_delay_ms);
    smc_roll_.delay_comp_s  = roll_delay_ms  * 1.0e-3f;
    smc_pitch_.delay_comp_s = pitch_delay_ms * 1.0e-3f;
    smc_yaw_.delay_comp_s   = yaw_delay_ms   * 1.0e-3f;

    // Same physical torque ceiling the PID rate loop uses -- see
    // firmware/apps/smc_rate/app_controller.cpp's loadSmcParams() for the
    // provenance comment (unchanged here).
    // レートPIDと同じ物理トルク上限 -- 出典コメントは
    // firmware/apps/smc_rate/app_controller.cppのloadSmcParams()参照
    // （ここでは無変更）。
    constexpr float kMaxRollPitchTorque = 5.2e-3f;  // [Nm], mirrors pid_controller.hpp
    smc_roll_.output_limit  = kMaxRollPitchTorque;
    smc_pitch_.output_limit = kMaxRollPitchTorque;
    sf::params::get_float("rate.yaw.max_torque", smc_yaw_.output_limit);
}

void AppController::loadVelSmcParams()
{
    sf::params::get_float("smc.velx.k",        smc_vel_x_.k);
    sf::params::get_float("smc.velx.eta",      smc_vel_x_.eta);
    sf::params::get_float("smc.velx.phi",      smc_vel_x_.phi);
    sf::params::get_float("smc.velx.lambda_i", smc_vel_x_.lambda_i);
    sf::params::get_float("smc.velx.e_reset",  smc_vel_x_.e_reset);
    sf::params::get_float("smc.vely.k",        smc_vel_y_.k);
    sf::params::get_float("smc.vely.eta",      smc_vel_y_.eta);
    sf::params::get_float("smc.vely.phi",      smc_vel_y_.phi);
    sf::params::get_float("smc.vely.lambda_i", smc_vel_y_.lambda_i);
    sf::params::get_float("smc.vely.e_reset",  smc_vel_y_.e_reset);

    // Dead-time predictor -- params are [ms], SlidingModeVelocity wants [s].
    // 無駄時間予測補償器 -- paramは[ms]、SlidingModeVelocityは[s]。
    float velx_delay_ms = 0.0f, vely_delay_ms = 0.0f;
    sf::params::get_float("smc.velx.delay_comp_ms", velx_delay_ms);
    sf::params::get_float("smc.vely.delay_comp_ms", vely_delay_ms);
    smc_vel_x_.delay_comp_s = velx_delay_ms * 1.0e-3f;
    smc_vel_y_.delay_comp_s = vely_delay_ms * 1.0e-3f;

    // Same physical acceleration ceiling PidController's vel_x_/vel_y_ used
    // before being overridden (gravity_ * max_pos_tilt_, pid_controller.cpp)
    // -- see this file's kMaxPosTilt comment above for why it is duplicated
    // here rather than read from PidController.
    // 差し替え前にPidControllerのvel_x_/vel_y_が使っていたのと同じ物理加速度
    // 上限（gravity_*max_pos_tilt_, pid_controller.cpp）-- ここに複製する
    // 理由はファイル先頭のkMaxPosTiltコメント参照。
    const float output_limit = sf::math::kGravity * kMaxPosTilt;
    smc_vel_x_.output_limit = output_limit;
    smc_vel_y_.output_limit = output_limit;
}

void AppController::reset()
{
    pid_.reset();
    smc_roll_.reset();
    smc_pitch_.reset();
    smc_yaw_.reset();
    smc_vel_x_.reset();
    smc_vel_y_.reset();
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
    // startExcitation() for the full rationale (unchanged here: the
    // excitation is injected into pid_'s OWN rate_sp_*, upstream of both the
    // velocity-loop override and the rate-loop SMC).
    // 無改造で転送 -- 詳細な根拠はfirmware/apps/smc_rate/app_controller.cppの
    // startExcitation()参照（ここでは無変更: 励振はpid_自身のrate_sp_*へ
    // 注入され、速度ループの差し替えとレートSMCの両方より上流）。
    pid_.startExcitation(cmd);
}

bool AppController::fetchSysidResult(sf::SysidFreqResult& out)
{
    return pid_.fetchSysidResult(out);
}

void AppController::reloadParams()
{
    pid_.reloadParams();
    loadRateSmcParams();
    loadVelSmcParams();
}

}  // namespace sf::app
