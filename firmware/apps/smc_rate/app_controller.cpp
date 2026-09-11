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
 * @design docs/plans/smc-rate-loop-plan.md §2 — rate-loop-only SMC design [--]
 */

#include "app_controller.hpp"
#include "params.hpp"

namespace sf::app {

// X-quad spec inertia Ixx/Iyy/Izz [kg*m^2]. Same values and same caveat as
// firmware/vehicle/tasks/api_task.cpp's kSpecInertia (no inertia SSOT header
// exists to alias -- code_review L-13 -- so every consumer keeps its own
// local copy). Used here to scale the sliding-mode reaching law from
// acceleration/rate units into physical torque [Nm]; see smc_rate.hpp.
// X-quad仕様慣性 Ixx/Iyy/Izz [kg*m^2]。firmware/vehicle/tasks/api_task.cpp の
// kSpecInertia と同じ値・同じ注意点（別名にすべき慣性SSOTヘッダは存在しない
// -- code_review L-13 -- そのため各利用箇所が自分のローカルコピーを持つ）。
// ここではスライディングモード到達則を角加速度/角速度の単位から物理トルク
// [Nm]へスケールするのに使う。smc_rate.hpp参照。
static const float kInertia[3] = {9.16e-6f, 13.3e-6f, 20.4e-6f};  // roll, pitch, yaw

void AppController::init()
{
    pid_.init();
    smc_roll_.inertia  = kInertia[0];
    smc_pitch_.inertia = kInertia[1];
    smc_yaw_.inertia   = kInertia[2];
    loadSmcParams();
}

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    // Run the full PID cascade: thrust, altitude, position, takeoff/landing
    // phase state machine, guidance, trim/hover-thrust learning, DOB, and
    // heading hold are all computed exactly as the default vehicle build
    // would (docs/plans/smc-rate-loop-plan.md §2.1 scope decision). We only
    // discard torque[0..2] below.
    // PIDカスケードを丸ごと実行する: 推力・高度・位置・離着陸フェーズ
    // ステートマシン・誘導・トリム/ホバー推力学習・DOB・ヘディングホールドは
    // 全て既定vehicleビルドと全く同じように計算される
    // （docs/plans/smc-rate-loop-plan.md §2.1 スコープ判断）。以下で破棄する
    // のはtorque[0..2]だけ。
    sf::ControlOutput output = pid_.compute(state, setpoint, dt);

    // output.rate_ref[R,P,Y] is PidController's OWN final rate-loop target
    // (pid_controller.cpp:824-826) -- i.e. it already has heading-hold,
    // guidance, POS_HOLD, and the Landing level-gate baked in. Feeding it to
    // the sliding-mode controllers means everything upstream of the rate
    // loop keeps working unmodified; only the target->torque LAW changes.
    // output.rate_ref[R,P,Y]はPidController自身の最終レートループ目標
    // （pid_controller.cpp:824-826）-- つまりヘディングホールド・誘導・
    // POS_HOLD・Landing水平ゲートが既に反映済み。これをスライディングモード
    // コントローラへ渡すことで、レートループより上流は無改造のまま動き、
    // 目標->トルクの「則」だけが変わる。
    output.torque[0] = smc_roll_.compute(output.rate_ref[0], state.angular_rate[0], dt);
    output.torque[1] = smc_pitch_.compute(output.rate_ref[1], state.angular_rate[1], dt);
    output.torque[2] = smc_yaw_.compute(output.rate_ref[2], state.angular_rate[2], dt);

    // output.thrust / rate_ref / angle_ref are left exactly as pid_ computed
    // them: thrust because the altitude/position cascade is delegated
    // unchanged, and rate_ref/angle_ref because they represent the TARGET
    // (unchanged in meaning) -- keeping them as-is lets `sf log viz` overlay
    // target vs. measured for a direct PID/SMC tracking comparison.
    // output.thrust / rate_ref / angle_ref はpid_の計算値のまま維持する:
    // thrustは高度/位置カスケードが無改造で委譲されているため、
    // rate_ref/angle_refは「目標値」（意味は不変）を表すため -- そのまま
    // 残すことで`sf log viz`が目標と実測を重ねてPID/SMCの追従を直接比較
    // できる。
    return output;
}

void AppController::loadSmcParams()
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

    // Share the SAME physical torque ceiling the PID rate loop uses
    // (rate.yaw.max_torque is runtime-tunable; roll/pitch's cap is a fixed
    // PidController constant not exposed as a param -- see
    // pid_controller.hpp's max_roll_pitch_torque_ comment for the 5.2e-3 Nm
    // provenance). Duplicated here as a local constant for the same reason
    // kInertia is (PidController does not expose it).
    // レートPIDと同じ物理トルク上限を共有する（rate.yaw.max_torqueは実行時
    // 調整可能。roll/pitchの上限はPidControllerの固定定数でparam化されて
    // いない -- 5.2e-3 Nmの出所はpid_controller.hppのmax_roll_pitch_torque_
    // コメント参照）。kInertiaと同じ理由（PidControllerが公開していない）で
    // ここにローカル定数として複製する。
    constexpr float kMaxRollPitchTorque = 5.2e-3f;  // [Nm], mirrors pid_controller.hpp
    smc_roll_.output_limit  = kMaxRollPitchTorque;
    smc_pitch_.output_limit = kMaxRollPitchTorque;
    sf::params::get_float("rate.yaw.max_torque", smc_yaw_.output_limit);
}

void AppController::reset()
{
    pid_.reset();
    smc_roll_.reset();
    smc_pitch_.reset();
    smc_yaw_.reset();
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
    // Forwarded unchanged: PidController injects the chirp/doublet/stepped-
    // sine signal into its OWN rate_sp_* before publishing rate_ref, so the
    // excitation reaches the sliding-mode controllers transparently through
    // compute()'s output.rate_ref read above -- no separate wiring needed.
    // 無改造で転送: PidControllerはチャープ/ダブレット/ステップドサイン信号を
    // 自身のrate_sp_*へ注入してからrate_refを公開するため、励振は上の
    // compute()のoutput.rate_ref読み取りを通じて透過的にスライディングモード
    // コントローラへ届く -- 別配線は不要。
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
