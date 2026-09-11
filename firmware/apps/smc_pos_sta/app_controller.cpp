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
 * @design docs/plans/smc-rate-loop-plan.md §7 / §7.11-7.24 — rate+velocity-loop STA design [--]
 */

#include "app_controller.hpp"
#include "params.hpp"
#include "sf_math.hpp"

namespace sf::app {

// X-quad spec inertia Ixx/Iyy/Izz [kg*m^2] -- same value and same caveat as
// firmware/apps/smc_rate_sta/app_controller.cpp's kInertia. Used to set the
// rate-loop SuperTwisting instances' output_scale.
// X-quad仕様慣性 Ixx/Iyy/Izz [kg*m^2] -- firmware/apps/smc_rate_sta/
// app_controller.cppのkInertiaと同じ値・同じ注意点。レートループの
// SuperTwistingインスタンスのoutput_scale設定に使う。
static const float kInertia[3] = {9.16e-6f, 13.3e-6f, 20.4e-6f};  // roll, pitch, yaw

// POS_HOLD tilt limit [rad] -- same value as PidController's private
// max_pos_tilt_ (pid_controller.hpp), duplicated here for the same reason
// firmware/apps/smc_pos/app_controller.cpp's kMaxPosTilt is: PidController
// does not expose it as a param, and this app needs it (with kGravity) to
// set the velocity-loop STA's output_limit to the SAME physical ceiling
// vel_x_/vel_y_ used before being overridden.
// POS_HOLDの傾き上限 [rad] -- PidControllerの非公開max_pos_tilt_
// （pid_controller.hpp）と同じ値。firmware/apps/smc_pos/app_controller.cpp
// のkMaxPosTiltと同じ理由でここに複製する。
static constexpr float kMaxPosTilt = 0.1745f;  // [rad] (10 deg)

void AppController::init()
{
    pid_.init();
    smc_roll_.output_scale  = kInertia[0];
    smc_pitch_.output_scale = kInertia[1];
    smc_yaw_.output_scale   = kInertia[2];
    // smc_vel_x_/smc_vel_y_ keep the struct's default output_scale=1.0
    // (acceleration is the physical output quantity, no scaling needed).
    // smc_vel_x_/smc_vel_y_は構造体既定のoutput_scale=1.0のまま
    // （加速度は物理出力量そのもの、スケール不要）。
    loadRateSmcParams();
    loadVelSmcParams();

    // Plug the velocity-loop STA into pid_'s position cascade (see
    // pid_controller.hpp PidController::setVelocityLawOverride()). pid_'s
    // OWN position-loop stage (pos_x_/pos_y_) stays untouched -- only the
    // vx/vy -> ax/ay stage is replaced, for BOTH the stick-reposition and
    // hold paths (they share this one stage, pid_controller.cpp:1300-1333).
    // pid_の位置カスケードへ速度STAを注入する（PidController::
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
    // only discard torque[0..2] below (same as smc_rate/smc_pos).
    // カスケードを丸ごと実行する: 推力・高度・位置ループ（pos_x_/pos_y_、
    // 無改造）・離着陸フェーズステートマシン・誘導・トリム/ホバー推力学習・
    // DOB・ヘディングホールドは全て既定vehicleビルドと全く同じように計算
    // される。この呼び出し内部の速度ループは、init()で登録した差し替え口
    // 経由で既にsmc_vel_x_/smc_vel_y_を通っている
    // （docs/plans/smc-rate-loop-plan.md §7参照）。以下で破棄するのは
    // torque[0..2]だけ（smc_rate/smc_posと同じ）。
    sf::ControlOutput output = pid_.compute(state, setpoint, dt);

    // output.rate_ref[R,P,Y] is PidController's OWN final rate-loop target,
    // already reflecting whatever roll/pitch tilt the (STA-driven) velocity
    // loop commanded this cycle -- see firmware/apps/smc_rate_sta's
    // app_controller.cpp for the full rationale (unchanged here).
    // output.rate_ref[R,P,Y]はPidController自身の最終レートループ目標で、
    // （STA駆動の）速度ループが今サイクル指令したroll/pitch傾きを既に反映済み
    // -- 詳細な根拠はfirmware/apps/smc_rate_staのapp_controller.cpp参照
    // （ここでは無変更）。
    output.torque[0] = smc_roll_.compute(output.rate_ref[0], state.angular_rate[0], dt);
    output.torque[1] = smc_pitch_.compute(output.rate_ref[1], state.angular_rate[1], dt);
    output.torque[2] = smc_yaw_.compute(output.rate_ref[2], state.angular_rate[2], dt);

    return output;
}

void AppController::loadRateSmcParams()
{
    sf::params::get_float("smc_pos_sta.roll.k1",        smc_roll_.k1);
    sf::params::get_float("smc_pos_sta.roll.k2",        smc_roll_.k2);
    sf::params::get_float("smc_pos_sta.roll.phi",       smc_roll_.phi);
    sf::params::get_float("smc_pos_sta.roll.lambda_i",  smc_roll_.lambda_i);
    sf::params::get_float("smc_pos_sta.roll.e_reset",   smc_roll_.e_reset);
    sf::params::get_float("smc_pos_sta.roll.z_leak_tau", smc_roll_.z_leak_tau);
    sf::params::get_float("smc_pos_sta.pitch.k1",        smc_pitch_.k1);
    sf::params::get_float("smc_pos_sta.pitch.k2",        smc_pitch_.k2);
    sf::params::get_float("smc_pos_sta.pitch.phi",       smc_pitch_.phi);
    sf::params::get_float("smc_pos_sta.pitch.lambda_i",  smc_pitch_.lambda_i);
    sf::params::get_float("smc_pos_sta.pitch.e_reset",   smc_pitch_.e_reset);
    sf::params::get_float("smc_pos_sta.pitch.z_leak_tau", smc_pitch_.z_leak_tau);
    sf::params::get_float("smc_pos_sta.yaw.k1",        smc_yaw_.k1);
    sf::params::get_float("smc_pos_sta.yaw.k2",        smc_yaw_.k2);
    sf::params::get_float("smc_pos_sta.yaw.phi",       smc_yaw_.phi);
    sf::params::get_float("smc_pos_sta.yaw.lambda_i",  smc_yaw_.lambda_i);
    sf::params::get_float("smc_pos_sta.yaw.e_reset",   smc_yaw_.e_reset);
    sf::params::get_float("smc_pos_sta.yaw.z_leak_tau", smc_yaw_.z_leak_tau);

    // Same physical torque ceiling the PID rate loop uses -- see
    // firmware/apps/smc_rate_sta/app_controller.cpp's loadSmcParams() for
    // the provenance comment (unchanged here).
    // レートPIDと同じ物理トルク上限 -- 出典コメントは
    // firmware/apps/smc_rate_sta/app_controller.cppのloadSmcParams()参照
    // （ここでは無変更）。
    constexpr float kMaxRollPitchTorque = 5.2e-3f;  // [Nm], mirrors pid_controller.hpp
    smc_roll_.output_limit  = kMaxRollPitchTorque;
    smc_pitch_.output_limit = kMaxRollPitchTorque;
    sf::params::get_float("rate.yaw.max_torque", smc_yaw_.output_limit);
}

void AppController::loadVelSmcParams()
{
    sf::params::get_float("smc_pos_sta.velx.k1",        smc_vel_x_.k1);
    sf::params::get_float("smc_pos_sta.velx.k2",        smc_vel_x_.k2);
    sf::params::get_float("smc_pos_sta.velx.phi",       smc_vel_x_.phi);
    sf::params::get_float("smc_pos_sta.velx.lambda_i",  smc_vel_x_.lambda_i);
    sf::params::get_float("smc_pos_sta.velx.e_reset",   smc_vel_x_.e_reset);
    sf::params::get_float("smc_pos_sta.velx.z_leak_tau", smc_vel_x_.z_leak_tau);
    sf::params::get_float("smc_pos_sta.vely.k1",        smc_vel_y_.k1);
    sf::params::get_float("smc_pos_sta.vely.k2",        smc_vel_y_.k2);
    sf::params::get_float("smc_pos_sta.vely.phi",       smc_vel_y_.phi);
    sf::params::get_float("smc_pos_sta.vely.lambda_i",  smc_vel_y_.lambda_i);
    sf::params::get_float("smc_pos_sta.vely.e_reset",   smc_vel_y_.e_reset);
    sf::params::get_float("smc_pos_sta.vely.z_leak_tau", smc_vel_y_.z_leak_tau);

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
    // pid_.onModeChange() resets PidController's OWN pos_x_/pos_y_/vel_x_/
    // vel_y_ on a POS_HOLD-boundary transition and re-captures the current
    // position as the new hold target (capture_pos_ = true) -- see
    // pid_controller.cpp's onModeChange() comment. But those pos_x_/vel_x_
    // instances are DEAD CODE here (bypassed by setVelocityLawOverride() in
    // init()) -- the STA velocity controllers that are ACTUALLY driving the
    // vehicle, smc_vel_x_/smc_vel_y_, were never reset on a mode change,
    // only on a full AppController::reset(). That gap meant a freshly
    // re-captured position target (from PidController) could be fed to a
    // velocity-loop STA instance still carrying stale integral/z state from
    // before the transition -- found from real-flight WiFi telemetry
    // (rapid STABILIZE<->POS_HOLD toggling, docs/plans/
    // smc-rate-loop-plan.md §7.27) showing rate-tracking error spiking
    // >10x (to 300-460 deg/s RMSE, peak >1800 deg/s) in the 0.3s right
    // after each mode transition, vs. 30-40 deg/s RMSE in steady POS_HOLD
    // flight. Resetting smc_vel_x_/smc_vel_y_ here mirrors what
    // PidController does for its own (otherwise-unused) vel_x_/vel_y_,
    // keeping the override's behavior consistent with the delegate it
    // replaces at the SAME transition boundary.
    // Rate loop (smc_roll_/smc_pitch_/smc_yaw_) is NOT reset here --
    // PidController doesn't reset its own rate_roll_/rate_pitch_/rate_yaw_
    // on a mode change either (the rate loop is the innermost, fastest,
    // mode-agnostic loop), so this preserves that same design intent.
    // pid_.onModeChange()はPOS_HOLD境界を跨ぐ遷移でPidController自身の
    // pos_x_/pos_y_/vel_x_/vel_y_をリセットし、現在位置を新しい保持目標として
    // 再捕捉する（capture_pos_=true、pid_controller.cppのonModeChange()コメント
    // 参照）。しかしそのpos_x_/vel_x_インスタンスはここでは**死んだコード**
    // （init()のsetVelocityLawOverride()で迂回される）——実際に機体を駆動している
    // STA速度コントローラsmc_vel_x_/smc_vel_y_は、モード切替時には一度も
    // リセットされておらず、AppController::reset()の全体リセット時のみだった。
    // このギャップにより、（PidControllerから）新たに再捕捉された位置目標が、
    // 遷移前の積分/zの古い状態をまだ持ち越したままの速度ループSTAインスタンスへ
    // 与えられる状況が起きていた——実機飛行のWiFiテレメトリで発見
    // （STABILIZE⇔POS_HOLDの高速トグル、docs/plans/smc-rate-loop-plan.md
    // §7.27）：モード遷移直後0.3秒だけレート追従誤差が10倍超に跳ね上がる
    // （RMSE 300-460°/s、最大1800°/s超）一方、POS_HOLD安定飛行時は30-40°/s
    // RMSE。ここでsmc_vel_x_/smc_vel_y_をリセットすることで、PidControllerが
    // 自身の（本来は未使用の）vel_x_/vel_y_に対して行っている挙動を再現し、
    // 差し替え口の挙動を同じ遷移境界で置き換え先の委譲先と整合させる。
    // レートループ（smc_roll_/smc_pitch_/smc_yaw_）はここではリセットしない
    // -- PidController自身もモード切替でrate_roll_/rate_pitch_/rate_yaw_を
    // リセットしないため（レートループは最内周・最速でモードに依存しない）、
    // 同じ設計意図を保つ。
    smc_vel_x_.reset();
    smc_vel_y_.reset();
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
    // Forwarded unchanged -- see firmware/apps/smc_rate_sta/app_controller.cpp's
    // startExcitation() for the full rationale (unchanged here: the
    // excitation is injected into pid_'s OWN rate_sp_*, upstream of both the
    // velocity-loop override and the rate-loop STA).
    // 無改造で転送 -- 詳細な根拠はfirmware/apps/smc_rate_sta/app_controller.cpp
    // のstartExcitation()参照（ここでは無変更: 励振はpid_自身のrate_sp_*へ
    // 注入され、速度ループの差し替えとレートSTAの両方より上流）。
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
