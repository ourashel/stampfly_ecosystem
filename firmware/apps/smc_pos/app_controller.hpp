/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.hpp
 * @brief Sliding-mode rate loop (reused from firmware/apps/smc_rate,
 *        unchanged) + sliding-mode horizontal VELOCITY loop: an
 *        `IController` that delegates everything to `PidController`
 *        (altitude, POSITION loop, takeoff/landing phases, guidance,
 *        trim/hover-thrust learning, DOB) except:
 *          - the FINAL roll/pitch/yaw torque, recomputed by 3 independent
 *            `SlidingModeRate` controllers fed by `PidController`'s own
 *            published rate targets (`ControlOutput::rate_ref`) -- same as
 *            smc_rate;
 *          - the NED horizontal ACCELERATION command (vx/vy velocity error
 *            -> ax/ay), recomputed by 2 independent `SlidingModeVelocity`
 *            controllers plugged into `PidController` via
 *            `setVelocityLawOverride()` -- the position loop (pos_x_/pos_y_)
 *            stays PidController's own PID, unchanged.
 *        レートループのスライディングモード化（firmware/apps/smc_rateから
 *        無変更で流用）+ 水平速度ループのスライディングモード化: 高度・
 *        位置ループ・離着陸フェーズ・誘導・トリム/ホバー推力学習・DOBを
 *        含む全てを`PidController`に委譲し、以下だけを差し替える`IController`:
 *          - roll/pitch/yawの最終トルク（`PidController`自身が公開する
 *            レート目標`ControlOutput::rate_ref`を入力とする独立3軸の
 *            `SlidingModeRate`で再計算 -- smc_rateと同一）
 *          - NED水平加速度指令（vx/vy速度誤差→ax/ay、`PidController`へ
 *            `setVelocityLawOverride()`経由で注入する独立2軸の
 *            `SlidingModeVelocity`で再計算 -- 位置ループ（pos_x_/pos_y_）は
 *            `PidController`自身のPIDのまま無改造）
 *
 * See docs/plans/smc-rate-loop-plan.md §7 for the full design rationale,
 * INCLUDING the caveat that PidController's position/velocity gains were
 * already re-derived from a hardware plant identification (motor-torque-
 * effectiveness shortfall, ~0.4g effective tilt->velocity gain,
 * firmware/vehicle/docs/poshold_journey.md §4) and validated robust over a
 * wide margin (K in [2.8,7]) -- this app exists to A/B/C-compare against
 * that already-robustified baseline, not because the baseline is known
 * broken.
 * 設計の全体像は docs/plans/smc-rate-loop-plan.md §7 参照。これには
 * PidControllerの位置/速度ゲインが、実機プラント同定（モータトルク効き
 * 不足、実効「傾き→速度」ゲイン約0.4g、firmware/vehicle/docs/
 * poshold_journey.md §4）から既に再導出され、広いマージン（K∈[2.8,7]）で
 * ロバスト性検証済みである、という留保も含む——本appは「既存が壊れている
 * から」ではなく、その既に頑健化済みベースラインとのA/B/C比較のために存在する。
 *
 * @design architecture.md §2.5 — L1: IEstimator/IController を実装して差替え [OK]
 * @design controller.hpp — IController interface (12 methods)              [OK]
 * @design docs/plans/smc-rate-loop-plan.md §7 — velocity-loop SMC design    [--]
 * @design pid_controller.hpp PidController::setVelocityLawOverride()        [OK]
 * @design architecture.md INV-1/INV-2/INV-5 — delegate-only, trivially compliant [OK]
 */

#pragma once

#include "controller.hpp"
#include "pid_controller.hpp"
#include "smc_rate.hpp"
#include "smc_vel.hpp"

namespace sf::app {

/// `IController` wrapper around `PidController` that replaces the final
/// rate-loop torque AND the horizontal velocity-loop acceleration with
/// sliding-mode laws. Forwards every other method unchanged.
/// `PidController`を包み、最終レートループのトルクと水平速度ループの加速度を
/// スライディングモード則に差し替える`IController`。他の全メソッドはそのまま
/// 転送する。
class AppController : public sf::IController {
public:
    /// Initialize the wrapped PidController, the 3 rate-loop SMC axes, the 2
    /// velocity-loop SMC axes, and plug the latter into PidController via
    /// setVelocityLawOverride().
    /// 内側のPidController・3軸レートSMC・2軸速度SMCを初期化し、後者を
    /// setVelocityLawOverride()経由でPidControllerへ注入する。
    void init();

    /// Run PidController's full cascade (thrust/altitude/POSITION-loop/phase
    /// state machine untouched -- the velocity loop inside it now calls our
    /// SlidingModeVelocity via the override), then overwrite torque[0..2]
    /// with the sliding-mode rate controllers' output, fed by PidController's
    /// own published rate_ref targets.
    /// PidControllerの全カスケードを実行し（推力/高度/位置ループ/フェーズ
    /// ステートマシンは無改造 -- 内部の速度ループは差し替え経由で
    /// SlidingModeVelocityを呼ぶ）、torque[0..2]だけをスライディングモード・
    /// レートコントローラの出力で上書きする（入力はPidController自身が
    /// 公開するrate_ref目標）。
    sf::ControlOutput compute(
        const sf::StateEstimate& state,
        const sf::CommandSetpoint& setpoint,
        float dt) override;

    void reset() override;
    void onModeChange(sf::FlightMode new_mode) override;
    void onLanding() override;
    void onTakeoff() override;
    void onTakeoffComplete() override;
    bool isTakeoffComplete() const override;
    void setGuidanceTarget(const sf::GuidanceTarget& target,
                           const sf::CommandSetpoint& current_sticks) override;
    bool isGuidanceActive() const override;
    void startExcitation(const sf::SysidCommand& cmd) override;
    bool fetchSysidResult(sf::SysidFreqResult& out) override;
    void reloadParams() override;

private:
    /// Re-read the rate-loop SMC gains (k/eta/phi/lambda_i/e_reset per axis)
    /// from params.cpp. Called by init() and reloadParams(). Identical to
    /// smc_rate's loadSmcParams() -- see app_controller.cpp.
    /// レートSMCのゲインをparams.cppから再読込する。init()とreloadParams()
    /// から呼ばれる。smc_rateのloadSmcParams()と同一 -- app_controller.cpp参照。
    void loadRateSmcParams();

    /// Re-read the velocity-loop SMC gains (k/eta/phi/lambda_i/e_reset per
    /// axis) from params.cpp. Called by init() and reloadParams().
    /// 速度SMCのゲインをparams.cppから再読込する。init()とreloadParams()
    /// から呼ばれる。
    void loadVelSmcParams();

    /// The delegate: owns thrust, altitude, position loop, takeoff/landing
    /// phase state machine, guidance, trim/hover-thrust learning, DOB --
    /// everything except the final rate-loop torque and (via the override)
    /// the velocity-loop acceleration.
    /// 委譲先: 推力・高度・位置ループ・離着陸フェーズステートマシン・誘導・
    /// トリム/ホバー推力学習・DOBを所有する -- 最終レートループのトルクと
    /// （差し替え経由の）速度ループの加速度を除く全て。
    sf::PidController pid_;

    /// The 3 independent per-axis sliding-mode rate controllers replacing
    /// pid_'s rate_roll_/rate_pitch_/rate_yaw_ for the FINAL torque only
    /// (reused unchanged from firmware/apps/smc_rate).
    /// pid_のrate_roll_/rate_pitch_/rate_yaw_を最終トルクのみ置き換える、
    /// 独立3軸のスライディングモード・レートコントローラ
    /// （firmware/apps/smc_rateから無変更で流用）。
    sf::app::SlidingModeRate smc_roll_;
    sf::app::SlidingModeRate smc_pitch_;
    sf::app::SlidingModeRate smc_yaw_;

    /// The 2 independent per-axis sliding-mode velocity controllers plugged
    /// into pid_'s velocity-loop stage (vel_x_/vel_y_ replacement) via
    /// setVelocityLawOverride(). pid_'s position-loop stage (pos_x_/pos_y_)
    /// is untouched.
    /// pid_の速度ループ段（vel_x_/vel_y_の置換）へsetVelocityLawOverride()
    /// 経由で注入される独立2軸のスライディングモード速度コントローラ。
    /// pid_の位置ループ段（pos_x_/pos_y_）は無改造。
    sf::app::SlidingModeVelocity smc_vel_x_;
    sf::app::SlidingModeVelocity smc_vel_y_;
};

}  // namespace sf::app
