/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.hpp
 * @brief Super-twisting (2nd-order sliding-mode) rate loop + super-twisting
 *        horizontal VELOCITY loop: an `IController` that delegates
 *        everything to `PidController` (altitude, POSITION loop,
 *        takeoff/landing phases, guidance, trim/hover-thrust learning, DOB)
 *        except:
 *          - the FINAL roll/pitch/yaw torque, recomputed by 3 independent
 *            `SuperTwisting` instances (output_scale = axis inertia) fed by
 *            `PidController`'s own published rate targets
 *            (`ControlOutput::rate_ref`) -- same idea as smc_rate_sta, but
 *            using the sliding_mode_sta.hpp instance shared with the
 *            velocity axes below;
 *          - the NED horizontal ACCELERATION command (vx/vy velocity error
 *            -> ax/ay), recomputed by 2 independent `SuperTwisting`
 *            instances (output_scale left at its default 1.0) plugged into
 *            `PidController` via `setVelocityLawOverride()` -- the position
 *            loop (pos_x_/pos_y_) stays PidController's own PID, unchanged.
 *        All 5 instances share ONE struct (`sliding_mode_sta.hpp`'s
 *        `SuperTwisting`) -- see that file's header for why this app unifies
 *        the rate- and velocity-loop control laws that smc_rate_sta.hpp and
 *        an earlier draft of this app kept as separate near-duplicate types.
 *        スーパーツイスティング法（2次スライディングモード）のレートループ
 *        + 水平速度ループ: 高度・位置ループ・離着陸フェーズ・誘導・
 *        トリム/ホバー推力学習・DOBを含む全てを`PidController`に委譲し、
 *        以下だけを差し替える`IController`:
 *          - roll/pitch/yawの最終トルク（`PidController`自身が公開する
 *            レート目標`ControlOutput::rate_ref`を入力とする独立3軸の
 *            `SuperTwisting`（output_scale=軸慣性）で再計算 -- smc_rate_sta
 *            と同じ考え方だが、下の速度軸と同じsliding_mode_sta.hppの
 *            インスタンスを使う）
 *          - NED水平加速度指令（vx/vy速度誤差→ax/ay、`PidController`へ
 *            `setVelocityLawOverride()`経由で注入する独立2軸の
 *            `SuperTwisting`（output_scale=既定1.0のまま）で再計算 -- 位置
 *            ループ（pos_x_/pos_y_）は`PidController`自身のPIDのまま無改造）
 *        5インスタンス全てが1つの構造体（sliding_mode_sta.hppの
 *        `SuperTwisting`）を共有する -- smc_rate_sta.hppと本appの初期案が
 *        別々のほぼ重複した型として持っていたレート/速度ループ制御則を
 *        本appが統合した理由はそのファイルのヘッダ参照。
 *
 * See docs/plans/smc-rate-loop-plan.md §7 (velocity-loop SMC design/caveats)
 * and §7.11-7.24 (STA design, tuning history, z-leak safety mechanism) for
 * the full rationale, INCLUDING the caveat that PidController's
 * position/velocity gains were already re-derived from a hardware plant
 * identification and validated robust over a wide margin (K in [2.8,7]) --
 * this app exists to A/B/C-compare against that already-robustified
 * baseline AND against firmware/apps/smc_pos's 1st-order SMC (which §7.9
 * found safe but not clearly superior to PID), not because either baseline
 * is known broken.
 * 設計の全体像は docs/plans/smc-rate-loop-plan.md §7（速度ループSMC設計・
 * 留保）と §7.11-7.24（STA設計・チューニング履歴・z漏れ積分安全機構）参照。
 * これにはPidControllerの位置/速度ゲインが実機プラント同定から既に再導出され
 * 広いマージン（K∈[2.8,7]）でロバスト性検証済みである、という留保も含む
 * ——本appは、その既に頑健化済みベースラインと、firmware/apps/smc_posの
 * 1次SMC（§7.9で「安全だが明確に優れてはいない」と判定済み）の両方との
 * A/B/C比較のために存在する（どちらかが壊れているからではない）。
 *
 * @design architecture.md §2.5 — L1: IEstimator/IController を実装して差替え [OK]
 * @design controller.hpp — IController interface (12 methods)              [OK]
 * @design docs/plans/smc-rate-loop-plan.md §7 — velocity-loop SMC design    [--]
 * @design docs/plans/smc-rate-loop-plan.md §7.11-7.24 — STA design          [--]
 * @design pid_controller.hpp PidController::setVelocityLawOverride()        [OK]
 * @design architecture.md INV-1/INV-2/INV-5 — delegate-only, trivially compliant [OK]
 */

#pragma once

#include "controller.hpp"
#include "pid_controller.hpp"
#include "sliding_mode_sta.hpp"

namespace sf::app {

/// `IController` wrapper around `PidController` that replaces the final
/// rate-loop torque AND the horizontal velocity-loop acceleration with
/// super-twisting sliding-mode laws. Forwards every other method unchanged.
/// `PidController`を包み、最終レートループのトルクと水平速度ループの加速度を
/// スーパーツイスティング・スライディングモード則に差し替える`IController`。
/// 他の全メソッドはそのまま転送する。
class AppController : public sf::IController {
public:
    /// Initialize the wrapped PidController, the 3 rate-loop STA axes, the 2
    /// velocity-loop STA axes, and plug the latter into PidController via
    /// setVelocityLawOverride().
    /// 内側のPidController・3軸レートSTA・2軸速度STAを初期化し、後者を
    /// setVelocityLawOverride()経由でPidControllerへ注入する。
    void init();

    /// Run PidController's full cascade (thrust/altitude/POSITION-loop/phase
    /// state machine untouched -- the velocity loop inside it now calls our
    /// SuperTwisting velocity axes via the override), then overwrite
    /// torque[0..2] with the super-twisting rate controllers' output, fed by
    /// PidController's own published rate_ref targets.
    /// PidControllerの全カスケードを実行し（推力/高度/位置ループ/フェーズ
    /// ステートマシンは無改造 -- 内部の速度ループは差し替え経由で
    /// SuperTwisting速度軸を呼ぶ）、torque[0..2]だけをスーパーツイスティング・
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
    /// Re-read the rate-loop STA gains (k1/k2/phi/lambda_i/e_reset/z_leak_tau
    /// per axis) from params.cpp. Called by init() and reloadParams().
    /// レートSTAのゲインをparams.cppから再読込する。init()とreloadParams()
    /// から呼ばれる。
    void loadRateSmcParams();

    /// Re-read the velocity-loop STA gains (k1/k2/phi/lambda_i/e_reset/
    /// z_leak_tau per axis) from params.cpp. Called by init() and
    /// reloadParams().
    /// 速度STAのゲインをparams.cppから再読込する。init()とreloadParams()
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

    /// The 3 independent per-axis super-twisting rate controllers replacing
    /// pid_'s rate_roll_/rate_pitch_/rate_yaw_ for the FINAL torque only
    /// (output_scale set to each axis's inertia in init()).
    /// pid_のrate_roll_/rate_pitch_/rate_yaw_を最終トルクのみ置き換える、
    /// 独立3軸のスーパーツイスティング・レートコントローラ
    /// （init()で各軸の慣性をoutput_scaleに設定）。
    sf::app::SuperTwisting smc_roll_;
    sf::app::SuperTwisting smc_pitch_;
    sf::app::SuperTwisting smc_yaw_;

    /// The 2 independent per-axis super-twisting velocity controllers
    /// plugged into pid_'s velocity-loop stage (vel_x_/vel_y_ replacement)
    /// via setVelocityLawOverride() (output_scale left at its default 1.0 --
    /// acceleration is the physical output quantity directly). pid_'s
    /// position-loop stage (pos_x_/pos_y_) is untouched.
    /// pid_の速度ループ段（vel_x_/vel_y_の置換）へsetVelocityLawOverride()
    /// 経由で注入される独立2軸のスーパーツイスティング速度コントローラ
    /// （output_scaleは既定1.0のまま -- 加速度は物理出力量そのもの）。
    /// pid_の位置ループ段（pos_x_/pos_y_）は無改造。
    sf::app::SuperTwisting smc_vel_x_;
    sf::app::SuperTwisting smc_vel_y_;
};

}  // namespace sf::app
