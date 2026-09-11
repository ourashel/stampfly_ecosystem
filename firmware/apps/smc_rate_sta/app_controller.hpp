/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.hpp
 * @brief Super-twisting rate loop: an `IController` that delegates
 *        everything to `PidController` (altitude, position, takeoff/
 *        landing phases, guidance, trim/hover-thrust learning, DOB) except
 *        the FINAL roll/pitch/yaw torque, which is recomputed by 3
 *        independent `SuperTwistingRate` controllers fed by
 *        `PidController`'s own published rate targets
 *        (`ControlOutput::rate_ref`). Structurally identical to
 *        firmware/apps/smc_rate's AppController, with `SlidingModeRate`
 *        swapped for `SuperTwistingRate` -- see smc_rate_sta.hpp for why.
 *        レートループのスーパーツイスティング化: 高度・位置・離着陸
 *        フェーズ・誘導・トリム/ホバー推力学習・DOBを含む全てを
 *        `PidController`に委譲し、roll/pitch/yawの最終トルクだけを、
 *        `PidController`自身が公開するレート目標（`ControlOutput::
 *        rate_ref`）を入力とする独立3軸の`SuperTwistingRate`で再計算する
 *        `IController`。firmware/apps/smc_rateのAppControllerと構造は
 *        同一で、`SlidingModeRate`を`SuperTwistingRate`に差し替えたもの
 *        -- 理由はsmc_rate_sta.hpp参照。
 *
 * @design architecture.md §2.5 — L1: IEstimator/IController を実装して差替え [OK]
 * @design controller.hpp — IController interface (12 methods)              [OK]
 * @design docs/plans/smc-rate-loop-plan.md §7.11 — STA trial for motor-delay [--]
 * @design architecture.md INV-1/INV-2/INV-5 — delegate-only, trivially compliant [OK]
 */

#pragma once

#include "controller.hpp"
#include "pid_controller.hpp"
#include "smc_rate_sta.hpp"

namespace sf::app {

/// `IController` wrapper around `PidController` that replaces the final
/// rate-loop torque computation with a super-twisting (2nd-order sliding
/// mode) law.
/// `PidController`を包み、最終レートループのトルク計算だけをスーパー
/// ツイスティング（2次スライディングモード）則に差し替える`IController`。
class AppController : public sf::IController {
public:
    void init();

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
    /// Re-read the STA gains (k1/k2/phi/lambda_i/e_reset/output_limit per
    /// axis) from params.cpp. Called by init() and reloadParams().
    /// STAのゲイン（軸ごとのk1/k2/phi/lambda_i/e_reset/output_limit）を
    /// params.cppから再読込する。init()とreloadParams()から呼ばれる。
    void loadSmcParams();

    /// The delegate: owns thrust, altitude, position, takeoff/landing phase
    /// state machine, guidance, trim/hover-thrust learning, DOB -- everything
    /// except the final rate-loop torque.
    /// 委譲先: 推力・高度・位置・離着陸フェーズステートマシン・誘導・
    /// トリム/ホバー推力学習・DOBを所有する -- 最終レートループのトルクを
    /// 除く全て。
    sf::PidController pid_;

    /// The 3 independent per-axis super-twisting rate controllers replacing
    /// pid_'s rate_roll_/rate_pitch_/rate_yaw_ for the FINAL torque only.
    /// pid_のrate_roll_/rate_pitch_/rate_yaw_を最終トルクのみ置き換える、
    /// 独立3軸のスーパーツイスティング・レートコントローラ。
    sf::app::SuperTwistingRate sta_roll_;
    sf::app::SuperTwistingRate sta_pitch_;
    sf::app::SuperTwistingRate sta_yaw_;
};

}  // namespace sf::app
