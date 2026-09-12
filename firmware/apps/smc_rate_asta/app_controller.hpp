/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.hpp
 * @brief Adaptive super-twisting rate loop: an `IController` that delegates
 *        everything to `PidController` (altitude, position, takeoff/
 *        landing phases, guidance, trim/hover-thrust learning, DOB) except
 *        the FINAL roll/pitch/yaw torque, which is recomputed by 3
 *        independent `AdaptiveSuperTwistingRate` controllers fed by
 *        `PidController`'s own published rate targets
 *        (`ControlOutput::rate_ref`). Structurally identical to
 *        firmware/apps/smc_rate_sta's AppController, with `SuperTwistingRate`
 *        swapped for `AdaptiveSuperTwistingRate` -- see smc_rate_asta.hpp
 *        for why (smc_rate_sta itself is the FINAL fixed-gain version and
 *        is not modified; this is a separate, independent experiment).
 *        レートループの適応スーパーツイスティング化: 高度・位置・離着陸
 *        フェーズ・誘導・トリム/ホバー推力学習・DOBを含む全てを
 *        `PidController`に委譲し、roll/pitch/yawの最終トルクだけを、
 *        `PidController`自身が公開するレート目標（`ControlOutput::
 *        rate_ref`）を入力とする独立3軸の`AdaptiveSuperTwistingRate`で
 *        再計算する`IController`。firmware/apps/smc_rate_staのAppController
 *        と構造は同一で、`SuperTwistingRate`を`AdaptiveSuperTwistingRate`
 *        に差し替えたもの——理由はsmc_rate_asta.hpp参照（smc_rate_sta自体は
 *        固定ゲイン版の最終バージョンであり変更しない。本appは別・独立の
 *        実験）。
 *
 * @design architecture.md §2.5 — L1: IEstimator/IController を実装して差替え [OK]
 * @design controller.hpp — IController interface (12 methods)              [OK]
 * @design docs/plans/smc-rate-loop-plan.md §7.31 — adaptive STA trial       [--]
 * @design architecture.md INV-1/INV-2/INV-5 — delegate-only, trivially compliant [OK]
 */

#pragma once

#include "controller.hpp"
#include "pid_controller.hpp"
#include "smc_rate_asta.hpp"

namespace sf::app {

/// `IController` wrapper around `PidController` that replaces the final
/// rate-loop torque computation with an adaptive-gain super-twisting
/// (2nd-order sliding mode) law.
/// `PidController`を包み、最終レートループのトルク計算だけを適応ゲイン
/// スーパーツイスティング（2次スライディングモード）則に差し替える
/// `IController`。
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
    /// Re-read the adaptive-STA gains (k1_init/k1_min/k1_max/k2_ratio/
    /// adapt_rate/leak_ratio/dead_band/phi/lambda_i/e_reset/z_leak_tau/
    /// output_limit per axis) from params.cpp. Called by init() and
    /// reloadParams().
    /// 適応STAのゲイン（軸ごとのk1_init/k1_min/k1_max/k2_ratio/adapt_rate/
    /// leak_ratio/dead_band/phi/lambda_i/e_reset/z_leak_tau/output_limit）を
    /// params.cppから再読込する。init()とreloadParams()から呼ばれる。
    void loadSmcParams();

    /// The delegate: owns thrust, altitude, position, takeoff/landing phase
    /// state machine, guidance, trim/hover-thrust learning, DOB -- everything
    /// except the final rate-loop torque.
    /// 委譲先: 推力・高度・位置・離着陸フェーズステートマシン・誘導・
    /// トリム/ホバー推力学習・DOBを所有する -- 最終レートループのトルクを
    /// 除く全て。
    sf::PidController pid_;

    /// The 3 independent per-axis adaptive super-twisting rate controllers
    /// replacing pid_'s rate_roll_/rate_pitch_/rate_yaw_ for the FINAL
    /// torque only.
    /// pid_のrate_roll_/rate_pitch_/rate_yaw_を最終トルクのみ置き換える、
    /// 独立3軸の適応スーパーツイスティング・レートコントローラ。
    sf::app::AdaptiveSuperTwistingRate sta_roll_;
    sf::app::AdaptiveSuperTwistingRate sta_pitch_;
    sf::app::AdaptiveSuperTwistingRate sta_yaw_;
};

}  // namespace sf::app
