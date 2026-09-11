/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.hpp
 * @brief Sliding-mode rate loop: an `IController` that delegates everything
 *        to `PidController` (altitude, position, takeoff/landing phases,
 *        guidance, trim/hover-thrust learning, DOB) except the FINAL
 *        roll/pitch/yaw torque, which is recomputed by 3 independent
 *        `SlidingModeRate` controllers fed by `PidController`'s own
 *        published rate targets (`ControlOutput::rate_ref`).
 *        レートループのスライディングモード化: 高度・位置・離着陸フェーズ・
 *        誘導・トリム/ホバー推力学習・DOBを含む全てを`PidController`に
 *        委譲し、roll/pitch/yawの最終トルクだけを、`PidController`自身が
 *        公開するレート目標（`ControlOutput::rate_ref`）を入力とする
 *        独立3軸の`SlidingModeRate`で再計算する`IController`。
 *
 * This is a variant of examples/11_app_controller's `adjust()` pattern, but
 * instead of scaling the PID's final torque post-hoc, it DISCARDS
 * torque[0..2] and recomputes them from PidController's own intermediate
 * rate_ref[3] target (populated at pid_controller.cpp:824-826, AFTER
 * heading-hold/guidance/POS_HOLD/landing-gate have all been applied) --
 * everything upstream of the rate loop (and the whole thrust/altitude/
 * position/phase state machine) is untouched. See
 * docs/plans/smc-rate-loop-plan.md for the full design rationale.
 *
 * examples/11_app_controller の`adjust()`パターンの派生形だが、PIDの最終
 * トルクを事後的にスケールする代わりに、torque[0..2]を破棄し
 * PidController自身の中間目標rate_ref[3]（pid_controller.cpp:824-826で
 * 設定、ヘディングホールド/誘導/POS_HOLD/着陸ゲートを全て適用した後の値）
 * から再計算する -- レートループより上流（推力/高度/位置/フェーズ
 * ステートマシン全体）は無改造。設計の全体像は
 * docs/plans/smc-rate-loop-plan.md 参照。
 *
 * @design architecture.md §2.5 — L1: IEstimator/IController を実装して差替え [OK]
 * @design controller.hpp — IController interface (12 methods)              [OK]
 * @design docs/plans/smc-rate-loop-plan.md §2 — rate-loop-only SMC design   [--]
 * @design architecture.md INV-1/INV-2/INV-5 — delegate-only, trivially compliant [OK]
 */

#pragma once

#include "controller.hpp"
#include "pid_controller.hpp"
#include "smc_rate.hpp"

namespace sf::app {

/// `IController` wrapper around `PidController` that replaces the final
/// rate-loop torque computation with a sliding-mode law. Forwards every
/// other method (including startExcitation/fetchSysidResult, which
/// transparently exercises the SMC too -- see file header) unchanged.
/// `PidController`を包み、最終レートループのトルク計算だけをスライディング
/// モード則に差し替える`IController`。他の全メソッド（startExcitation/
/// fetchSysidResultを含む -- SMCも透過的に励振される、ファイル先頭参照）は
/// そのまま転送する。
class AppController : public sf::IController {
public:
    /// Initialize the wrapped PidController and the 3 SMC axes (inertia +
    /// initial gains from params.cpp).
    /// 内側のPidControllerと3軸SMC（慣性+params.cppの初期ゲイン）を初期化する。
    void init();

    /// Run PidController's full cascade (thrust/altitude/position/phase
    /// state machine untouched), then overwrite torque[0..2] with the
    /// sliding-mode rate controllers' output, fed by PidController's own
    /// published rate_ref targets.
    /// PidControllerの全カスケードを実行し（推力/高度/位置/フェーズ
    /// ステートマシンは無改造）、torque[0..2]だけをスライディングモード・
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
    /// Re-read the SMC gains (k/eta/phi/output_limit per axis) from
    /// params.cpp. Called by init() and reloadParams(). See
    /// app_controller.cpp for the param names.
    /// SMCのゲイン（軸ごとのk/eta/phi/output_limit）をparams.cppから
    /// 再読込する。init()とreloadParams()から呼ばれる。パラメータ名は
    /// app_controller.cppを参照。
    void loadSmcParams();

    /// The delegate: owns thrust, altitude, position, takeoff/landing phase
    /// state machine, guidance, trim/hover-thrust learning, DOB, heading
    /// hold -- everything except the final rate-loop torque.
    /// 委譲先: 推力・高度・位置・離着陸フェーズステートマシン・誘導・
    /// トリム/ホバー推力学習・DOB・ヘディングホールドを所有する --
    /// 最終レートループのトルクを除く全て。
    sf::PidController pid_;

    /// The 3 independent per-axis sliding-mode rate controllers replacing
    /// pid_'s rate_roll_/rate_pitch_/rate_yaw_ for the FINAL torque only.
    /// pid_のrate_roll_/rate_pitch_/rate_yaw_を最終トルクのみ置き換える、
    /// 独立3軸のスライディングモード・レートコントローラ。
    sf::app::SlidingModeRate smc_roll_;
    sf::app::SlidingModeRate smc_pitch_;
    sf::app::SlidingModeRate smc_yaw_;
};

}  // namespace sf::app
