/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.hpp
 * @brief Adaptive super-twisting (2nd-order sliding-mode) rate loop +
 *        adaptive super-twisting horizontal VELOCITY loop: an `IController`
 *        that delegates everything to `PidController` (altitude, POSITION
 *        loop, takeoff/landing phases, guidance, trim/hover-thrust learning,
 *        DOB) except:
 *          - the FINAL roll/pitch/yaw torque, recomputed by 3 independent
 *            `AdaptiveSuperTwisting` instances (output_scale = axis inertia)
 *            fed by `PidController`'s own published rate targets
 *            (`ControlOutput::rate_ref`) -- same idea as smc_rate_asta;
 *          - the NED horizontal ACCELERATION command (vx/vy velocity error
 *            -> ax/ay), recomputed by 2 independent `AdaptiveSuperTwisting`
 *            instances (output_scale left at its default 1.0) plugged into
 *            `PidController` via `setVelocityLawOverride()` -- the position
 *            loop (pos_x_/pos_y_) stays PidController's own PID, unchanged.
 *        Structurally identical to firmware/apps/smc_pos_sta's AppController,
 *        with the fixed-gain `SuperTwisting` swapped for the adaptive-gain
 *        `AdaptiveSuperTwisting` (adaptive_sliding_mode_sta.hpp) -- see that
 *        file's header for why (docs/plans/smc-rate-loop-plan.md, 2026-09-13
 *        session: adaptive STA has been tried on the rate loop only, never
 *        on position/velocity).
 *        適応スーパーツイスティング（2次スライディングモード）のレートループ
 *        + 適応水平速度ループ: 高度・位置ループ・離着陸フェーズ・誘導・
 *        トリム/ホバー推力学習・DOBを含む全てを`PidController`に委譲し、
 *        以下だけを差し替える`IController`:
 *          - roll/pitch/yawの最終トルク（`PidController`自身が公開する
 *            レート目標を入力とする独立3軸の`AdaptiveSuperTwisting`
 *            （output_scale=軸慣性）で再計算 -- smc_rate_astaと同じ考え方）
 *          - NED水平加速度指令（vx/vy速度誤差→ax/ay、`setVelocityLawOverride()`
 *            経由で注入する独立2軸の`AdaptiveSuperTwisting`
 *            （output_scale=既定1.0のまま）で再計算 -- 位置ループは無改造）
 *        firmware/apps/smc_pos_staのAppControllerと構造は同一で、固定ゲイン
 *        `SuperTwisting`を適応ゲイン`AdaptiveSuperTwisting`
 *        （adaptive_sliding_mode_sta.hpp）に差し替えたもの -- 理由は同ファイル
 *        参照（docs/plans/smc-rate-loop-plan.md、2026-09-13セッション:
 *        適応STAはレートループでしか試されておらず、位置/速度では未検証だった）。
 *
 * @design architecture.md §2.5 — L1: IEstimator/IController を実装して差替え [OK]
 * @design controller.hpp — IController interface (12 methods)              [OK]
 * @design docs/plans/smc-rate-loop-plan.md section 7.36 — adaptive STA design (rate-loop origin) [--]
 * @design docs/plans/smc-rate-loop-plan.md 2026-09-13 session — adaptive STA applied to position/velocity [--]
 * @design pid_controller.hpp PidController::setVelocityLawOverride()        [OK]
 * @design architecture.md INV-1/INV-2/INV-5 — delegate-only, trivially compliant [OK]
 */

#pragma once

#include "controller.hpp"
#include "pid_controller.hpp"
#include "adaptive_sliding_mode_sta.hpp"

namespace sf::app {

/// `IController` wrapper around `PidController` that replaces the final
/// rate-loop torque AND the horizontal velocity-loop acceleration with
/// adaptive super-twisting sliding-mode laws. Forwards every other method
/// unchanged.
/// `PidController`を包み、最終レートループのトルクと水平速度ループの加速度を
/// 適応スーパーツイスティング・スライディングモード則に差し替える
/// `IController`。他の全メソッドはそのまま転送する。
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
    /// Re-read the rate-loop adaptive-STA gains from params.cpp. Called by
    /// init() and reloadParams().
    /// レート適応STAのゲインをparams.cppから再読込する。init()と
    /// reloadParams()から呼ばれる。
    void loadRateSmcParams();

    /// Re-read the velocity-loop adaptive-STA gains from params.cpp. Called
    /// by init() and reloadParams().
    /// 速度適応STAのゲインをparams.cppから再読込する。init()と
    /// reloadParams()から呼ばれる。
    void loadVelSmcParams();

    /// The delegate: owns thrust, altitude, position loop, takeoff/landing
    /// phase state machine, guidance, trim/hover-thrust learning, DOB --
    /// everything except the final rate-loop torque and (via the override)
    /// the velocity-loop acceleration.
    /// 委譲先: 推力・高度・位置ループ・離着陸フェーズステートマシン・誘導・
    /// トリム/ホバー推力学習・DOBを所有する -- 最終レートループのトルクと
    /// （差し替え経由の）速度ループの加速度を除く全て。
    sf::PidController pid_;

    /// The 3 independent per-axis adaptive super-twisting rate controllers
    /// replacing pid_'s rate_roll_/rate_pitch_/rate_yaw_ for the FINAL
    /// torque only (output_scale set to each axis's inertia in init()).
    /// pid_のrate_roll_/rate_pitch_/rate_yaw_を最終トルクのみ置き換える、
    /// 独立3軸の適応スーパーツイスティング・レートコントローラ
    /// （init()で各軸の慣性をoutput_scaleに設定）。
    sf::app::AdaptiveSuperTwisting smc_roll_;
    sf::app::AdaptiveSuperTwisting smc_pitch_;
    sf::app::AdaptiveSuperTwisting smc_yaw_;

    /// The 2 independent per-axis adaptive super-twisting velocity
    /// controllers plugged into pid_'s velocity-loop stage (vel_x_/vel_y_
    /// replacement) via setVelocityLawOverride() (output_scale left at its
    /// default 1.0). pid_'s position-loop stage (pos_x_/pos_y_) is untouched.
    /// pid_の速度ループ段（vel_x_/vel_y_の置換）へsetVelocityLawOverride()
    /// 経由で注入される独立2軸の適応スーパーツイスティング速度コントローラ
    /// （output_scaleは既定1.0のまま）。pid_の位置ループ段は無改造。
    sf::app::AdaptiveSuperTwisting smc_vel_x_;
    sf::app::AdaptiveSuperTwisting smc_vel_y_;

    /// Optional slew-rate limit [m/s^2] applied to vx_sp/vy_sp (the
    /// velocity-loop target handed to smc_vel_x_/smc_vel_y_) BEFORE it
    /// reaches the STA -- 0 disables it (byte-identical to the pre-existing
    /// behavior). Added docs/plans/smc-rate-loop-plan.md §7.54続報6: SILS
    /// found a real-flight-identified rate-loop actuator lag (~60ms,
    /// §7.54) reproduces a complete tumble in the pos_roll/pitch/yaw/flight
    /// scenario family (§7.54続報3), and neither raising duty/torque
    /// capacity nor retuning the STA's own gain resolved it (§7.54続報4) --
    /// this ramps the STEP the position loop currently hands the STA into a
    /// ramp instead, so the STA is never asked to react to an instantaneous
    /// setpoint jump under that added actuator lag. Diagnostic/experimental;
    /// default OFF.
    /// vx_sp/vy_sp（速度ループがsmc_vel_x_/smc_vel_y_に渡す目標）に適用する
    /// 任意のスルーレート制限[m/s^2]——STAに届く前に適用する。0で無効
    /// （既存動作とバイト同一）。docs/plans/smc-rate-loop-plan.md §7.54続報6で
    /// 追加: 実機同定したレートループのアクチュエータ遅れ（約60ms、§7.54）を
    /// SILSに注入すると`pos_roll/pitch/yaw/flight`系列シナリオで完全転倒が
    /// 再現し（§7.54続報3）、duty/トルク容量を増やしてもSTA自身のゲイン再調整
    /// でも解決しなかった（§7.54続報4）——本パラメータは位置ループが現在STAに
    /// 渡している「瞬時ステップ」を「ランプ」に整形し、その遅れの下でSTAが
    /// 瞬時の目標跳躍に反応させられる状況そのものをなくす。診断・実験用、
    /// 既定OFF。
    float vsp_slew_max_ = 0.0f;
    float vx_sp_limited_ = 0.0f;
    float vy_sp_limited_ = 0.0f;

    // TEMPORARY diagnostic (docs/plans/smc-rate-loop-plan.md §7.56続報2): decimation
    // counter for a k1/e_model_env ESP_LOGI probe on smc_vel_y_, to check whether the
    // adaptive law's own k1 is what self-activates during the long-hold "3rd loop"
    // symptom (§7.56続報: ay_ned/roll_sp rms grows ~1.56x over 40s while vy_est itself
    // does not). Remove after the measurement.
    // 一時診断（§7.56続報2）: smc_vel_y_のk1/e_model_envをESP_LOGIで観測する間引き
    // カウンタ——長時間保持の「第3のループ」症状（§7.56続報: ay_ned/roll_spのrmsが
    // 40秒でvy_est自体は伸びないまま約1.56倍に成長）が、適応則自身のk1の自己活性化
    // によるものかを確認する。調査後に削除する。
    int posdiag_counter_ = 0;
};

}  // namespace sf::app
