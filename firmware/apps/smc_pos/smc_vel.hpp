/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file smc_vel.hpp
 * @brief Single-axis sliding-mode HORIZONTAL VELOCITY controller: the same
 *        PI-type sliding surface + boundary-layer reaching law + 3-layer
 *        anti-windup as smc_rate.hpp's SlidingModeRate, re-derived for the
 *        NED velocity-loop stage of POS_HOLD's position cascade (velocity
 *        error -> horizontal acceleration, instead of rate error -> torque).
 *        1軸スライディングモード・水平速度コントローラ: smc_rate.hppの
 *        SlidingModeRateと同じPI型スライディング面＋境界層付き到達則＋
 *        3層アンチワインドアップを、POS_HOLD位置カスケードの速度ループ段
 *        （速度誤差→水平加速度、レート誤差→トルクの代わり）向けに再導出。
 *
 * Sliding surface (PI-type, same structure as smc_rate.hpp):
 *
 *   e = vel_sp - vel_meas          [m/s]
 *   s = e + lambda_i * integral(e dt)
 *
 * Reaching law:
 *
 *   accel = k*sat(s/phi) + eta*s   [m/s^2]
 *
 * No inertia/mass multiplier is needed here (unlike the rate loop's
 * torque = I_axis*(...)): the output IS the physical quantity itself
 * (acceleration), not a force/torque that a mass/inertia term would scale
 * into. k[m/s^2] and eta[1/s] play exactly the role inertia*k and inertia*eta
 * played in SlidingModeRate.
 *
 * Relative degree: acceleration-command -> velocity is relative degree 1
 * (the SAME structure as torque -> rate in the rate loop), under the
 * standard cascade time-scale-separation assumption that the inner angle+
 * rate loop tracks the resulting tilt command fast enough -- the same
 * assumption the existing linear vel_x_/vel_y_ PID already makes. So s=e
 * (plus the PI-surface integral term for steady-bias rejection) suffices,
 * exactly as it did for the rate loop.
 *
 * Design rationale, motivation and the "already-robustified PID" caveat:
 * docs/plans/smc-rate-loop-plan.md §7.
 *
 * スライディング面（PI型、smc_rate.hppと同じ構造）:
 *
 *   e = vel_sp - vel_meas          [m/s]
 *   s = e + lambda_i * integral(e dt)
 *
 * 到達則:
 *
 *   accel = k*sat(s/phi) + eta*s   [m/s^2]
 *
 * ここでは慣性/質量の乗数は不要（レートループの torque = I_axis*(...) とは
 * 異なる）: 出力そのものが物理量（加速度）であり、質量/慣性項でスケール
 * すべき力/トルクではないため。k[m/s^2]とeta[1/s]は、SlidingModeRateで
 * inertia*k・inertia*etaが担っていた役割をそのまま担う。
 *
 * 相対次数: 加速度指令→速度の相対次数は1（レートループのトルク→レートと
 * 同じ構造）、内側の角度+レートループが結果の傾き指令を十分速く追従すると
 * いう標準的なカスケード時間尺度分離の仮定の下で——既存の線形vel_x_/vel_y_
 * PIDが既に置いている仮定と同じ。よってレートループと同じく、s=e
 * （+定常バイアス除去用のPI面積分項）で足りる。
 *
 * 設計根拠・導入動機・「既にPIDで頑健化済み」という留保: 計画書
 * docs/plans/smc-rate-loop-plan.md §7 参照。
 *
 * @design docs/plans/smc-rate-loop-plan.md §7 -- control law derivation [--]
 * @design pid_controller.hpp PidController::setVelocityLawOverride() -- injection point [OK]
 *
 * References / 参考文献 (see smc_rate.hpp for the full citations this reuses):
 *   [R1] W. Gao and J. C. Hung, IEEE Trans. Industrial Electronics, 1993.
 *   [R2] J.-J. E. Slotine and W. Li, Applied Nonlinear Control, 1991.
 */

#pragma once

#include <cmath>

namespace sf::app {

/// Single-axis sliding-mode horizontal velocity controller
/// 1軸スライディングモード・水平速度コントローラ
struct SlidingModeVelocity {
    // Gains (reloaded from params.cpp by the caller's reloadParams()) / ゲイン
    float k        = 0;     // [m/s^2] switching/reaching gain
    float eta      = 0;     // [1/s] linear reaching gain near the surface
    float phi      = 0.1f;  // [m/s] boundary-layer half-width
    float lambda_i = 0;     // [1/s] PI-surface integral gain (0 = pure-P surface)
    float e_reset  = 0;     // [m/s] |e| threshold above which integral snaps to 0 (0 = disabled)

    // Output limit [m/s^2] -- same physical ceiling the PID velocity loop
    // uses (gravity_ * max_pos_tilt_, pid_controller.cpp), set by the caller
    // so both laws share one ceiling.
    // 出力上限 [m/s^2] -- 速度PIDと同じ物理上限（gravity_*max_pos_tilt_,
    // pid_controller.cpp）を呼び出し側が設定し、両則で上限を共有する。
    float output_limit = 1.0f;

    // Integral-of-error state [m] and its previous sample (trapezoidal
    // integration, same scheme as smc_rate.hpp).
    // 積分誤差状態 [m] と前回サンプル（台形積分、smc_rate.hppと同じ方式）。
    float integral   = 0;
    float prev_error = 0;

    // --- Dead-time predictor (opt-in), same mechanism as SlidingModeRate's
    // in smc_rate.hpp -- see that struct's field comment for the full
    // derivation and Smith-predictor [R4] citation. Tried here on the user's
    // request to check whether compensating BOTH the rate loop and this
    // velocity loop helps beyond the rate-loop-only compensation validated
    // in docs/plans/smc-rate-loop-plan.md SS7.7 (that work's own hypothesis
    // was that fixing the rate loop alone should suffice, since it is
    // upstream of this loop in the cascade -- this field exists to test
    // that hypothesis, not because it was assumed necessary).
    //
    //   vel_predicted = vel_meas + sum_{i=t-L}^{t} accel_i * dt
    //
    // No inertia division needed (unlike the rate loop's torque): the
    // output IS the acceleration itself, same reasoning as this file's
    // header note on why no inertia multiplier is needed in the reaching
    // law either.
    // --- 無駄時間予測補償器（opt-in）、smc_rate.hppのSlidingModeRateと同じ
    // 機構 -- 導出とSmith予測器[R4]の出典は同structのフィールドコメント参照。
    // ユーザーの要請により、レートループのみの補償（docs/plans/
    // smc-rate-loop-plan.md §7.7で検証済み）を超えて、この速度ループにも
    // 補償を足すと追加の効果があるかを確認するために試す（その作業自体の
    // 仮説は「レートループだけ直せば十分なはず（カスケードでこのループより
    // 上流だから）」であり、本フィールドはその仮説を検証するために存在する
    // ——必要だと決め打ちしたわけではない）。
    //
    //   vel_predicted = vel_meas + Σ_{i=t-L}^{t} accel_i * dt
    //
    // 慣性による除算は不要（レートループのトルクと異なり）: 出力そのものが
    // 加速度であるため——本ファイル冒頭の「到達則に慣性乗数が不要」という
    // 注記と同じ理由。
    static constexpr int kDelayCompMaxSamples = 16;  // covers up to 40ms @ 400Hz
    float delay_comp_s = 0;  // [s] nominal dead time to compensate (0 = disabled)
    float accel_ring_[kDelayCompMaxSamples] = {};
    int   accel_ring_idx_   = 0;
    int   accel_ring_count_ = 0;

    /// Compute the sliding-mode acceleration output / スライディングモード・加速度出力を計算
    /// @param vel_sp    Target NED velocity [m/s] / 目標NED速度
    /// @param vel_meas  Measured NED velocity [m/s] (ESKF) / 測定NED速度
    /// @param dt        Time step [s] / タイムステップ
    /// @return          Acceleration [m/s^2], clamped to +/-output_limit / 加速度、+/-output_limitにクランプ
    float compute(float vel_sp, float vel_meas, float dt)
    {
        float vel_used = vel_meas;
        if (delay_comp_s > 1.0e-6f && dt > 0) {
            int n = static_cast<int>(delay_comp_s / dt + 0.5f);
            if (n > kDelayCompMaxSamples) n = kDelayCompMaxSamples;
            if (n > accel_ring_count_)    n = accel_ring_count_;
            float sum_dv = 0.0f;
            int idx = accel_ring_idx_;
            for (int i = 0; i < n; ++i) {
                idx = (idx - 1 + kDelayCompMaxSamples) % kDelayCompMaxSamples;
                sum_dv += accel_ring_[idx] * dt;
            }
            vel_used = vel_meas + sum_dv;
        }

        const float e = vel_sp - vel_used;

        // Trapezoidal trial update of the integral state -- see
        // smc_rate.hpp's compute() for the full anti-windup rationale (this
        // is a straight unit re-derivation, layer-for-layer identical).
        // 積分状態の台形則による試験更新 -- アンチワインドアップの詳細根拠は
        // smc_rate.hppのcompute()参照（単位を変えただけの層ごと同一移植）。
        const float integral_trial = (dt > 0)
            ? integral + (e + prev_error) * (dt * 0.5f)
            : integral;

        auto reachingAccel = [this](float e_now, float integ) {
            const float s = e_now + lambda_i * integ;
            float switching = 0.0f;
            if (phi > 1.0e-6f) {
                switching = s / phi;
                if (switching >  1.0f) switching =  1.0f;
                if (switching < -1.0f) switching = -1.0f;
            }
            return k * switching + eta * s;
        };

        const float accel_trial = reachingAccel(e, integral_trial);

        // Layer 1 -- conditional integration.
        // 第1層 -- 条件付き積分。
        const bool push_high = (accel_trial >  output_limit) && (e > 0);
        const bool push_low  = (accel_trial < -output_limit) && (e < 0);
        if (!push_high && !push_low) {
            integral = integral_trial;
        }

        // Layer 2 -- backstop hard clamp on |integral|'s linear contribution.
        // 第2層 -- |integral|の線形寄与分へのバックストップ・ハードクランプ。
        const float linear_gain = eta * lambda_i;
        if (linear_gain > 1.0e-9f) {
            const float integral_max = output_limit / linear_gain;
            if (integral >  integral_max) integral =  integral_max;
            if (integral < -integral_max) integral = -integral_max;
        }

        // Layer 3 -- large-error gate reset.
        // 第3層 -- 偏差ゲートによるリセット。
        if (e_reset > 1.0e-6f && fabsf(e) > e_reset) {
            integral = 0;
        }

        prev_error = e;

        float accel = reachingAccel(e, integral);
        if (accel >  output_limit) accel =  output_limit;
        if (accel < -output_limit) accel = -output_limit;

        // Record this cycle's (already-clamped) accel for the dead-time
        // predictor's ring buffer -- see SlidingModeRate::compute() in
        // smc_rate.hpp for the identical rationale.
        // このサイクルの加速度を無駄時間予測補償器のring bufferへ記録する
        // -- 同じ根拠はsmc_rate.hppのSlidingModeRate::compute()参照。
        accel_ring_[accel_ring_idx_] = accel;
        accel_ring_idx_ = (accel_ring_idx_ + 1) % kDelayCompMaxSamples;
        if (accel_ring_count_ < kDelayCompMaxSamples) ++accel_ring_count_;

        return accel;
    }

    /// Reset internal state / 内部状態をリセット
    void reset()
    {
        integral   = 0;
        prev_error = 0;
        for (float& a : accel_ring_) a = 0.0f;
        accel_ring_idx_   = 0;
        accel_ring_count_ = 0;
    }
};

}  // namespace sf::app
