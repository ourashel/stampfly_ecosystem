/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sliding_mode_sta.hpp
 * @brief GENERIC single-axis SECOND-ORDER sliding-mode controller: the
 *        Super-Twisting Algorithm (STA) [R5][R6], unified across the RATE
 *        loop (torque output) and the horizontal VELOCITY loop (acceleration
 *        output) into a single struct.
 *        汎用1軸2次スライディングモード制御器: スーパーツイスティング法
 *        （STA）[R5][R6]を、レートループ（トルク出力）と水平速度ループ
 *        （加速度出力）の両方で1つの構造体に統合したもの。
 *
 * Background / 背景:
 * firmware/apps/smc_rate_sta/smc_rate_sta.hpp's SuperTwistingRate computes
 * torque = inertia*(u1+z); a first draft of this app's velocity-loop
 * controller (SuperTwistingVelocity) computed accel = u1+z (no multiplier,
 * since acceleration IS the physical output quantity, unlike torque which
 * needs the inertia scale). These two structs differ ONLY in whether that
 * output multiplier exists and what it's called -- otherwise byte-identical
 * control law and anti-windup logic. Per a mid-session design review
 * ("SMC提供クラスを用意する方がいいのでは？" -- "wouldn't it be better to
 * have a class that provides SMC?"), the two are unified here into one
 * `SuperTwisting` struct with a generic `output_scale` field: `inertia`
 * [kg*m^2] for a rate axis, `1.0` (the struct's own default) for a velocity
 * axis. This keeps the control law in exactly one place inside this app
 * (5 instances -- 3 rate axes + 2 velocity axes -- all built from the same
 * struct), instead of two near-duplicate copies that could drift apart.
 *
 * Scope note: this unification is local to smc_pos_sta only.
 * firmware/apps/smc_rate_sta itself (already real-hardware-flashed,
 * independently validated) is left untouched -- no shared cross-app
 * component was introduced, matching the project's "an app is
 * self-contained" convention (coding_and_education.md).
 *
 * firmware/apps/smc_rate_sta/smc_rate_sta.hppのSuperTwistingRateは
 * torque = inertia*(u1+z) を計算する。本appの速度ループ制御器の初期案
 * （SuperTwistingVelocity）は accel = u1+z （乗数なし——加速度は物理量
 * そのものであり、慣性でスケールすべきトルクとは異なる）を計算していた。
 * この2つは出力乗数の有無・呼び名以外はバイト単位で同一の制御則・
 * アンチワインドアップロジックだった。セッション途中の設計レビュー
 * （「SMC提供クラスを用意する方がいいのでは？」）を受け、汎用フィールド
 * `output_scale`を持つ1つの`SuperTwisting`構造体へ統合する: レート軸には
 * `inertia` [kg*m^2]、速度軸には`1.0`（構造体自身の既定値）。これにより
 * 制御則が本app内の1箇所だけに存在する（5インスタンス——レート3軸+速度2軸
 * ——全て同じ構造体から生成）、かい離しうる2つのほぼ重複したコピーを
 * 避けられる。
 *
 * 範囲についての注記: この統合は`smc_pos_sta`内だけに限定する。
 * firmware/apps/smc_rate_sta自体（既に実機投入済み・独立検証済み）は
 * 無改造のまま維持する——app間で共有するコンポーネントは導入せず、
 * プロジェクトの「appは自己完結」規約（coding_and_education.md）と整合する。
 *
 * Control law (independent per axis, identical to smc_rate_sta.hpp's
 * SuperTwistingRate with `inertia` generalized to `output_scale`):
 *
 *   e = sp - meas                          (rate axis: rad/s; velocity axis: m/s)
 *   s = e + lambda_i * integral(e dt)      (PI-type surface)
 *   u1 = k1 * sqrt(|s|) * sign(s)          (continuous "proportional" term)
 *   z_dot = -k2 * sign(s) - z/z_leak_tau   (leaky integral term; z_leak_tau=0
 *                                           disables the leak, pure integration)
 *   output = output_scale * (u1 + z)       (rate axis: torque [Nm];
 *                                           velocity axis: accel [m/s^2])
 *
 * See smc_rate_sta.hpp for the full design rationale (why STA vs. the
 * 1st-order boundary-layer design, the leaky-integration safety mechanism
 * and its docs/plans/smc-rate-loop-plan.md §7.17/§7.18 origin, and why phi
 * here is a numerical-smoothing width, not a chattering-mitigation boundary
 * layer). This file only generalizes the output scaling; every other design
 * decision is a direct, unmodified port.
 *
 * z_leak_tau defaults to NONZERO here (unlike this struct's other opt-in
 * fields) for the same reason smc_rate_sta.hpp's does: z_leak_tau=0 (pure
 * integration) is exactly the behavior that caused a real tumble in SILS
 * (docs/plans/smc-rate-loop-plan.md §7.17) -- the caller (params.cpp) sets
 * a nonzero default for both rate AND velocity axes from the start, rather
 * than adding the leak only after observing a failure the way smc_rate_sta
 * did historically.
 *
 * No dead-time predictor: docs/plans/smc-rate-loop-plan.md §7.9 shelved the
 * original velocity-loop SMC trial partly because its dead-time predictor
 * had too narrow a safety margin to be usable -- this struct deliberately
 * omits that mechanism so the STA reaching law's own merit (vs. PID) can be
 * evaluated on its own, without reintroducing a known-risky feature.
 *
 * @design docs/plans/smc-rate-loop-plan.md §7 (velocity-loop SMC) + §7.11-7.24 (STA) [--]
 * @design controller.hpp -- IController interface (used via AppController)  [OK]
 * @design pid_controller.hpp PidController::setVelocityLawOverride() -- injection point [OK]
 *
 * References / 参考文献 (see smc_rate_sta.hpp for the full citations):
 *   [R5] A. Levant, "Sliding order and sliding accuracy in sliding mode
 *        control," International Journal of Control, vol. 58, no. 6,
 *        pp. 1247-1263, 1993.
 *   [R6] J. A. Moreno and M. Osorio, "Strict Lyapunov functions for the
 *        super-twisting algorithm," IEEE Trans. Automatic Control, vol. 57,
 *        no. 4, pp. 1035-1040, 2012. (practical gain-tuning conditions)
 */

#pragma once

#include <cmath>

namespace sf::app {

/// Generic single-axis super-twisting (2nd-order sliding-mode) controller,
/// usable for both a rate axis (output_scale = inertia, output = torque)
/// and a horizontal-velocity axis (output_scale = 1.0, output = acceleration).
/// 汎用1軸スーパーツイスティング（2次スライディングモード）制御器。
/// レート軸（output_scale=慣性、出力=トルク）・水平速度軸
/// （output_scale=1.0、出力=加速度）の両方に使える。
struct SuperTwisting {
    // Output scaling multiplier. Rate axis: set to the axis's physical
    // moment of inertia [kg*m^2] so output = torque [Nm]. Velocity axis:
    // leave at the default 1.0 so output = acceleration [m/s^2] directly
    // (acceleration IS the physical quantity being commanded, same
    // reasoning smc_vel.hpp's header used for why no multiplier is needed).
    // 出力スケール乗数。レート軸: 軸の物理慣性モーメント[kg*m^2]を設定し
    // 出力=トルク[Nm]とする。速度軸: 既定の1.0のままにし出力=加速度
    // [m/s^2]そのものとする（加速度は物理量そのものであり、
    // smc_vel.hppのヘッダが「乗数不要」とした理由と同じ）。
    float output_scale = 1.0f;

    // Gains (reloaded from params.cpp by the caller's reloadParams()) / ゲイン
    float k1       = 0;     // continuous-term gain [rate axis: rad/s^2 per sqrt(rad/s); velocity axis: m/s^2 per sqrt(m/s)]
    float k2       = 0;     // integral-term gain [rate axis: rad/s^3; velocity axis: m/s^3]
    float phi      = 0.02f; // sign() smoothing width (numerical only, NOT the primary chattering fix -- see smc_rate_sta.hpp's file header)
    float lambda_i = 0;     // PI-surface integral gain on s itself (0 = textbook STA on s=e)
    float e_reset  = 0;     // |e| threshold above which BOTH integral states snap to 0 (0 = disabled)
    float z_leak_tau = 0;   // [s] leaky-integration time constant for z (0 = disabled, pure integration); see smc_rate_sta.hpp's z_trial comment for the |z_eq|=k2*z_leak_tau derivation

    // Output limit [rate axis: Nm; velocity axis: m/s^2]
    // 出力上限 [レート軸: Nm; 速度軸: m/s^2]
    float output_limit = 1.0f;

    // Two independent integral-like states, both reset the same way:
    //   integral -- the PI-surface integral of e
    //   z        -- the STA's own integral term
    // 2つの独立した積分状態、どちらも同じ方式でリセットされる:
    //   integral -- PI面のeの積分
    //   z        -- STA自身の積分項
    float integral   = 0;
    float z          = 0;
    float prev_error = 0;

    /// Compute the super-twisting output / スーパーツイスティング出力を計算
    /// @param sp    Target value [rate axis: rad/s; velocity axis: m/s] / 目標値
    /// @param meas  Measured value [same units as sp] / 測定値
    /// @param dt    Time step [s] / タイムステップ
    /// @return      Output, clamped to +/-output_limit [rate axis: Nm; velocity axis: m/s^2]
    float compute(float sp, float meas, float dt)
    {
        const float e = sp - meas;

        // Trapezoidal trial update of the PI-surface integral (same scheme
        // as smc_rate_sta.hpp's SuperTwistingRate::compute()).
        // PI面積分の台形則による試験更新（smc_rate_sta.hppの
        // SuperTwistingRate::compute()と同じ方式）。
        const float integral_trial = (dt > 0)
            ? integral + (e + prev_error) * (dt * 0.5f)
            : integral;

        auto smoothSign = [this](float x) {
            if (phi <= 1.0e-6f) return (x > 0) ? 1.0f : (x < 0 ? -1.0f : 0.0f);
            float v = x / phi;
            if (v >  1.0f) v =  1.0f;
            if (v < -1.0f) v = -1.0f;
            return v;
        };

        // Trial STA output at the trial integral states -- "trial" because
        // anti-windup below may reject the update.
        // 試験積分状態でのSTA出力（試験更新） -- 「試験」なのはこの後の
        // アンチワインドアップが棄却しうるため。
        auto staOutput = [this, &smoothSign](float e_now, float integ, float z_now) {
            const float s = e_now + lambda_i * integ;
            const float sign_s = smoothSign(s);
            const float u1 = k1 * sqrtf(fabsf(s)) * sign_s;
            return output_scale * (u1 + z_now);
        };

        const float s_trial = e + lambda_i * integral_trial;
        const float sign_s_trial = smoothSign(s_trial);

        // Leaky integration of z -- see smc_rate_sta.hpp's z_trial comment
        // for the full derivation (docs/plans/smc-rate-loop-plan.md §7.18)
        // of why this exists and the |z_eq|=k2*z_leak_tau bound.
        // zの漏れ積分 -- 存在理由と|z_eq|=k2*z_leak_tauの境界の導出は
        // smc_rate_sta.hppのz_trialコメント参照（docs/plans/
        // smc-rate-loop-plan.md §7.18）。
        const float z_trial = (dt > 0)
            ? z + (-k2 * sign_s_trial - (z_leak_tau > 1.0e-6f ? z / z_leak_tau : 0.0f)) * dt
            : z;

        const float output_trial = staOutput(e, integral_trial, z_trial);

        // Conditional-integration anti-windup, applied to BOTH integral
        // states together (same trigger as smc_rate_sta.hpp: would the
        // trial output push an already-saturated output further into
        // saturation?).
        // 条件付き積分アンチワインドアップ、両方の積分状態に同時適用
        // （smc_rate_sta.hppと同じトリガ）。
        const bool push_high = (output_trial >  output_limit) && (e > 0);
        const bool push_low  = (output_trial < -output_limit) && (e < 0);
        if (!push_high && !push_low) {
            integral = integral_trial;
            z        = z_trial;
        }

        // Backstop hard clamp on z: since z_dot is +/-k2 (bounded by
        // construction), z itself is clamped directly to its own maximum
        // possible output contribution.
        // zへのバックストップ・ハードクランプ: z_dotは+/-k2で有界
        // （構造上）なので、z自体をその出力への最大寄与へ直接クランプする。
        if (fabsf(output_scale) > 1.0e-12f) {
            const float z_max = output_limit / output_scale;
            if (z >  z_max) z =  z_max;
            if (z < -z_max) z = -z_max;
        }

        // Large-error gate reset -- same rationale as smc_rate_sta.hpp,
        // applied to both integral states.
        // 偏差ゲートによるリセット -- smc_rate_sta.hppと同じ根拠、
        // 両積分状態に適用。
        if (e_reset > 1.0e-6f && fabsf(e) > e_reset) {
            integral = 0;
            z        = 0;
        }

        prev_error = e;

        float output = staOutput(e, integral, z);
        if (output >  output_limit) output =  output_limit;
        if (output < -output_limit) output = -output_limit;
        return output;
    }

    /// Reset internal state / 内部状態をリセット
    void reset()
    {
        integral   = 0;
        z          = 0;
        prev_error = 0;
    }
};

}  // namespace sf::app
