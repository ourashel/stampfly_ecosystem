/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file smc_rate_sta.hpp
 * @brief Single-axis SECOND-ORDER sliding-mode rate controller: the
 *        Super-Twisting Algorithm (STA) [R5][R6], tried as an alternative
 *        to firmware/apps/smc_rate's first-order PI-type-surface design
 *        (smc_rate.hpp's SlidingModeRate) for the still-unresolved
 *        motor-delay=15ms tilt_max FAIL (docs/plans/smc-rate-loop-plan.md
 *        section 3.3 onward -- neither gain re-tuning, a dead-time
 *        predictor, nor a sliding-surface dead-band fixed it; both PID
 *        (23.65deg) and the first-order SMC (24.06deg) hit almost the same
 *        ceiling under this condition, section 7.11's motivating
 *        observation).
 *        1軸2次スライディングモード・レートコントローラ:
 *        スーパーツイスティング法（STA）[R5][R6]。firmware/apps/smc_rateの
 *        1次PI型面設計（smc_rate.hppのSlidingModeRate）の代替として、
 *        §3.3以降未解決のmotor-delay=15msでのtilt_max FAILに対して試す
 *        （docs/plans/smc-rate-loop-plan.md §3.3以降 -- ゲイン再探索・
 *        無駄時間予測補償器・スライディング面不感バンドのいずれも解決
 *        せず、PID(23.65°)と1次SMC(24.06°)がほぼ同じ天井にぶつかっている
 *        という§7.11の着眼点）。
 *
 * Control law (independent per axis):
 *
 *   e = rate_sp - rate_meas
 *   s = e + lambda_i * integral(e dt)     (same PI-type surface as
 *                                           smc_rate.hpp, for steady-bias
 *                                           rejection under torque-authority
 *                                           perturbation; lambda_i=0
 *                                           recovers the textbook STA on s=e)
 *   u1 = k1 * sqrt(|s|) * sign(s)          (continuous "proportional" term --
 *                                           vanishes continuously as s->0,
 *                                           unlike a bare sign(s)/sat(s/phi)
 *                                           switching term)
 *   z_dot = -k2 * sign(s)                  (integral term, discrete: z +=
 *                                           -k2*sign(s)*dt)
 *   torque = I_axis * (u1 + z)
 *
 * Why this might help where the first-order design (k*sat(s/phi) + eta*s)
 * did not: the boundary-layer design's linear-near-origin term (eta*s)
 * requires a LARGE eta to reject torque-authority disturbance (section 3.3's
 * finding), and that same large eta is what destabilizes under motor-delay
 * (section 3.3's "eta increase fixes torque-authority but tumbles under
 * motor-delay=15ms" trade-off). STA's u1 term is nonlinear (sqrt), so its
 * gain near s=0 is HIGH (steep initial slope, d(u1)/ds -> infinity as s->0)
 * while its gain far from the origin GROWS SLOWER than a large linear eta
 * would -- potentially decoupling "fast disturbance rejection near the
 * surface" from "aggressive linear gain everywhere" in a way smc_rate.hpp's
 * single scalar eta cannot. This is a hypothesis to test numerically (see
 * docs/plans/smc-rate-loop-plan.md section 7.11), not a guarantee -- STA's
 * own stability proofs [R5][R6] assume a Lipschitz-bounded matched
 * disturbance derivative, not an explicit actuation delay; delay margin is
 * not what STA was designed to solve, only tried here because the
 * mechanism (continuous control, no artificial boundary layer) is
 * structurally different from what has already failed.
 *
 * なぜ1次設計（k*sat(s/phi) + eta*s）で効かなかった条件に効くかもしれないか:
 * 境界層設計の原点近傍の線形項（eta*s）はtorque-authority外乱を抑えるのに
 * 大きなetaを要求し（§3.3の知見）、その同じ大きなetaがmotor-delay=15ms下で
 * 不安定化させる（§3.3の「eta増でtorque-authorityは直るがmotor-delayで
 * 転倒」というトレードオフ）。STAのu1項は非線形（平方根）で、原点近傍の
 * ゲインは高く（初期勾配が急、s->0でd(u1)/ds->∞）、原点から離れるとその
 * ゲインの伸びは大きな線形etaほど急でない——smc_rate.hppの単一スカラーeta
 * ではできない形で「面近傍での速い外乱抑制」と「全域での積極的な線形
 * ゲイン」を切り離せる可能性がある。これは数値的に検証すべき仮説であり
 * （docs/plans/smc-rate-loop-plan.md §7.11参照）、保証ではない——STA自身の
 * 安定性証明[R5][R6]はリプシッツ有界な整合外乱の導関数を仮定しており、
 * 実際のアクチュエータ無駄時間を解くために設計されたものではない。ここで
 * 試すのは、機構（連続制御、人為的境界層なし）がこれまで失敗した設計とは
 * 構造的に異なるという理由による。
 *
 * Anti-windup: the z (STA integral) state is bounded the same 3-layer way
 * smc_rate.hpp's PI-surface integral is -- conditional integration,
 * backstop clamp, large-error gate reset -- reusing that file's proven
 * design rather than inventing a new one.
 * アンチワインドアップ: z（STA積分）状態はsmc_rate.hppのPI面積分と同じ
 * 3層（条件付き積分・バックストップクランプ・偏差ゲートリセット）で
 * 束縛する -- 新規に考案せず、実績のある設計を流用。
 *
 * A small smoothing width phi is applied to sign(s) (both in u1 and the
 * z update) -- sat(s/phi) instead of a bare +/-1 -- purely to avoid
 * discrete-time (400Hz) numerical chattering at s=0; unlike smc_rate.hpp's
 * boundary layer, phi here is NOT the primary chattering-mitigation
 * mechanism (u1's sqrt(|s|) factor already vanishes at s=0) -- it only
 * smooths the residual corner.
 * sign(s)（u1とz更新の両方）には小さな平滑化幅phiを適用する（生の±1でなく
 * sat(s/phi)）——純粋に離散時間（400Hz）でのs=0における数値的チャタリング
 * を避けるため。smc_rate.hppの境界層と異なり、phiはここでは主要な
 * チャタリング抑制機構ではない（u1のsqrt(|s|)項が既にs=0で消えるため）
 * ——残る角を丸めるだけ。
 *
 * @design docs/plans/smc-rate-loop-plan.md section 7.11 -- STA trial for motor-delay [--]
 * @design controller.hpp -- IController interface (used via AppController)  [OK]
 *
 * References / 参考文献:
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

/// Single-axis super-twisting (2nd-order sliding-mode) rate controller
/// 1軸スーパーツイスティング（2次スライディングモード）レートコントローラ
struct SuperTwistingRate {
    // Physical moment of inertia for this axis [kg*m^2] -- same role as
    // smc_rate.hpp's SlidingModeRate::inertia.
    // この軸の物理慣性モーメント [kg*m^2] -- smc_rate.hppのSlidingModeRate::
    // inertiaと同じ役割。
    float inertia = 1.0f;   // [kg*m^2]

    // Gains (reloaded from params.cpp by the caller's reloadParams()) / ゲイン
    float k1       = 0;     // [rad/s^2 per sqrt(rad/s)] continuous-term gain
    float k2       = 0;     // [rad/s^3] integral-term gain
    float phi      = 0.02f; // [rad/s] sign() smoothing width (numerical only, NOT the primary chattering fix -- see file header)
    float lambda_i = 0;     // [1/s] PI-surface integral gain on s itself (0 = textbook STA on s=e)
    float e_reset  = 0;     // [rad/s] |e| threshold above which BOTH integral states snap to 0 (0 = disabled)
    float z_leak_tau = 0;   // [s] leaky-integration time constant for z (0 = disabled, pure integration); see the z_trial comment in compute() for the derivation (docs/plans/smc-rate-loop-plan.md §7.18)

    // Output limit [Nm] -- same physical torque ceiling smc_rate.hpp's
    // SlidingModeRate uses.
    // 出力上限 [Nm] -- smc_rate.hppのSlidingModeRateと同じ物理トルク上限。
    float output_limit = 1.0f;

    // Two independent integral-like states, both reset the same way:
    //   integral -- the PI-surface integral of e (same role as
    //               SlidingModeRate::integral)
    //   z        -- the STA's own integral term (the "u1" auxiliary state
    //               in the control-law comment above)
    // 2つの独立した積分状態、どちらも同じ方式でリセットされる:
    //   integral -- PI面のeの積分（SlidingModeRate::integralと同じ役割）
    //   z        -- STA自身の積分項（上の制御則コメントの"u1"補助状態）
    float integral   = 0;
    float z          = 0;
    float prev_error = 0;

    /// Compute the super-twisting torque output / スーパーツイスティング・トルク出力を計算
    /// @param rate_sp    Target angular rate [rad/s] / 目標角速度
    /// @param rate_meas  Measured angular rate [rad/s] (bias-corrected gyro) / 測定角速度（バイアス補正済ジャイロ）
    /// @param dt         Time step [s] / タイムステップ
    /// @return           Torque [Nm], clamped to +/-output_limit / トルク [Nm]、+/-output_limit にクランプ
    float compute(float rate_sp, float rate_meas, float dt)
    {
        const float e = rate_sp - rate_meas;

        // Trapezoidal trial update of the PI-surface integral (mirrors
        // smc_rate.hpp's SlidingModeRate::compute()).
        // PI面積分の台形則による試験更新（smc_rate.hppのSlidingModeRate::
        // compute()を踏襲）。
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

        // Trial STA torque at the trial integral states -- "trial" because
        // anti-windup below may reject the update, same pattern as
        // smc_rate.hpp.
        // 試験積分状態でのSTAトルク（試験更新） -- 「試験」なのはこの後の
        // アンチワインドアップが棄却しうるため、smc_rate.hppと同じ方式。
        auto staOutput = [this, &smoothSign](float e_now, float integ, float z_now) {
            const float s = e_now + lambda_i * integ;
            const float sign_s = smoothSign(s);
            const float u1 = k1 * sqrtf(fabsf(s)) * sign_s;
            return inertia * (u1 + z_now);
        };

        const float s_trial = e + lambda_i * integral_trial;
        const float sign_s_trial = smoothSign(s_trial);
        // Leaky integration of z (opt-in safety mechanism, docs/plans/
        // smc-rate-loop-plan.md §7.18, added after §7.17's SILS finding: a
        // SUSTAINED same-sign disturbance made z grow linearly and
        // unboundedly -- up to ~150-180 before the FIRST e_reset fired --
        // while u1's sqrt(|s|) term grew to nearly CANCEL z (net torque
        // stayed small even as z was enormous), so the rate loop never
        // actually rejected the disturbance; after 3-4 such reset cycles
        // (~26 s) the craft tumbled. A leaky integrator (a standard anti-
        // windup technique -- cf. PID "integral leakage"/"false
        // integration") adds a decay term -z/z_leak_tau, so z settles to a
        // BOUNDED equilibrium (|z_eq| = k2*z_leak_tau) under a sustained
        // disturbance instead of growing without limit, without needing a
        // guessed hard clamp. z_leak_tau=0 (this file's prior behavior)
        // disables the leak entirely -- pure integration, unchanged unless
        // an app explicitly sets this parameter.
        // zの漏れ積分（opt-inの安全機構、docs/plans/smc-rate-loop-plan.md
        // §7.18、§7.17のSILS実測を受けて追加: 持続的な同符号外乱下でzが
        // 歯止めなく線形成長し——最初のe_resetが発火するまでに~150-180に
        // 達し——その間u1の√|s|項がzをほぼ相殺してしまう（zが巨大でも正味
        // トルクは終始小さいまま）ため、レートループが実質的に外乱を
        // 抑えられていなかった。約3-4回のリセットサイクル（~26秒）の後、
        // 機体は転倒した。漏れ積分（標準的なアンチワインドアップ手法 --
        // PIDの「積分リーク」/「偽積分」に相当）は減衰項-z/z_leak_tauを
        // 加えることで、持続外乱下でも歯止めなく成長する代わりに
        // 有界な平衡値（|z_eq|=k2*z_leak_tau）へ収束させる——恣意的な
        // ハードクランプの推測が不要。z_leak_tau=0（本ファイルの従来動作）
        // で漏れを完全無効化——appが明示的にこのパラメータを設定しない限り
        // 純粋な積分のまま、挙動は不変。
        const float z_trial = (dt > 0)
            ? z + (-k2 * sign_s_trial - (z_leak_tau > 1.0e-6f ? z / z_leak_tau : 0.0f)) * dt
            : z;

        const float torque_trial = staOutput(e, integral_trial, z_trial);

        // Conditional-integration anti-windup, applied to BOTH integral
        // states together (same trigger as smc_rate.hpp: would the trial
        // torque push an already-saturated output further into
        // saturation?).
        // 条件付き積分アンチワインドアップ、両方の積分状態に同時適用
        // （smc_rate.hppと同じトリガ: 試験トルクが既に飽和した出力を
        // さらに飽和方向へ押すか）。
        const bool push_high = (torque_trial >  output_limit) && (e > 0);
        const bool push_low  = (torque_trial < -output_limit) && (e < 0);
        if (!push_high && !push_low) {
            integral = integral_trial;
            z        = z_trial;
        }

        // Backstop hard clamp on |integral|'s linear contribution -- same
        // form as smc_rate.hpp. z is bounded separately: since z_dot is
        // +/-k2 (bounded by construction), z itself is clamped directly to
        // +/-output_limit/inertia (its own maximum possible torque
        // contribution).
        // |integral|の線形寄与分へのバックストップ・ハードクランプ --
        // smc_rate.hppと同形。zは別途束縛する: z_dotは+/-k2で有界（構造上）
        // なので、z自体を+/-output_limit/inertia（zが持ちうる最大トルク
        // 寄与）へ直接クランプする。
        const float linear_gain = inertia * lambda_i;  // integral's contribution enters via s, scaled implicitly through k1/k2's use of s -- see note below
        (void)linear_gain;  // the STA reaching law is nonlinear in s, so there is no single closed-form "linear_gain" the way SlidingModeRate has (k/phi+eta); z is bounded directly instead (see below)
        if (inertia > 1.0e-12f) {
            const float z_max = output_limit / inertia;
            if (z >  z_max) z =  z_max;
            if (z < -z_max) z = -z_max;
        }

        // Large-error gate reset -- same rationale as smc_rate.hpp, applied
        // to both integral states.
        // 偏差ゲートによるリセット -- smc_rate.hppと同じ根拠、両積分状態に適用。
        if (e_reset > 1.0e-6f && fabsf(e) > e_reset) {
            integral = 0;
            z        = 0;
        }

        prev_error = e;

        float torque = staOutput(e, integral, z);
        if (torque >  output_limit) torque =  output_limit;
        if (torque < -output_limit) torque = -output_limit;
        return torque;
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
