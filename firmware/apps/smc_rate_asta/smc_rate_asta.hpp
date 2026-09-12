/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file smc_rate_asta.hpp
 * @brief Single-axis SECOND-ORDER sliding-mode rate controller with an
 *        ADAPTIVE switching gain: the same Super-Twisting Algorithm (STA)
 *        as firmware/apps/smc_rate_sta's SuperTwistingRate, but k1 (and k2,
 *        tied to it) adapts online [R7] instead of being a fixed constant.
 *        1軸2次スライディングモード・レートコントローラ（適応スイッチング
 *        ゲイン版）: firmware/apps/smc_rate_staのSuperTwistingRateと同じ
 *        Super-Twisting法（STA）だが、k1（およびそれに連動するk2）は固定
 *        定数ではなくオンラインで適応する[R7]。
 *
 * Why a SEPARATE app instead of extending smc_rate_sta: user's explicit
 * instruction (session 2026-09-12) -- smc_rate_sta is the FINAL fixed-gain
 * version and must not change; the adaptive variant is a new, independent
 * experiment.
 * 既存smc_rate_staを拡張せず別appにする理由: ユーザーの明示的指示
 * （セッション2026-09-12）——smc_rate_staは固定ゲイン版の最終バージョンで
 * あり変更しない。適応版は新規・独立の実験とする。
 *
 * Motivation: docs/plans/smc-rate-loop-plan.md section 3.7 found a
 * NON-MONOTONIC trade-off tuning smc_rate_sta's fixed lambda_i/eta-like
 * gains -- raising them to reject torque-authority=0.4/0.55 disturbance
 * caused a NEW tumble under motor-delay=15ms that a smaller gain did not.
 * A single fixed switching gain cannot satisfy both an aggressive-disturbance
 * condition and a fast-delay condition at once when their required gains
 * conflict. An ADAPTIVE gain -- one that grows only while the sliding
 * variable s is not actually converging (i.e., while the current gain is
 * insufficient for whatever disturbance is present NOW), and decays slowly
 * otherwise -- can in principle track a moving target the fixed-gain
 * trade-off could not, without requiring an a priori worst-case bound.
 * This is a hypothesis to verify numerically (docs/plans/smc-rate-loop-
 * plan.md section 7.31), not a guarantee.
 * 動機: docs/plans/smc-rate-loop-plan.md §3.7で、smc_rate_staの固定
 * lambda_i/eta的ゲインの調整に**非単調な**トレードオフが見つかった——
 * torque-authority=0.4/0.55の外乱を抑えるためゲインを上げると、より小さい
 * ゲインでは起きなかったmotor-delay=15msでの転倒という新しい問題が生じた。
 * 単一の固定スイッチングゲインでは、要求されるゲインが矛盾する「強い外乱」
 * と「速い遅延」の両条件を同時に満たせない。**適応ゲイン**——スライディング
 * 変数sが実際に収束できていない間（＝現在のゲインが「今」存在する外乱に
 * 対して不十分な間）だけ増加し、それ以外は緩やかに減衰するもの——は、
 * 事前の最悪値見積りを必要とせず、固定ゲインでは追えなかった動く標的を
 * 原理的に追従できる可能性がある。これは数値的に検証すべき仮説であり
 * （docs/plans/smc-rate-loop-plan.md §7.31）、保証ではない。
 *
 * Control law (independent per axis):
 *
 *   e = rate_sp - rate_meas
 *   s = e + lambda_i * integral(e dt)        (identical PI-type surface to
 *                                              smc_rate_sta.hpp)
 *   u1 = k1 * sqrt(|s|) * sign(s)             (k1 now TIME-VARYING, see below)
 *   z_dot = -k2 * sign(s)                     (k2 = k2_ratio * k1, tied to k1)
 *   torque = I_axis * (u1 + z)
 *
 *   Adaptive law for k1 (Plestan-type [R7], simplified to a single gain):
 *     if |s| > dead_band:  k1_dot = +adapt_rate               (not converged -> grow)
 *     else:                k1_dot = -adapt_rate * leak_ratio  (converged -> decay slowly)
 *     k1 = clamp(k1, k1_min, k1_max)
 *     k2 = k2_ratio * k1
 *
 * dead_band exists so sensor/discretization noise near s=0 does not drive
 * k1 to grow forever (the exact failure mode this session's SILS-based
 * ESKF investigation found for an UNRELATED accel-comp filter under
 * sustained small residuals -- docs/plans/smc-rate-loop-plan.md section
 * 7.30続報6/8 -- applied here as a design precaution, not because that
 * finding is about this controller). leak_ratio < 1 makes the gain decay
 * slower than it grows, so a brief noise spike does not immediately erase
 * gain built up for a real disturbance.
 * dead_bandは、s=0近傍のセンサ/離散化ノイズがk1を際限なく成長させ続ける
 * ことを防ぐ（今回のセッションのSILSベースESKF調査で、無関係なaccel-comp
 * フィルタが持続的な小さい残差下で同様の失敗モードを示した——docs/plans/
 * smc-rate-loop-plan.md §7.30続報6/8——が、それを踏まえた設計上の予防措置
 * であり、その発見がこのコントローラ自体に関するものというわけではない）。
 * leak_ratio<1により、ゲインの減衰は成長より緩やかにし、一時的なノイズの
 * スパイクで実外乱に対して積み上げたゲインが即座に消えないようにする。
 *
 * k2 is tied to k1 by a fixed ratio (k2 = k2_ratio * k1) rather than
 * adapted independently, keeping this a ONE-degree-of-freedom adaptation
 * (matching smc_rate_sta.hpp's own design economy -- one nonlinear gain,
 * not two independent adaptive states) and preserving STA's textbook
 * finite-time convergence relation between k1 and k2 (k1^2 > 4*k2,
 * approximately, per [R5][R6]) as k1 moves.
 * k2はk1と独立に適応させず、固定比率（k2=k2_ratio*k1）で連動させる——
 * smc_rate_sta.hpp自身の設計の簡潔さ（非線形ゲイン1つ、独立な適応状態を
 * 2つ持たない）を踏襲し、k1が動いてもSTAの教科書的な有限時間収束関係
 * （k1^2>4*k2程度、[R5][R6]）を保つため。
 *
 * Everything else (PI-surface integral, z's leaky integration, the 3-layer
 * anti-windup, phi smoothing) is UNCHANGED from smc_rate_sta.hpp -- reusing
 * that proven design rather than inventing a new one. See smc_rate_sta.hpp
 * for the rationale behind each of those mechanisms.
 * それ以外（PI面積分、zの漏れ積分、3層アンチワインドアップ、phi平滑化）は
 * smc_rate_sta.hppから変更なし——実績のある設計を流用し新規に考案しない。
 * 各機構の根拠はsmc_rate_sta.hpp参照。
 *
 * @design docs/plans/smc-rate-loop-plan.md section 7.31 -- adaptive STA trial [--]
 * @design controller.hpp -- IController interface (used via AppController)  [OK]
 *
 * References / 参考文献:
 *   [R5] A. Levant, "Sliding order and sliding accuracy in sliding mode
 *        control," International Journal of Control, vol. 58, no. 6,
 *        pp. 1247-1263, 1993.
 *   [R6] J. A. Moreno and M. Osorio, "Strict Lyapunov functions for the
 *        super-twisting algorithm," IEEE Trans. Automatic Control, vol. 57,
 *        no. 4, pp. 1035-1040, 2012. (practical gain-tuning conditions)
 *   [R7] F. Plestan, Y. Shtessel, V. Bregeault, and A. Poznyak, "New
 *        methodologies for adaptive sliding mode control," International
 *        Journal of Control, vol. 83, no. 9, pp. 1907-1919, 2010.
 */

#pragma once

#include <cmath>

namespace sf::app {

/// Single-axis super-twisting (2nd-order sliding-mode) rate controller with
/// an adaptive switching gain (k1, with k2 tied to it by a fixed ratio).
/// 1軸スーパーツイスティング（2次スライディングモード）レートコントローラ、
/// 適応スイッチングゲイン版（k1が適応、k2は固定比率で連動）。
struct AdaptiveSuperTwistingRate {
    // Physical moment of inertia for this axis [kg*m^2] -- same role as
    // smc_rate_sta.hpp's SuperTwistingRate::inertia.
    // この軸の物理慣性モーメント [kg*m^2] -- SuperTwistingRate::inertiaと同じ役割。
    float inertia = 1.0f;   // [kg*m^2]

    // --- Adaptive-gain parameters (new vs. smc_rate_sta.hpp) / 適応ゲインパラメータ（smc_rate_sta.hppとの差分） ---
    float k1_init    = 30.0f;  // [rad/s^2 per sqrt(rad/s)] k1's initial value at reset()
    float k1_min     = 15.0f;  // lower clamp on k1 (never adapt below this -- keeps some baseline authority)
    float k1_max     = 150.0f; // upper clamp on k1 (hard safety ceiling -- see file header)
    float k2_ratio   = 0.5f;   // k2 = k2_ratio * k1 (fixed ratio, not independently adapted)
    float adapt_rate = 20.0f;  // [same units as k1, per second] growth rate of k1 while the FILTERED |s| > dead_band
    float leak_ratio = 0.2f;   // decay rate while the filtered |s| <= dead_band, as a fraction of adapt_rate (< 1: decay slower than growth)
    float dead_band  = 0.05f;  // [rad/s] filtered-|s| threshold below which k1 is considered "converged" and decays
    float filter_tau = 0.05f;  // [s] low-pass time constant on |s| BEFORE the dead-band comparison -- see compute()'s rationale comment

    // --- Same-as-smc_rate_sta.hpp parameters / smc_rate_sta.hppと同じパラメータ ---
    float phi      = 0.02f; // [rad/s] sign() smoothing width (numerical only -- see smc_rate_sta.hpp)
    float lambda_i = 0;     // [1/s] PI-surface integral gain on s itself (0 = textbook STA on s=e)
    float e_reset  = 0;     // [rad/s] |e| threshold above which BOTH integral states snap to 0 (0 = disabled)
    float z_leak_tau = 0;   // [s] leaky-integration time constant for z (0 = disabled, pure integration); see smc_rate_sta.hpp

    // Output limit [Nm] -- same physical torque ceiling smc_rate_sta.hpp uses.
    // 出力上限 [Nm] -- smc_rate_sta.hppと同じ物理トルク上限。
    float output_limit = 1.0f;

    // State: the two integral-like states from smc_rate_sta.hpp, PLUS the
    // adaptive k1 itself (k2 is derived from k1 each cycle, not stored
    // independently), PLUS a low-pass filtered |s| used ONLY for the
    // dead-band decision (see filter_tau below and its use in compute()).
    // 状態: smc_rate_sta.hppと同じ2つの積分状態、加えて適応k1自体
    // （k2はk1から毎サイクル導出、独立には保持しない）、加えて不感帯判定
    // 専用の低域通過フィルタ済み|s|（下のfilter_tau、compute()内の使用箇所参照）。
    float integral   = 0;
    float z          = 0;
    float prev_error = 0;
    float k1         = 30.0f;  // current adaptive gain -- reset() seeds this from k1_init
    float s_abs_lpf  = 0;      // low-pass filtered |s|, for the dead-band decision only

    /// Compute the adaptive super-twisting torque output / 適応スーパーツイスティング・トルク出力を計算
    /// @param rate_sp    Target angular rate [rad/s] / 目標角速度
    /// @param rate_meas  Measured angular rate [rad/s] (bias-corrected gyro) / 測定角速度（バイアス補正済ジャイロ）
    /// @param dt         Time step [s] / タイムステップ
    /// @return           Torque [Nm], clamped to +/-output_limit / トルク [Nm]、+/-output_limit にクランプ
    float compute(float rate_sp, float rate_meas, float dt)
    {
        const float e = rate_sp - rate_meas;

        // Trapezoidal trial update of the PI-surface integral (mirrors
        // smc_rate_sta.hpp).
        // PI面積分の台形則による試験更新（smc_rate_sta.hppを踏襲）。
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

        const float s_trial = e + lambda_i * integral_trial;
        const float sign_s_trial = smoothSign(s_trial);

        // --- Adaptive law: update k1 based on whether s is converged ---
        // (Placed BEFORE the trial torque so the SAME-cycle k1 is used for
        // both the trial and final torque -- no one-cycle lag between the
        // adaptation and its effect.)
        //
        // The dead-band decision runs on a LOW-PASS FILTERED |s|
        // (s_abs_lpf), not the raw instantaneous value. Rationale (SILS
        // finding, docs/plans/smc-rate-loop-plan.md section 7.31): with a
        // raw-|s| dead-band, a single k1_max choice could not satisfy both
        // a sustained disturbance (torque-authority=0.4, which needs k1 to
        // grow) and sensor noise (noise n1, which should NOT make k1 grow
        // -- noise spikes |s| briefly but doesn't represent an unrejected
        // disturbance). Filtering |s| first exploits exactly that
        // difference in time structure: a sustained disturbance keeps the
        // FILTERED |s| elevated across many cycles, while noise's brief
        // spikes average out below dead_band. filter_tau sets this
        // separation's time scale -- large enough to reject fast noise,
        // small enough to still react to a real disturbance promptly.
        // --- 適応則: sが収束しているかに基づきk1を更新 ---
        // （試験トルクの計算前に置き、同一サイクルのk1をtrial/finalどちらの
        // トルクにも使う——適応とその効果の間に1サイクルの遅れを作らない）
        //
        // 不感帯判定は生の瞬時値ではなく、**低域通過フィルタ済み|s|**
        // (s_abs_lpf)で行う。根拠（SILSでの発見、docs/plans/
        // smc-rate-loop-plan.md §7.31）: 生の|s|で不感帯判定すると、単一の
        // k1_max値では持続外乱（torque-authority=0.4、k1を成長させたい）と
        // センサノイズ（noise n1、k1を成長させたくない——ノイズは|s|を
        // 一瞬だけ跳ね上げるが未抑制の外乱ではない）を両立できなかった。
        // |s|を先にフィルタすることで、まさにこの時間構造の違いを利用する:
        // 持続外乱はフィルタ後の|s|を何サイクルも高く保つが、ノイズの
        // 一瞬のスパイクは平均するとdead_band以下に収まる。filter_tauが
        // この切り分けの時間スケールを決める——速いノイズを除去できる
        // 程度に大きく、実外乱には即座に反応できる程度に小さく。
        if (dt > 0) {
            const float alpha_filt = dt / (filter_tau + dt);
            s_abs_lpf += alpha_filt * (fabsf(s_trial) - s_abs_lpf);

            const float k1_dot = (s_abs_lpf > dead_band)
                ? adapt_rate
                : -adapt_rate * leak_ratio;
            k1 += k1_dot * dt;
            if (k1 < k1_min) k1 = k1_min;
            if (k1 > k1_max) k1 = k1_max;
        }
        const float k2 = k2_ratio * k1;

        // Trial STA torque at the trial integral states -- "trial" because
        // anti-windup below may reject the update, same pattern as
        // smc_rate_sta.hpp.
        // 試験積分状態でのSTAトルク（試験更新） -- 「試験」なのはこの後の
        // アンチワインドアップが棄却しうるため、smc_rate_sta.hppと同じ方式。
        auto staOutput = [this](float e_now, float integ, float z_now) {
            const float s = e_now + lambda_i * integ;
            float sign_s;
            if (phi <= 1.0e-6f) sign_s = (s > 0) ? 1.0f : (s < 0 ? -1.0f : 0.0f);
            else {
                float v = s / phi;
                if (v >  1.0f) v =  1.0f;
                if (v < -1.0f) v = -1.0f;
                sign_s = v;
            }
            const float u1 = k1 * sqrtf(fabsf(s)) * sign_s;
            return inertia * (u1 + z_now);
        };

        // Leaky integration of z -- identical mechanism to smc_rate_sta.hpp
        // (see that file's header for the full rationale); k2 here is this
        // cycle's adaptive value.
        // zの漏れ積分——smc_rate_sta.hppと同一機構（根拠は同ファイル参照）。
        // ここでのk2は本サイクルの適応値。
        const float z_trial = (dt > 0)
            ? z + (-k2 * sign_s_trial - (z_leak_tau > 1.0e-6f ? z / z_leak_tau : 0.0f)) * dt
            : z;

        const float torque_trial = staOutput(e, integral_trial, z_trial);

        // Conditional-integration anti-windup, applied to BOTH integral
        // states together (same trigger as smc_rate_sta.hpp).
        // 条件付き積分アンチワインドアップ、両方の積分状態に同時適用
        // （smc_rate_sta.hppと同じトリガ）。
        const bool push_high = (torque_trial >  output_limit) && (e > 0);
        const bool push_low  = (torque_trial < -output_limit) && (e < 0);
        if (!push_high && !push_low) {
            integral = integral_trial;
            z        = z_trial;
        }

        // Backstop clamp on z -- identical to smc_rate_sta.hpp: z_dot is
        // +/-k2 (bounded), so z itself is clamped to its own maximum
        // possible torque contribution.
        // zへのバックストップクランプ——smc_rate_sta.hppと同一: z_dotは
        // +/-k2で有界なので、zが持ちうる最大トルク寄与へ直接クランプ。
        if (inertia > 1.0e-12f) {
            const float z_max = output_limit / inertia;
            if (z >  z_max) z =  z_max;
            if (z < -z_max) z = -z_max;
        }

        // Large-error gate reset -- same rationale as smc_rate_sta.hpp,
        // applied to both integral states (NOT to k1 -- a large transient
        // error should not throw away gain built up for an ongoing
        // disturbance; k1 only responds to the adaptive law above).
        // 偏差ゲートによるリセット——smc_rate_sta.hppと同じ根拠、両積分状態
        // に適用（k1には適用しない——大きな過渡誤差で、継続中の外乱向けに
        // 積み上げたゲインを捨てるべきではない。k1は上の適応則のみに従う）。
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

    /// Reset internal state, including re-seeding the adaptive gain from
    /// k1_init (so a mode re-entry starts from the tuned baseline, not
    /// wherever k1 drifted to last flight).
    /// 内部状態をリセット。適応ゲインもk1_initから再シードする（モード再
    /// 突入時、前回飛行でk1が漂着した値からではなく調整済みの初期値から
    /// 始まるように）。
    void reset()
    {
        integral   = 0;
        z          = 0;
        prev_error = 0;
        k1         = k1_init;
        s_abs_lpf  = 0;
    }
};

}  // namespace sf::app
