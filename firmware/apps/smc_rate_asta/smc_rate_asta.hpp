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
 * --- Section 7.32 -> 7.33 design history: from crossing-count to a
 * reference model / セクション7.32→7.33の設計変遷: クロス計数から規範
 * モデルへ ---
 * §7.32's FIRST attempt at telling "sustained bias" (grow k1) apart from
 * "delay-driven oscillatory divergence" (shrink k1) counted s's zero
 * crossings via a leaky EMA and gated growth on that count. SILS confirmed
 * this fixes the primary divergence (pos_flight+motor-delay=15ms's
 * tumble-class failure) but introduced a SECOND, narrower failure: near
 * the end of a long POS_HOLD, with RC input completely unchanged, the
 * gate still (correctly, per its own logic) detected 2+ crossings and
 * shrank k1 -- but this particular oscillation was a marginal, essentially
 * convergent limit cycle (the vehicle was on the edge of having JUST
 * enough gain), not a divergent one, and shrinking k1 pushed it past that
 * edge into an actual motor-duty asymmetry and a hard "impact detected" /
 * emergency disarm. A raw crossing COUNT cannot make this distinction --
 * it has no notion of "converged relative to what". A 4-point parameter
 * sweep (osc_thresh 2.0/3.0, osc_shrink_ratio 0.5/0.2, osc_thresh=20 i.e.
 * near-disabled) confirmed the gate is binary in effect: any active
 * setting reproduces the tail-end failure, disabling it reproduces the
 * original divergence (docs/plans/smc-rate-loop-plan.md section 7.32).
 *
 * §7.33 replaces the crossing counter with a REFERENCE MODEL [R11]: a
 * simple, non-adaptive first-order lag per axis, driven by the SAME
 * rate_sp this controller receives, produces an idealized "healthy
 * closed-loop" rate response rate_model. The MODEL-FOLLOWING error
 * e_model = rate_meas - rate_model is tracked with a FAST and a SLOW leaky
 * EMA of |e_model|; when the fast EMA notably exceeds the slow one (by a
 * ratio, past an absolute floor so both being near-zero doesn't trigger a
 * noisy ratio), the actual response is judged to be diverging AWAY from
 * the reference model's ideal trajectory -- not merely "s is nonzero" or
 * "s crossed zero N times" -- and k1 is shrunk. During the C2-step
 * divergence, e_model genuinely grows large and fast (the real system
 * cannot track rate_sp as well as the reference model can) -- this still
 * triggers correctly. During the §7.32 tail-end near-limit-cycle, the
 * actual rate stays close to the (small, near-constant) commanded rate
 * and hence close to the reference model too, so e_model stays small and
 * the gate should NOT fire -- this is the hypothesis this design is meant
 * to verify numerically (docs/plans/smc-rate-loop-plan.md section 7.33),
 * not a guarantee.
 * §7.32の最初の試み（「持続バイアス」（k1増加すべき）と「遅延駆動の発振的
 * 発散」（k1減少すべき）の区別）は、sのゼロクロスを漏れ積分EMAで計数し、
 * その回数でゲインの成長をゲーティングするものだった。SILSにより主破綻
 * （pos_flight+motor-delay=15msの転倒級破綻）の解消は確認できたが、
 * より狭い第二の破綻を新たに生んだ: 長時間POS_HOLDの終盤、RC入力が
 * 一切変化していないにも関わらず、ゲートは（そのロジック通り正しく）
 * 2回以上のクロスを検知してk1を縮小した——しかしこの振動は発散的では
 * なく、限界的でほぼ収束気味の極限サイクル（機体はぎりぎり十分なゲインの
 * 際にいた）であり、k1を縮小したことでその際から実際に踏み外し、モータ
 * デューティの非対称・「衝撃検知」→緊急DISARMに至った。生のクロス
 * 「回数」だけではこの区別ができない——「何に対して収束しているか」という
 * 概念自体を持たないため。4点のパラメータ実験（osc_thresh 2.0/3.0、
 * osc_shrink_ratio 0.5/0.2、osc_thresh=20＝実質無効化）で、このゲートが
 * 「有るか無いか」の二値的な効き方をすることを確認済み——有効な設定は
 * 全て末尾の破綻を再現し、無効化すると当初の発散が全面再発する
 * （docs/plans/smc-rate-loop-plan.md §7.32）。
 *
 * §7.33ではクロス計数を**規範モデル**[R11]で置き換える: 各軸に単純な
 * 非適応の一次遅れモデルを追加し、本コントローラと同じrate_spを入力として
 * 「健全な閉ループ」の理想的なレート応答rate_modelを生成する。規範モデル
 * への追従誤差e_model = rate_meas - rate_modelの絶対値を、速い/遅い2つの
 * 漏れ積分EMAで追跡し、速い方が遅い方を（絶対フロアを超えて）明確に
 * 上回ったときだけ、実際の応答が規範モデルの理想軌道から発散していると
 * 判定する——単に「sが非ゼロ」「sがN回ゼロクロスした」ではなく。k1は
 * このときのみ縮小する。C2ステップの発散時はe_modelが実際に急成長する
 * （実システムは規範モデルほどrate_spに追従できない）ので、正しく検知
 * される。§7.32の終盤の限界サイクルでは、実際のレートは指令された
 * （小さく、ほぼ一定の）レートに近いままであり、規範モデルにも近いままの
 * はずなのでe_modelは小さく保たれ、ゲートは発火しないはず——これは
 * 数値的に検証すべき仮説であり（docs/plans/smc-rate-loop-plan.md
 * §7.33）、保証ではない。
 *
 * @design docs/plans/smc-rate-loop-plan.md section 7.33 -- reference-model adaptive STA trial [--]
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
 *   [R8] Y. Shtessel, M. Taleb, and F. Plestan, "A novel adaptive-gain
 *        supertwisting sliding mode controller: Methodology and
 *        application," Automatica, vol. 48, no. 5, pp. 759-769, 2012.
 *        doi:10.1016/j.automatica.2012.02.024.
 *   [R9] Y. Wang, W. Zhang, Y. Yang, C. Xue, S. Yuan, and H. Zhang,
 *        "Adaptive Second-Order Sliding Mode Control of Buck Converters
 *        with Multi-Disturbances," Energies, vol. 15, no. 14, p. 5139,
 *        2022. doi:10.3390/en15145139. -- counts sliding-surface
 *        zero-crossings online to drive a time-varying gain. Section
 *        7.32's FIRST attempt applied this directly as a crossing-count
 *        gate; superseded below by [R11]'s reference-model approach after
 *        SILS found a second failure mode (see the design-history comment
 *        above and docs/plans/smc-rate-loop-plan.md section 7.32/7.33).
 *   [R10] "New methodology for adaptive sliding mode control with
 *        self-tuning threshold based on chattering detection," Mechanical
 *        Systems and Signal Processing, 2025 (online). A gain-adaptation
 *        law driven directly by the appearance of chattering in the
 *        closed loop, rather than an arbitrary amplitude threshold.
 *   [R11] W. Barreto da Silveira, P. J. D. de Oliveira Evald,
 *        G. V. Hollweg, D. M. C. Milbradt, R. V. Tambara, and
 *        H. A. Gruendling, "Robust Model Reference Adaptive Control With a
 *        Full Adaptive Super-Twisting Sliding Mode Action: Discrete-Time
 *        Stability Analysis and Application," International Journal of
 *        Adaptive Control and Signal Processing, Wiley, 2025.
 *        doi:10.1002/acs.4101. -- combines a REFERENCE MODEL with an
 *        adaptive-gain STA: the switching action is driven by the
 *        MODEL-FOLLOWING error (measured output vs. the reference model's
 *        own state) and is reduced once the closed loop reaches steady
 *        state relative to that model, rather than by the raw sliding
 *        variable's amplitude or crossing count alone. THIS is the
 *        mechanism section 7.33 (and this file's compute()) implements
 *        below, replacing section 7.32's zero-crossing count.
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

    // --- Reference-model divergence gate (docs/plans/smc-rate-loop-plan.md
    // section 7.33, [R11] -- supersedes section 7.32's zero-crossing-count
    // gate, see the file header's design-history comment for why). A
    // simple first-order lag model, driven by the SAME rate_sp this
    // controller receives, stands in for "how a healthy closed loop would
    // respond". Its own state is mref_tau; everything else here governs
    // how the model-following error e_model = rate_meas - rate_model is
    // turned into a shrink decision.
    // --- 規範モデルによる発散ゲート（docs/plans/smc-rate-loop-plan.md
    // §7.33、[R11]——§7.32のゼロクロス計数ゲートを置き換える、理由はファイル
    // 冒頭の設計変遷コメント参照）。本コントローラと同じrate_spで駆動される
    // 単純な一次遅れモデルが「健全な閉ループならどう応答するか」の代役を
    // 果たす。そのモデル自身の状態はmref_tau、それ以外はここでは規範モデル
    // への追従誤差e_model = rate_meas - rate_modelを縮小判断へどう変換するか
    // を司る。
    float mref_tau          = 0.03f; // [s] reference model's own first-order time constant (target/ideal rate-loop response) -- SEED, SILS-unverified
    float mref_fast_tau     = 0.05f; // [s] fast leaky-EMA time constant on |e_model| -- reacts within roughly one C2-step timescale
    float mref_slow_tau     = 0.5f;  // [s] slow leaky-EMA time constant on |e_model| -- the "recent normal" baseline the fast EMA is compared against
    float mref_growth_ratio = 1.5f;  // fast EMA must exceed slow EMA by this multiple (before mref_abs_floor is added) to judge "diverging"
    float mref_abs_floor    = 0.05f; // [rad/s] additive floor so two near-zero EMAs (both quiet) don't trigger on ratio noise alone
    float mref_shrink_ratio = 0.5f;  // decay rate while diverging, as a fraction of adapt_rate (independent of leak_ratio) -- same role/value as section 7.32's osc_shrink_ratio

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
    // independently), PLUS a low-pass filtered SIGNED s (NOT |s|) used
    // ONLY for the dead-band decision (see filter_tau below and its use in
    // compute() for why filtering s, not |s|, matters), PLUS the
    // reference-model state (rate_model) and its fast/slow model-following
    // error EMAs used for the divergence gate (see mref_* above).
    // 状態: smc_rate_sta.hppと同じ2つの積分状態、加えて適応k1自体
    // （k2はk1から毎サイクル導出、独立には保持しない）、加えて不感帯判定
    // 専用の低域通過フィルタ済み**符号付き**s（|s|ではない、下のfilter_tau・
    // compute()内のなぜsをフィルタすべきかの説明参照）、加えて規範モデルの
    // 状態（rate_model）と、発散ゲートに使う追従誤差の速い/遅いEMA
    // （mref_*参照）。
    float integral   = 0;
    float z          = 0;
    float prev_error = 0;
    float k1         = 30.0f;  // current adaptive gain -- reset() seeds this from k1_init
    float s_lpf      = 0;      // low-pass filtered SIGNED s (not |s|!), for the dead-band decision only -- see compute()'s rationale comment
    float rate_model   = 0;    // reference model's own state (an idealized rate_meas, driven by rate_sp) -- see mref_tau
    float e_model_fast = 0;    // fast leaky EMA of |e_model| -- see mref_fast_tau
    float e_model_slow = 0;    // slow leaky EMA of |e_model| -- see mref_slow_tau

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
        // The dead-band decision runs on |LPF(s)| -- the SIGNED s is
        // filtered FIRST, then rectified -- NOT LPF(|s|) (filtering the
        // already-rectified magnitude), which is what an earlier version of
        // this file did. That earlier version filtered |s|, and this SIGN
        // ERROR is why it failed to separate noise from a sustained
        // disturbance (docs/plans/smc-rate-loop-plan.md section 7.31): |s|
        // is already non-negative, so averaging it does NOT cancel
        // zero-mean noise -- a zero-mean chattering s produces a
        // rectified |s| with a persistent POSITIVE bias (its mean absolute
        // deviation), which low-pass filtering preserves rather than
        // removes. Filtering s itself is different: e/s under sensor noise
        // is genuinely close to zero-mean (chatters around the true value),
        // so LPF(s) correctly decays toward zero, while a sustained
        // one-directional disturbance (torque-authority=0.4 -- the plant
        // genuinely cannot produce enough differential torque, so e stays
        // same-signed) biases s away from zero and LPF(s) reflects that
        // directly. filter_tau sets the separation time scale -- large
        // enough to average out fast zero-mean chatter, small enough to
        // still react to a real disturbance promptly.
        // --- 適応則: sが収束しているかに基づきk1を更新 ---
        // （試験トルクの計算前に置き、同一サイクルのk1をtrial/finalどちらの
        // トルクにも使う——適応とその効果の間に1サイクルの遅れを作らない）
        //
        // 不感帯判定は|LPF(s)|——**符号付きsを先にフィルタしてから絶対値**
        // を取る——であって、LPF(|s|)（既に絶対値を取った後の量をフィルタ
        // する）ではない。このファイルの以前のバージョンは|s|をフィルタして
        // おり、この**符号の誤り**が、ノイズと持続外乱を分離できなかった
        // 原因だった（docs/plans/smc-rate-loop-plan.md §7.31）:
        // |s|は既に非負なので、平均してもゼロ平均ノイズは打ち消せない
        // ——ゼロ平均で振動するsを整流した|s|は、持続的な**正のバイアス**
        // （その平均絶対偏差）を持ち、低域通過フィルタはこれを除去する
        // どころか保持してしまう。s自体をフィルタするのは違う: センサ
        // ノイズ下のe/sは真値の周りで振動する、実質ゼロ平均に近い信号
        // なので、LPF(s)は正しくゼロへ減衰する。一方、持続的な一方向の
        // 外乱（torque-authority=0.4——プラントが本当に十分な差動トルクを
        // 出せず、eが同符号のまま留まる）はsをゼロから偏らせ、LPF(s)は
        // それを直接反映する。filter_tauがこの切り分けの時間スケールを
        // 決める——速いゼロ平均のチャタリングを平均化できる程度に大きく、
        // 実外乱には即座に反応できる程度に小さく。
        //
        // --- Reference-model divergence gate (docs/plans/smc-rate-loop-
        // plan.md section 7.33, [R11]; supersedes section 7.32's zero-
        // crossing count -- see file header's design-history comment for
        // why): a non-adaptive first-order model, driven by the SAME
        // rate_sp, stands in for how a healthy closed loop should respond.
        // Its model-following error e_model = rate_meas - rate_model is
        // tracked with a fast and a slow leaky EMA of |e_model|. When the
        // fast EMA meaningfully exceeds the slow one (a real, fast-forming
        // gap between "recent" and "current" tracking quality -- not just
        // e_model being nonzero, which it always is to some degree), the
        // actual response is judged to be diverging FROM THE REFERENCE
        // MODEL's ideal trajectory, and k1 is forced to shrink -- even if
        // |LPF(s)| is still above dead_band, same priority rule section
        // 7.32 used for its (now superseded) oscillation flag.
        // --- 規範モデルによる発散ゲート（docs/plans/smc-rate-loop-plan.md
        // §7.33、[R11]；§7.32のゼロクロス計数を置き換える——理由はファイル
        // 冒頭の設計変遷コメント参照）: 本コントローラと同じrate_spで駆動
        // される非適応の一次遅れモデルが、健全な閉ループならどう応答すべき
        // かの代役を果たす。その規範モデルへの追従誤差
        // e_model = rate_meas - rate_modelの絶対値を、速い/遅い2つの漏れ
        // 積分EMAで追跡する。速い方が遅い方を明確に上回ったとき（「最近」と
        // 「今」の追従品質の間に実際に急速なギャップが生じている——単に
        // e_modelが非ゼロというだけではない、これは常にある程度非ゼロ）、
        // 実際の応答が規範モデルの理想軌道から発散していると判定し、k1を
        // 強制的に縮小する——|LPF(s)|が不感帯を超えていても、§7.32の
        // （今は置き換えられた）発振フラグと同じ優先ルールに従う。
        if (dt > 0) {
            const float alpha_model = dt / (mref_tau + dt);
            rate_model += alpha_model * (rate_sp - rate_model);
        }
        const float e_model = rate_meas - rate_model;
        const float abs_e_model = fabsf(e_model);
        if (dt > 0) {
            const float alpha_fast = dt / (mref_fast_tau + dt);
            const float alpha_slow = dt / (mref_slow_tau + dt);
            e_model_fast += alpha_fast * (abs_e_model - e_model_fast);
            e_model_slow += alpha_slow * (abs_e_model - e_model_slow);
        }
        const bool diverging = e_model_fast > (mref_growth_ratio * e_model_slow + mref_abs_floor);

        if (dt > 0) {
            const float alpha_filt = dt / (filter_tau + dt);
            s_lpf += alpha_filt * (s_trial - s_lpf);

            float k1_dot;
            if (diverging) {
                k1_dot = -adapt_rate * mref_shrink_ratio;
            } else if (fabsf(s_lpf) > dead_band) {
                k1_dot = adapt_rate;
            } else {
                k1_dot = -adapt_rate * leak_ratio;
            }
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
    /// wherever k1 drifted to last flight), and the reference model
    /// (so a mode re-entry does not inherit a stale model state from
    /// whatever rate_sp/rate_meas prevailed at the end of the previous
    /// flight segment).
    /// 内部状態をリセット。適応ゲインもk1_initから再シードする（モード再
    /// 突入時、前回飛行でk1が漂着した値からではなく調整済みの初期値から
    /// 始まるように）。規範モデルもリセットする（モード再突入時、前回飛行
    /// 終端のrate_sp/rate_measに由来する古いモデル状態を引き継がないよう
    /// に）。
    void reset()
    {
        integral      = 0;
        z             = 0;
        prev_error    = 0;
        k1            = k1_init;
        s_lpf         = 0;
        rate_model    = 0;
        e_model_fast  = 0;
        e_model_slow  = 0;
    }
};

}  // namespace sf::app
