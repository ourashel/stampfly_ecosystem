/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file smc_rate.hpp
 * @brief Single-axis sliding-mode rate controller: a PI-type sliding surface
 *        (rate error + integral of rate error) with a boundary-layer
 *        reaching law.
 *        1軸スライディングモード・レートコントローラ: PI型スライディング面
 *        （レート誤差+その積分）＋境界層付き到達則。
 *
 * Sliding surface (PI-type, docs/plans/smc-rate-loop-plan.md SS3.4/SS6 [R3]):
 *
 *   e = rate_sp - rate_meas
 *   s = e + lambda_i * integral(e dt)
 *
 * On the surface (s=0), differentiating gives e_dot + lambda_i*e = 0 -- a
 * stable first-order error decay with time constant 1/lambda_i. This is the
 * SAME "s = e_dot + lambda*e" hyperplane structure classical SMC uses for a
 * relative-degree-2 plant (docs/plans/smc-rate-loop-plan.md SS2.2), except
 * here the second term is an INTEGRAL, not a derivative: the rate-loop
 * plant is still relative-degree-1 (SS2.2's reasoning is unchanged), so the
 * bare error e alone remains sufficient for TRACKING. lambda_i's job is
 * different -- it drives out STEADY multiplicative actuator-effectiveness
 * loss (e.g. --torque-authority < 1), the one thing the original pure-P
 * reaching law had no mechanism for (see SS3.3's root-cause analysis: a
 * bounded switching term k*sat() cannot substitute for PID's integral once
 * it saturates). lambda_i=0 recovers the original SS2.2 design exactly.
 *
 * IMPORTANT precision note (SS6 in the plan doc): this is the common
 * *practical simplification* usually called a "PI-type sliding surface",
 * NOT Utkin & Shi's original "Integral Sliding Mode" [R3], which is a more
 * elaborate order-preserving construction that removes the reaching phase
 * entirely from t=0. Do not conflate the two in comments/docs -- see the
 * plan's SS6 reference table for the distinction.
 *
 * Reaching law (unchanged from the original design, [R1][R2]):
 *
 *   torque = I_axis * ( k*sat(s/phi) + eta*s )
 *
 * Anti-windup: THREE layers, the extra two added 2026-09-11 after review
 * (docs/plans/smc-rate-loop-plan.md SS3.6):
 *   1. Conditional integration -- the SAME pattern pid.hpp's PID struct
 *      already uses (only accumulate when doing so would not push an
 *      already-saturated output further into saturation) -- adapted here to
 *      gate on the TRIAL torque computed through the full reaching law,
 *      since the surface (not the integral term alone) is what gets
 *      clamped.
 *   2. Backstop hard clamp on |integral| -- pid.hpp's PID keeps this as a
 *      belt-and-suspenders bound even though (1) should already prevent
 *      windup; SlidingModeRate lacked it (an oversight caught in review).
 *      Bounded so the integral's own LINEAR contribution to torque
 *      (inertia*eta*lambda_i*integral) alone cannot exceed output_limit --
 *      the closest equivalent to PID's own integral-in-torque-units clamp,
 *      given this integral is kept in [rad] (raw integral(e dt)), not
 *      torque units.
 *   3. Large-error gate reset -- if |e| exceeds e_reset (a rate threshold,
 *      0 = disabled), the integral is snapped to 0 rather than merely
 *      frozen. (1)/(2) only bound accumulation; they do not discard a
 *      stale integral value left over from a large transient (an
 *      aggressive maneuver, a big disturbance) before it leaks into calmer
 *      tracking afterward. e_reset is expressed as a multiple of phi (the
 *      boundary layer) by convention in params.cpp's seed, since "how far
 *      outside the linear operating regime counts as a large transient" is
 *      naturally phi-relative.
 *

 * スライディング面（PI型、docs/plans/smc-rate-loop-plan.md §3.4/§6 [R3]）:
 *
 *   e = rate_sp - rate_meas
 *   s = e + lambda_i * integral(e dt)
 *
 * 面上（s=0）を微分すると e_dot + lambda_i*e = 0 -- 収束時定数 1/lambda_i の
 * 安定な1次誤差減衰になる。これは古典的SMCが相対次数2のプラントに使う
 * 「s = e_dot + lambda*e」型超平面と同じ構造（docs/plans/smc-rate-loop-plan.md
 * §2.2）だが、第2項が微分でなく積分である点が異なる: レートループのプラント
 * は依然相対次数1のまま（§2.2の論拠は不変）なので、追従そのものには誤差e
 * 単体で十分。lambda_iの役割は別物 -- 持続的な乗法的アクチュエータ有効度
 * 損失（--torque-authority<1等）を駆逐する。これは元の純P到達則には無かった
 * 機構（§3.3の根本原因分析: 有界なスイッチング項k*sat()は、飽和後はPIDの
 * 積分の代替になれない）。lambda_i=0で元の§2.2設計に厳密に一致する。
 *
 * 重要な精度に関する注記（計画書§6）: これは一般に「PI型スライディング面」
 * と呼ばれる実務上の簡略版であり、Utkin & ShiのオリジナルIntegral Sliding
 * Mode [R3]（次数を保存し初期時刻から到達フェーズ自体を除去するより精緻な
 * 構成）そのものではない。コメント・文書内でこの2つを混同しないこと --
 * 区別は計画書§6の文献表を参照。
 *
 * 到達則（元の設計から変更なし、[R1][R2]）:
 *
 *   torque = I_axis * ( k*sat(s/phi) + eta*s )
 *
 * アンチワインドアップ: 3段構え（後の2つは2026-09-11のレビューで追加、
 * docs/plans/smc-rate-loop-plan.md §3.6）:
 *   1. 条件付き積分 -- pid.hppのPID structが既に使うパターンと同じ（既に
 *      飽和している出力をさらに飽和方向へ押す場合は積分を更新しない）--
 *      ここでは積分項単体でなく到達則全体を通した試験トルクでゲートする
 *      （クランプされるのは面自体だから）。
 *   2. |integral|へのバックストップ・ハードクランプ -- pid.hppのPIDは(1)が
 *      ワインドアップを防ぐはずでも保険として持つ。SlidingModeRateには
 *      これが欠けていた（レビューで発覚した抜け）。積分の「線形寄与分」
 *      （inertia*eta*lambda_i*integral）単体がoutput_limitを超えないよう
 *      境界を定める -- PIDの「積分をトルク単位で保持しクランプする」方式
 *      に最も近い等価物（本積分は[rad]（生の∫e dt）で保持しておりトルク
 *      単位ではないため）。
 *   3. 偏差ゲートによるリセット -- |e|がe_reset（レート閾値、0で無効）を
 *      超えたら、積分を単に凍結するのでなく0にスナップする。(1)/(2)は
 *      蓄積の上限を定めるだけで、大きな過渡（激しい機動・大外乱）から
 *      残った古い積分値が、その後の平穏な追従に漏れ出すのを防げない。
 *      e_reset はparams.cppの初期値でphi（境界層）の倍数として表現する
 *      慣例とする——「線形動作域からどれだけ外れれば大きな過渡と見なすか」
 *      は本質的にphi相対の概念であるため。
 *

 * @design docs/plans/smc-rate-loop-plan.md SS2.2/SS3.4/SS3.6/SS6 -- control law + PI-surface + anti-windup derivation [--]
 * @design controller.hpp -- IController interface (used via AppController)  [OK]
 *
 * References / 参考文献 (docs/plans/smc-rate-loop-plan.md SS6):
 *   [R1] W. Gao and J. C. Hung, "Variable structure control of nonlinear
 *        systems: a new approach," IEEE Trans. Industrial Electronics,
 *        vol. 40, no. 1, pp. 45-55, 1993.
 *   [R2] J.-J. E. Slotine and W. Li, Applied Nonlinear Control,
 *        Prentice Hall, 1991.
 *   [R3] V. Utkin and J. Shi, "Integral sliding mode in systems operating
 *        under uncertainty conditions," Proc. 35th IEEE CDC, Kobe, 1996,
 *        pp. 4591-4596.
 */

#pragma once

#include <cmath>

namespace sf::app {

/// Single-axis sliding-mode rate controller / 1軸スライディングモード・レートコントローラ
struct SlidingModeRate {
    // Physical moment of inertia for this axis [kg*m^2] -- multiplies the
    // reaching law so k/eta below are expressed in acceleration/rate units
    // (see file header), not raw torque. Set once at init() from
    // control/models/stampfly_physical.yaml's Ixx/Iyy/Izz (SSOT).
    // この軸の物理慣性モーメント [kg*m^2] -- 到達則に掛けることで、下の
    // k/eta が生トルクでなく角加速度/角速度の単位で表現される（ファイル
    // 先頭参照）。control/models/stampfly_physical.yaml の Ixx/Iyy/Izz（SSOT）
    // からinit()で一度だけ設定する。
    float inertia = 1.0f;   // [kg*m^2]

    // Gains (reloaded from params.cpp by the caller's reloadParams()) / ゲイン
    float k        = 0;     // [rad/s^2] switching/reaching gain (bounded torque outside the boundary layer = inertia*k)
    float eta      = 0;     // [1/s] linear reaching gain near the surface
    float phi      = 0.3f;  // [rad/s] boundary-layer half-width
    float lambda_i = 0;     // [1/s] PI-surface integral gain (0 = original pure-P surface, SS2.2)
    float e_reset  = 0;     // [rad/s] |e| threshold above which integral snaps to 0 (0 = disabled)

    // Output limit [Nm] -- same physical torque ceiling the PID rate loop
    // uses (max_roll_pitch_torque_ / rate.yaw.max_torque), set by the
    // caller so the two controllers share one ceiling.
    // 出力上限 [Nm] -- レートPIDと同じ物理トルク上限
    // （max_roll_pitch_torque_ / rate.yaw.max_torque）を呼び出し側が設定し、
    // 両制御器で上限を共有する。
    float output_limit = 1.0f;

    // Integral-of-error state [rad] and its previous sample (trapezoidal
    // integration, same scheme as pid.hpp's PID::integral/prev_error).
    // 積分誤差状態 [rad] と前回サンプル（台形積分、pid.hppのPID::integral/
    // prev_errorと同じ方式）。
    float integral    = 0;
    float prev_error  = 0;

    /// Compute the sliding-mode torque output / スライディングモード・トルク出力を計算
    /// @param rate_sp    Target angular rate [rad/s] / 目標角速度
    /// @param rate_meas  Measured angular rate [rad/s] (bias-corrected gyro) / 測定角速度（バイアス補正済ジャイロ）
    /// @param dt         Time step [s] / タイムステップ
    /// @return           Torque [Nm], clamped to +/-output_limit / トルク [Nm]、+/-output_limit にクランプ
    float compute(float rate_sp, float rate_meas, float dt)
    {
        const float e = rate_sp - rate_meas;

        // Trapezoidal trial update of the integral state (mirrors pid.hpp's
        // PID::compute -- "trial" because anti-windup below may reject it).
        // 積分状態の台形則による試験更新（pid.hppのPID::computeを踏襲 --
        // 「試験」なのは下のアンチワインドアップが棄却しうるため）。
        const float integral_trial = (dt > 0)
            ? integral + (e + prev_error) * (dt * 0.5f)
            : integral;

        // Boundary layer: replace sign(s) with a saturated linear ramp
        // sat(s/phi) so the switching term does not chatter across s=0 at
        // every 2.5ms control step -- the classical SMC chattering
        // mitigation ([R2]). phi=0 would divide by zero, so guard it
        // (should never happen: params.cpp's min bound keeps phi > 0).
        // 境界層: sign(s) の代わりに飽和付き線形ランプ sat(s/phi) を使い、
        // 2.5msの制御周期ごとにs=0をまたいでスイッチング項がチャタリング
        // するのを防ぐ（[R2]の古典的チャタリング抑制）。phi=0はゼロ除算に
        // なるためガードする（params.cppのmin境界でphi>0が保証されるため
        // 通常は発生しない）。
        auto reachingTorque = [this](float e_now, float integ) {
            const float s = e_now + lambda_i * integ;
            float switching = 0.0f;
            if (phi > 1.0e-6f) {
                switching = s / phi;
                if (switching >  1.0f) switching =  1.0f;
                if (switching < -1.0f) switching = -1.0f;
            }
            return inertia * (k * switching + eta * s);
        };

        const float torque_trial = reachingTorque(e, integral_trial);

        // Conditional-integration anti-windup ([pid.hpp] pattern): only
        // accept the integral update if it would NOT push an already-
        // saturated output further into saturation. Gated on the FULL
        // reaching-law torque (not just the integral term in isolation),
        // since it is the surface s -- not integral alone -- that
        // ultimately gets clamped by output_limit below.
        // 条件付き積分アンチワインドアップ（[pid.hpp]のパターン）: 既に
        // 飽和している出力をさらに飽和方向へ押す場合は積分の更新を受け
        // 入れない。到達則全体のトルク（積分項単体でなく）でゲートする --
        // 最終的にoutput_limitでクランプされるのは面s自体であって積分単体
        // ではないため。
        const bool push_high = (torque_trial >  output_limit) && (e > 0);
        const bool push_low  = (torque_trial < -output_limit) && (e < 0);
        if (!push_high && !push_low) {
            integral = integral_trial;
        }

        // Layer 2 -- backstop hard clamp: bound |integral| so its own linear
        // contribution to torque cannot alone exceed output_limit, even if
        // some future change bypasses layer 1. Skipped when eta*lambda_i is
        // negligible (no meaningful linear contribution to bound; avoids a
        // near-zero denominator).
        // 第2層 -- バックストップ・ハードクランプ: 積分の線形寄与分単体が
        // output_limitを超えないよう|integral|を制限する（将来の変更が
        // 第1層を迂回しても保険になる）。eta*lambda_iが無視できるほど
        // 小さい場合はスキップ（束縛すべき線形寄与が実質無く、ゼロに近い
        // 除数を避ける）。
        const float linear_gain = inertia * eta * lambda_i;
        if (linear_gain > 1.0e-9f) {
            const float integral_max = output_limit / linear_gain;
            if (integral >  integral_max) integral =  integral_max;
            if (integral < -integral_max) integral = -integral_max;
        }

        // Layer 3 -- large-error gate reset: a big transient (aggressive
        // maneuver, large disturbance) makes the accumulated integral stale;
        // discard it outright rather than let it leak into calmer tracking
        // afterward. e_reset=0 disables this layer.
        // 第3層 -- 偏差ゲートによるリセット: 大きな過渡（激しい機動・大
        // 外乱）は蓄積済みの積分を古くする——単に凍結するのでなく、その後の
        // 平穏な追従へ漏れ出す前に完全に破棄する。e_reset=0でこの層を無効化。
        if (e_reset > 1.0e-6f && fabsf(e) > e_reset) {
            integral = 0;
        }

        prev_error = e;

        const float torque = reachingTorque(e, integral);
        if (torque >  output_limit) return  output_limit;
        if (torque < -output_limit) return -output_limit;
        return torque;
    }

    /// Reset internal state / 内部状態をリセット
    void reset()
    {
        integral   = 0;
        prev_error = 0;
    }
};

}  // namespace sf::app
