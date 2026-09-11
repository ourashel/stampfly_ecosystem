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
 *   [R4] O. J. M. Smith, "Closer control of loops with dead time,"
 *        Chemical Engineering Progress, vol. 53, no. 5, pp. 217-219, 1957.
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

    // --- Dead-time predictor (opt-in, docs/plans/smc-rate-loop-plan.md's
    // motor-delay root-cause work) ---
    // Compensates a pure transport delay L in the torque->rate path (e.g.
    // SILS's --motor-delay, or the real hardware's measured per-axis delay,
    // L_total~10.8-17.3ms, analysis/reports/rate_sysid_reference/reference.json,
    // corrected 2026-09-11 -- an earlier 8.4-14.7ms figure understated pitch by
    // ~2x, see docs/plans/smc-rate-loop-plan.md SS7.7e) by predicting the
    // CURRENT rate from the raw measurement
    // plus the integrated effect of this controller's OWN torque commands
    // over the last L seconds -- the delayed torque has not fully acted on
    // the plant yet at the moment it was issued, so replaying its
    // (torque/inertia)*dt contribution catches the loop up to "as if" there
    // were no delay:
    //
    //   rate_predicted = rate_meas + sum_{i=t-L}^{t} (torque_i/inertia)*dt
    //
    // This is a simplified, integrator-plant-specific relative of the Smith
    // predictor [R4] (O.J.M. Smith, "Closer control of loops with dead
    // time," Chem. Eng. Progress, 1957): a true Smith predictor differences
    // a delayed AND an undelayed nominal-model prediction against the real
    // (delayed) measurement; here, because the rate-loop plant is
    // essentially a pure integrator (torque->angular accel, negligible
    // additional lag -- the same relative-degree-1 assumption this file's
    // header already makes), summing the controller's own recent command
    // history directly onto the raw measurement plays the same role without
    // needing a separate plant-lag model. delay_comp_s=0 (default) disables
    // this entirely -- e is computed from the raw rate_meas exactly as
    // before, so existing smc_rate/smc_pos behavior is unchanged unless an
    // app explicitly sets this parameter.
    // --- 無駄時間予測補償器（opt-in、docs/plans/smc-rate-loop-plan.mdの
    // motor-delay根本対処の一環） ---
    // トルク→レート経路の純粋な輸送遅れL（SILSの--motor-delay、または実機の
    // 軸別実測遅れ L_total~10.8-17.3ms、analysis/reports/rate_sysid_reference/
    // reference.json、2026-09-11訂正——旧8.4-14.7msはpitchを約半分に過小評価
    // していた、docs/plans/smc-rate-loop-plan.md §7.7e参照）を、生の測定値に
    // 「直近L秒間に自分が出した
    // トルク指令の積算効果」を足し込んで現在レートを予測することで補償する
    // -- 指令した瞬間はまだプラントに完全には効いていないトルクの
    // (torque/inertia)*dt寄与を再生することで、無駄時間が無かったかのように
    // ループを追いつかせる:
    //
    //   rate_predicted = rate_meas + Σ_{i=t-L}^{t} (torque_i/inertia)*dt
    //
    // これはSmith予測器[R4]（O.J.M. Smith, "Closer control of loops with
    // dead time," Chem. Eng. Progress, 1957）の、積分器プラント特化の簡略版
    // というべき関係にある: 真のSmith予測器は「遅延ありのノミナルモデル予測」
    // と「遅延なしのノミナルモデル予測」の差分を実測（遅延あり）と比較するが、
    // ここではレートループのプラントが本質的に純粋な積分器（トルク→角加速度、
    // 追加の1次遅れは無視できる -- このファイル冒頭の相対次数1の前提と同じ）
    // であるため、コントローラ自身の最近の指令履歴を生の測定値へ直接積算する
    // だけで、別途プラント遅れモデルを持たずに同じ役割を果たす。
    // delay_comp_s=0（既定）はこれを完全に無効化する -- eは従来どおり生の
    // rate_measから計算され、appが明示的にこのパラメータを設定しない限り
    // 既存のsmc_rate/smc_posの挙動は変わらない。
    static constexpr int   kDelayCompMaxSamples = 16;  // covers up to 40ms @ 400Hz (2.5ms/sample)
    float delay_comp_s = 0;  // [s] nominal dead time to compensate (0 = disabled)
    float torque_ring_[kDelayCompMaxSamples] = {};
    int   torque_ring_idx_   = 0;
    int   torque_ring_count_ = 0;  // samples filled so far (ramps in at startup/reset)

    // --- Sliding-surface dead-band (opt-in, alternative/complementary to the
    // dead-time predictor above -- docs/plans/smc-rate-loop-plan.md SS7.10,
    // added 2026-09-11 after review: the predictor requires an accurate
    // model of the ACTUAL delay L and diverges tumble-class when mismatched
    // by only ~7-9ms (SS7.7c/7.7e); a dead-band needs no delay knowledge at
    // all). Shrinks |s| toward zero by up to s_deadband before the boundary
    // layer sees it:
    //
    //   s' = s>s_deadband ? s-s_deadband : (s<-s_deadband ? s+s_deadband : 0)
    //
    // Rationale: a delay-induced instability is a GROWING OSCILLATION (a
    // classical dead-time/phase-margin problem -- the same "growing unstable
    // limit cycle" signature documented in firmware/vehicle/docs/
    // poshold_journey.md SS4.2 for the position loop). Small-amplitude s
    // excursions are exactly where such growth starts; refusing to react to
    // them (rather than reacting proportionally, as the boundary layer sat()
    // alone does) removes the energy the reaching law would otherwise feed
    // back into the growing cycle. This trades a small steady-state
    // tracking band (|s|<=s_deadband is not actively driven to zero) for
    // robustness that does not depend on knowing L -- unlike delay_comp_s,
    // a WRONG s_deadband cannot make the mismatch direction worse, only
    // under- or over-shrink the dead zone. s_deadband=0 (default) disables
    // this entirely -- s' = s exactly, unchanged behavior.
    // --- スライディング面の不感バンド（opt-in、上の無駄時間予測補償器の
    // 代替/補完、docs/plans/smc-rate-loop-plan.md §7.10、2026-09-11レビュー
    // を受けて追加: 予測補償器は実際の遅れLの正確なモデルを要求し、想定との
    // ズレが~7-9msあるだけで転倒級に発散する（§7.7c/7.7e）——不感バンドは
    // 遅れの知識を一切必要としない）。境界層に渡す前に|s|をs_deadband分だけ
    // ゼロへ縮める:
    //
    //   s' = s>s_deadband ? s-s_deadband : (s<-s_deadband ? s+s_deadband : 0)
    //
    // 根拠: 遅れ由来の不安定化は「成長する振動」（古典的な無駄時間・位相余裕
    // 問題——firmware/vehicle/docs/poshold_journey.md §4.2で位置ループに
    // ついて記録された「成長する不安定リミットサイクル」と同じ症状）。
    // 小振幅のs逸脱こそがその成長の起点であり、それに（境界層sat()単体の
    // ような比例反応でなく）反応しないことで、到達則が成長サイクルへ
    // フィードバックするエネルギーを絶つ。定常追従に小さな不感帯
    // （|s|<=s_deadbandは能動的にゼロへ駆動されない）が生じる代償として、
    // Lを知る必要のない頑健性を得る——delay_comp_sと異なり、s_deadbandを
    // 間違えても悪化方向にミスマッチすることはなく、不感帯の縮小/過剰の
    // 違いにしかならない。s_deadband=0（既定）で完全無効——s'=sのまま、
    // 挙動は不変。
    float s_deadband = 0;  // [rad/s] dead-zone half-width on s (0 = disabled)

    /// Compute the sliding-mode torque output / スライディングモード・トルク出力を計算
    /// @param rate_sp    Target angular rate [rad/s] / 目標角速度
    /// @param rate_meas  Measured angular rate [rad/s] (bias-corrected gyro) / 測定角速度（バイアス補正済ジャイロ）
    /// @param dt         Time step [s] / タイムステップ
    /// @return           Torque [Nm], clamped to +/-output_limit / トルク [Nm]、+/-output_limit にクランプ
    float compute(float rate_sp, float rate_meas, float dt)
    {
        // Dead-time predictor: replace rate_meas with rate_predicted for the
        // REST of this function (surface, reaching law, anti-windup) when
        // delay_comp_s > 0. See the field comment above for the derivation.
        // 無駄時間予測補償器: delay_comp_s>0ならrate_measをrate_predictedへ
        // 置き換え、この関数の残り（面・到達則・アンチワインドアップ）は
        // 全てそれを使う。導出は上のフィールドコメント参照。
        float rate_used = rate_meas;
        if (delay_comp_s > 1.0e-6f && dt > 0 && inertia > 1.0e-12f) {
            int n = static_cast<int>(delay_comp_s / dt + 0.5f);
            if (n > kDelayCompMaxSamples) n = kDelayCompMaxSamples;
            if (n > torque_ring_count_)   n = torque_ring_count_;
            float sum_domega = 0.0f;
            int idx = torque_ring_idx_;
            for (int i = 0; i < n; ++i) {
                idx = (idx - 1 + kDelayCompMaxSamples) % kDelayCompMaxSamples;
                sum_domega += (torque_ring_[idx] / inertia) * dt;
            }
            rate_used = rate_meas + sum_domega;
        }

        const float e = rate_sp - rate_used;

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
            float s = e_now + lambda_i * integ;
            // Dead-band (see the s_deadband field comment above): shrink |s|
            // toward zero by up to s_deadband before anything downstream
            // (boundary layer, reaching law) sees it. Continuous at the
            // band edge (no jump), so it composes cleanly with sat(s/phi).
            // 不感バンド（上のs_deadbandフィールドコメント参照）: 下流
            // （境界層・到達則）に渡す前に|s|をs_deadband分だけゼロへ縮める。
            // 帯の境界で連続（跳躍なし）、sat(s/phi)と自然に合成される。
            if (s_deadband > 1.0e-6f) {
                if (s > s_deadband)       s -= s_deadband;
                else if (s < -s_deadband) s += s_deadband;
                else                      s = 0.0f;
            }
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

        float torque = reachingTorque(e, integral);
        if (torque >  output_limit) torque =  output_limit;
        if (torque < -output_limit) torque = -output_limit;

        // Record this cycle's (already-clamped) torque for the dead-time
        // predictor's ring buffer, regardless of whether delay_comp_s is
        // currently enabled -- so switching it on live (param reload) has a
        // populated history immediately rather than ramping in from zero.
        // このサイクルの（クランプ済み）トルクを、delay_comp_sが現在有効かに
        // 関わらず無駄時間予測補償器のring bufferへ記録する -- ライブで
        // 有効化（paramリロード）した際にゼロから立ち上がるのでなく、
        // 即座に履歴が埋まっているようにするため。
        torque_ring_[torque_ring_idx_] = torque;
        torque_ring_idx_ = (torque_ring_idx_ + 1) % kDelayCompMaxSamples;
        if (torque_ring_count_ < kDelayCompMaxSamples) ++torque_ring_count_;

        return torque;
    }

    /// Reset internal state / 内部状態をリセット
    void reset()
    {
        integral   = 0;
        prev_error = 0;
        for (float& t : torque_ring_) t = 0.0f;
        torque_ring_idx_   = 0;
        torque_ring_count_ = 0;
    }
};

}  // namespace sf::app
