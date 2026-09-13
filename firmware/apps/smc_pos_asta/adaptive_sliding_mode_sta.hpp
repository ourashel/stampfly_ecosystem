/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file adaptive_sliding_mode_sta.hpp
 * @brief GENERIC single-axis SECOND-ORDER sliding-mode controller with an
 *        ADAPTIVE switching gain: the reference-model + trend-based design
 *        from firmware/apps/smc_rate_asta/smc_rate_asta.hpp (docs/plans/
 *        smc-rate-loop-plan.md section 7.36, the final/most-refined design
 *        of that investigation), generalized with an `output_scale` field
 *        the same way firmware/apps/smc_pos_sta/sliding_mode_sta.hpp
 *        generalized the FIXED-gain SuperTwisting struct -- usable for both
 *        a rate axis (output = torque) and a horizontal-velocity axis
 *        (output = acceleration).
 *        適応スイッチングゲイン付き汎用1軸2次スライディングモード制御器:
 *        firmware/apps/smc_rate_asta/smc_rate_asta.hpp（docs/plans/
 *        smc-rate-loop-plan.md §7.36、その調査の最終・最も精緻化された設計）
 *        の規範モデル+トレンド判定方式を、firmware/apps/smc_pos_sta/
 *        sliding_mode_sta.hppが固定ゲイン版SuperTwistingを汎用化したのと
 *        同じ方法で`output_scale`フィールドを持つよう汎用化したもの——
 *        レート軸（出力=トルク）・水平速度軸（出力=加速度）の両方に使える。
 *
 * Motivation / 動機: docs/plans/smc-rate-loop-plan.md's session on
 * 2026-09-13 asked "was choosing sliding mode control effective for BOTH
 * position and angle stabilization in this system?" -- the answer found was
 * that FIXED-gain STA (smc_rate_sta, smc_pos_sta) has been tried for both,
 * and ADAPTIVE-gain STA (smc_rate_asta) has been tried ONLY for the rate
 * loop, never for position/velocity. This app fills that gap: apply the
 * SAME adaptive mechanism (proven, in the rate-loop context, to match or
 * slightly exceed fixed-gain STA -- section 7.33-7.41) to BOTH the rate
 * loop AND the velocity loop, mirroring smc_pos_sta's scope exactly.
 * 2026-09-13のセッションで「この系で位置も角度もスライディングモード制御を
 * 選んだことは有効だったか」という問いへの答えとして、固定ゲインSTA
 * （smc_rate_sta・smc_pos_sta）は両方で試されたが、適応ゲインSTA
 * （smc_rate_asta）はレートループでしか試されていないと判明した。本appは
 * その空白を埋める: 同じ適応機構（レートループの文脈では固定ゲインSTAと
 * 同等かわずかに上回ると実証済み——§7.33-7.41）を、レートループと速度
 * ループの両方に、smc_pos_staと全く同じ範囲で適用する。
 *
 * Everything below (the adaptive law, the reference-model divergence gate,
 * the anti-windup, the leaky z-integration) is an UNMODIFIED port of
 * smc_rate_asta.hpp's `AdaptiveSuperTwistingRate::compute()`/`reset()`,
 * with `inertia` renamed to `output_scale` (matching sliding_mode_sta.hpp's
 * SuperTwisting::output_scale convention) and `sqrtf`/`fabsf` from <cmath>.
 * See smc_rate_asta.hpp for the FULL design-history rationale (sections
 * 7.31-7.36: why a dead-band adaptive law was replaced by a zero-crossing
 * gate, then a reference-model amplitude ratio, then finally this trend
 * (double-EMA) test) -- not repeated here to avoid drift between the two
 * copies; this file only documents the output-scale generalization and the
 * position/velocity application, which are new relative to smc_rate_asta.hpp.
 * 以下（適応則・規範モデル発散ゲート・アンチワインドアップ・zの漏れ積分）は
 * smc_rate_asta.hppの`AdaptiveSuperTwistingRate::compute()`/`reset()`の
 * 無改造移植であり、`inertia`を`output_scale`（sliding_mode_sta.hppの
 * `SuperTwisting::output_scale`規約と同じ）に改名しただけである。設計変遷の
 * 全経緯（§7.31-7.36: 不感帯適応則→ゼロクロスゲート→規範モデル振幅比→
 * 最終的な本トレンド（二重EMA）判定への変遷）はsmc_rate_asta.hpp参照
 * ——2つのコピー間でかい離しないよう本ファイルでは繰り返さない。本ファイルが
 * 文書化するのはoutput_scaleへの汎用化と位置/速度への適用という、
 * smc_rate_asta.hppに対する新規部分のみ。
 *
 * @design docs/plans/smc-rate-loop-plan.md section 7.36 -- trend-based reference-model adaptive STA (rate-loop origin) [OK]
 * @design docs/plans/smc-rate-loop-plan.md 2026-09-13 session -- adaptive STA applied to position/velocity, new for this app [--]
 * @design controller.hpp -- IController interface (used via AppController)  [OK]
 *
 * References / 参考文献 (see smc_rate_asta.hpp for the full citations):
 *   [R5][R6][R7][R8][R9][R10][R11] -- see smc_rate_asta.hpp's file header.
 */

#pragma once

#include <cmath>

namespace sf::app {

/// Generic single-axis adaptive super-twisting (2nd-order sliding-mode)
/// controller, usable for both a rate axis (output_scale = inertia, output
/// = torque) and a horizontal-velocity axis (output_scale = 1.0, output =
/// acceleration). Direct generalization of smc_rate_asta.hpp's
/// AdaptiveSuperTwistingRate -- see that file for the full design rationale.
/// 汎用1軸適応スーパーツイスティング（2次スライディングモード）制御器。
/// レート軸（output_scale=慣性、出力=トルク）・水平速度軸
/// （output_scale=1.0、出力=加速度）の両方に使える。smc_rate_asta.hppの
/// AdaptiveSuperTwistingRateの直接的な汎用化——設計根拠は同ファイル参照。
struct AdaptiveSuperTwisting {
    // Output scaling multiplier -- see sliding_mode_sta.hpp's SuperTwisting
    // ::output_scale for the full rationale (rate axis: inertia [kg*m^2];
    // velocity axis: default 1.0, acceleration IS the output quantity).
    // 出力スケール乗数 -- 根拠はsliding_mode_sta.hppのSuperTwisting::
    // output_scale参照。
    float output_scale = 1.0f;

    // --- Adaptive-gain parameters (identical fields/semantics to
    // smc_rate_asta.hpp's AdaptiveSuperTwistingRate) / 適応ゲインパラメータ
    // （smc_rate_asta.hppのAdaptiveSuperTwistingRateと同一のフィールド/意味）---
    float k1_init    = 30.0f;
    float k1_min     = 15.0f;
    float k1_max     = 150.0f;
    float k2_ratio   = 0.5f;
    float adapt_rate = 20.0f;
    float leak_ratio = 0.2f;
    float dead_band  = 0.05f;
    float filter_tau = 0.05f;

    float mref_tau          = 0.03f;
    float mref_env_tau      = 0.25f;
    float mref_env_base_tau = 0.6f;
    float mref_trend_floor  = 0.02f;
    float mref_shrink_ratio = 1.0f;
    float mref_dwell_time   = 0.06f;

    float k1_slew_max = 1000.0f;

    // --- Same-as-fixed-STA parameters / 固定STAと同じパラメータ ---
    float phi      = 0.02f;
    float lambda_i = 0;
    float e_reset  = 0;
    float z_leak_tau = 0;

    // Output limit [rate axis: Nm; velocity axis: m/s^2]
    // 出力上限 [レート軸: Nm; 速度軸: m/s^2]
    float output_limit = 1.0f;

    // State -- identical set to smc_rate_asta.hpp's AdaptiveSuperTwistingRate.
    // 状態 -- smc_rate_asta.hppのAdaptiveSuperTwistingRateと同一の集合。
    float integral   = 0;
    float z          = 0;
    float prev_error = 0;
    float k1         = 30.0f;
    float s_lpf      = 0;
    float rate_model      = 0;
    float e_model_env      = 0;
    float e_model_env_base = 0;
    float mref_dwell_timer = 0;

    /// Compute the adaptive super-twisting output / 適応スーパーツイスティング出力を計算
    /// @param sp    Target value [rate axis: rad/s; velocity axis: m/s] / 目標値
    /// @param meas  Measured value [same units as sp] / 測定値
    /// @param dt    Time step [s] / タイムステップ
    /// @return      Output, clamped to +/-output_limit [rate axis: Nm; velocity axis: m/s^2]
    float compute(float sp, float meas, float dt)
    {
        const float e = sp - meas;

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

        // Reference model driven by the SAME sp this controller receives --
        // see smc_rate_asta.hpp's compute() for the full rationale.
        // 本コントローラと同じspで駆動される規範モデル -- 根拠は
        // smc_rate_asta.hppのcompute()参照。
        if (dt > 0) {
            const float alpha_model = dt / (mref_tau + dt);
            rate_model += alpha_model * (sp - rate_model);
        }
        const float e_model = meas - rate_model;
        const float abs_e_model = fabsf(e_model);
        if (dt > 0) {
            const float alpha_env = dt / (mref_env_tau + dt);
            e_model_env += alpha_env * (abs_e_model - e_model_env);
            const float alpha_base = dt / (mref_env_base_tau + dt);
            e_model_env_base += alpha_base * (e_model_env - e_model_env_base);
        }
        const bool growing = (e_model_env - e_model_env_base) > mref_trend_floor;
        if (dt > 0) {
            if (growing) {
                mref_dwell_timer += dt;
            } else {
                mref_dwell_timer = 0;
            }
        }
        const bool diverging = growing && (mref_dwell_timer >= mref_dwell_time);

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
            if (k1_dot >  k1_slew_max) k1_dot =  k1_slew_max;
            if (k1_dot < -k1_slew_max) k1_dot = -k1_slew_max;
            k1 += k1_dot * dt;
            if (k1 < k1_min) k1 = k1_min;
            if (k1 > k1_max) k1 = k1_max;
        }
        const float k2 = k2_ratio * k1;

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
            return output_scale * (u1 + z_now);
        };

        const float z_trial = (dt > 0)
            ? z + (-k2 * sign_s_trial - (z_leak_tau > 1.0e-6f ? z / z_leak_tau : 0.0f)) * dt
            : z;

        const float output_trial = staOutput(e, integral_trial, z_trial);

        const bool push_high = (output_trial >  output_limit) && (e > 0);
        const bool push_low  = (output_trial < -output_limit) && (e < 0);
        if (!push_high && !push_low) {
            integral = integral_trial;
            z        = z_trial;
        }

        if (fabsf(output_scale) > 1.0e-12f) {
            const float z_max = output_limit / output_scale;
            if (z >  z_max) z =  z_max;
            if (z < -z_max) z = -z_max;
        }

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

    /// Reset internal state, including re-seeding the adaptive gain from
    /// k1_init and the reference model -- see smc_rate_asta.hpp's reset()
    /// for the full rationale.
    /// 内部状態をリセット。適応ゲインもk1_initから再シードし、規範モデルも
    /// リセットする -- 根拠はsmc_rate_asta.hppのreset()参照。
    void reset()
    {
        integral      = 0;
        z             = 0;
        prev_error    = 0;
        k1            = k1_init;
        s_lpf         = 0;
        rate_model       = 0;
        e_model_env      = 0;
        e_model_env_base = 0;
        mref_dwell_timer = 0;
    }
};

}  // namespace sf::app
