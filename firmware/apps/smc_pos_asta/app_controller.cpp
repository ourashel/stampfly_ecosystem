/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.cpp
 * @brief See app_controller.hpp for what this class is for.
 *        このクラスの目的は app_controller.hpp を参照。
 *
 * @design controller.hpp — IController interface (12 methods)   [OK]
 * @design docs/plans/smc-rate-loop-plan.md section 7.36 / 2026-09-13 session — adaptive STA for rate+velocity loops [--]
 */

#include "app_controller.hpp"
#include "params.hpp"
#include "sf_math.hpp"
#include "esp_log.h"  // TEMPORARY diagnostic, §7.56続報2 -- remove with the posdiag probe below
#include <algorithm>  // std::clamp (vsp_slew_max_ rate limiter, §7.54続報6)

namespace sf::app {

// TEMPORARY diagnostic tag (docs/plans/smc-rate-loop-plan.md §7.56続報2) -- see
// posdiag_counter_'s doc comment in app_controller.hpp.
static const char* TAG = "SMC_POS_ASTA";

// X-quad spec inertia Ixx/Iyy/Izz [kg*m^2] -- same value and same caveat as
// firmware/apps/smc_pos_sta/app_controller.cpp's kInertia.
// X-quad仕様慣性 Ixx/Iyy/Izz [kg*m^2] -- firmware/apps/smc_pos_sta/
// app_controller.cppのkInertiaと同じ値・同じ注意点。
static const float kInertia[3] = {9.16e-6f, 13.3e-6f, 20.4e-6f};  // roll, pitch, yaw

// POS_HOLD tilt limit [rad] -- same value/rationale as
// firmware/apps/smc_pos_sta/app_controller.cpp's kMaxPosTilt.
// POS_HOLDの傾き上限 [rad] -- firmware/apps/smc_pos_sta/app_controller.cppの
// kMaxPosTiltと同じ値・根拠。
static constexpr float kMaxPosTilt = 0.1745f;  // [rad] (10 deg)

void AppController::init()
{
    pid_.init();
    smc_roll_.output_scale  = kInertia[0];
    smc_pitch_.output_scale = kInertia[1];
    smc_yaw_.output_scale   = kInertia[2];
    // smc_vel_x_/smc_vel_y_ keep the struct's default output_scale=1.0.
    // smc_vel_x_/smc_vel_y_は構造体既定のoutput_scale=1.0のまま。
    loadRateSmcParams();
    loadVelSmcParams();
    // reset() seeds k1 from k1_init -- must run AFTER load*SmcParams() so
    // k1_init is the freshly-loaded value, not the struct's compile-time
    // default (same requirement as firmware/apps/smc_rate_asta's init()).
    // reset()はk1_initからk1をシードする——load*SmcParams()の後に実行し、
    // k1_initが構造体のコンパイル時既定値でなく読み込み済みの値になるように
    // する（firmware/apps/smc_rate_astaのinit()と同じ要件）。
    smc_roll_.reset();
    smc_pitch_.reset();
    smc_yaw_.reset();
    smc_vel_x_.reset();
    smc_vel_y_.reset();

    // Plug the velocity-loop adaptive STA into pid_'s position cascade --
    // see firmware/apps/smc_pos_sta/app_controller.cpp's init() for the full
    // rationale (unchanged here).
    // 速度適応STAをpid_の位置カスケードへ注入する -- 詳細な根拠は
    // firmware/apps/smc_pos_sta/app_controller.cppのinit()参照
    // （ここでは無変更）。
    pid_.setVelocityLawOverride(
        [this](float vx_sp, float vx, float vy_sp, float vy, float dt,
               float& ax_ned, float& ay_ned) {
            // Optional slew-rate limit on the STA's own target -- see
            // vsp_slew_max_'s doc comment (app_controller.hpp) for the
            // §7.54続報6 rationale. 0 (default) skips this entirely, so the
            // raw vx_sp/vy_sp reach the STA exactly as before this change.
            // STA自身の目標へのスルーレート制限（任意）——根拠は
            // vsp_slew_max_のドキュメントコメント参照（app_controller.hpp、
            // §7.54続報6）。0（既定）ならこのブロック自体を素通りし、この
            // 変更前と全く同じ生のvx_sp/vy_spがSTAへ渡る。
            float vx_target = vx_sp;
            float vy_target = vy_sp;
            if (vsp_slew_max_ > 1.0e-9f && dt > 0.0f) {
                const float step = vsp_slew_max_ * dt;
                vx_sp_limited_ += std::clamp(vx_sp - vx_sp_limited_, -step, step);
                vy_sp_limited_ += std::clamp(vy_sp - vy_sp_limited_, -step, step);
                vx_target = vx_sp_limited_;
                vy_target = vy_sp_limited_;
            }
            ax_ned = smc_vel_x_.compute(vx_target, vx, dt);
            ay_ned = smc_vel_y_.compute(vy_target, vy, dt);

            // TEMPORARY diagnostic (§7.56続報2): is smc_vel_y_'s own adaptive k1
            // what self-activates during the long-hold "3rd loop" symptom? Same
            // 0.05s decimation convention as pid_controller.cpp's posdiag A/B.
            // 一時診断（§7.56続報2）: smc_vel_y_自身の適応k1が、長時間保持の
            // 「第3のループ」症状で自己活性化しているか。pid_controller.cppの
            // posdiag A/Bと同じ0.05s間引き規約。
            if (++posdiag_counter_ >= 20) {
                posdiag_counter_ = 0;
                ESP_LOGI(TAG, "posdiag E k1=%.4f e_model_env=%.4f e_model_env_base=%.4f "
                         "z=%.4f s_lpf=%.4f ay_ned=%.4f vy=%.4f",
                         static_cast<double>(smc_vel_y_.k1),
                         static_cast<double>(smc_vel_y_.e_model_env),
                         static_cast<double>(smc_vel_y_.e_model_env_base),
                         static_cast<double>(smc_vel_y_.z),
                         static_cast<double>(smc_vel_y_.s_lpf),
                         static_cast<double>(ay_ned), static_cast<double>(vy));
            }
        });
}

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    // See firmware/apps/smc_pos_sta/app_controller.cpp's compute() for the
    // full rationale (identical here, only the rate/velocity laws differ).
    // 詳細な根拠はfirmware/apps/smc_pos_sta/app_controller.cppのcompute()
    // 参照（ここではレート/速度則だけが異なる）。
    sf::ControlOutput output = pid_.compute(state, setpoint, dt);

    output.torque[0] = smc_roll_.compute(output.rate_ref[0], state.angular_rate[0], dt);
    output.torque[1] = smc_pitch_.compute(output.rate_ref[1], state.angular_rate[1], dt);
    output.torque[2] = smc_yaw_.compute(output.rate_ref[2], state.angular_rate[2], dt);

    return output;
}

void AppController::loadRateSmcParams()
{
    sf::params::get_float("smc_pos_asta.roll.k1_init",    smc_roll_.k1_init);
    sf::params::get_float("smc_pos_asta.roll.k1_min",     smc_roll_.k1_min);
    sf::params::get_float("smc_pos_asta.roll.k1_max",     smc_roll_.k1_max);
    sf::params::get_float("smc_pos_asta.roll.k2_ratio",   smc_roll_.k2_ratio);
    sf::params::get_float("smc_pos_asta.roll.adapt_rate", smc_roll_.adapt_rate);
    sf::params::get_float("smc_pos_asta.roll.leak_ratio", smc_roll_.leak_ratio);
    sf::params::get_float("smc_pos_asta.roll.dead_band",  smc_roll_.dead_band);
    sf::params::get_float("smc_pos_asta.roll.filter_tau", smc_roll_.filter_tau);
    sf::params::get_float("smc_pos_asta.roll.mref_tau",          smc_roll_.mref_tau);
    sf::params::get_float("smc_pos_asta.roll.mref_env_tau",      smc_roll_.mref_env_tau);
    sf::params::get_float("smc_pos_asta.roll.mref_env_base_tau", smc_roll_.mref_env_base_tau);
    sf::params::get_float("smc_pos_asta.roll.mref_trend_floor",  smc_roll_.mref_trend_floor);
    sf::params::get_float("smc_pos_asta.roll.mref_shrink_ratio", smc_roll_.mref_shrink_ratio);
    sf::params::get_float("smc_pos_asta.roll.mref_dwell_time",   smc_roll_.mref_dwell_time);
    sf::params::get_float("smc_pos_asta.roll.k1_slew_max",       smc_roll_.k1_slew_max);
    sf::params::get_float("smc_pos_asta.roll.phi",        smc_roll_.phi);
    sf::params::get_float("smc_pos_asta.roll.lambda_i",   smc_roll_.lambda_i);
    sf::params::get_float("smc_pos_asta.roll.e_reset",    smc_roll_.e_reset);
    sf::params::get_float("smc_pos_asta.roll.z_leak_tau", smc_roll_.z_leak_tau);
    sf::params::get_float("smc_pos_asta.roll.predictor_tau_m",    smc_roll_.predictor_tau_m);
    sf::params::get_float("smc_pos_asta.roll.predictor_leak_tau", smc_roll_.predictor_leak_tau);

    sf::params::get_float("smc_pos_asta.pitch.k1_init",    smc_pitch_.k1_init);
    sf::params::get_float("smc_pos_asta.pitch.k1_min",     smc_pitch_.k1_min);
    sf::params::get_float("smc_pos_asta.pitch.k1_max",     smc_pitch_.k1_max);
    sf::params::get_float("smc_pos_asta.pitch.k2_ratio",   smc_pitch_.k2_ratio);
    sf::params::get_float("smc_pos_asta.pitch.adapt_rate", smc_pitch_.adapt_rate);
    sf::params::get_float("smc_pos_asta.pitch.leak_ratio", smc_pitch_.leak_ratio);
    sf::params::get_float("smc_pos_asta.pitch.dead_band",  smc_pitch_.dead_band);
    sf::params::get_float("smc_pos_asta.pitch.filter_tau", smc_pitch_.filter_tau);
    sf::params::get_float("smc_pos_asta.pitch.mref_tau",          smc_pitch_.mref_tau);
    sf::params::get_float("smc_pos_asta.pitch.mref_env_tau",      smc_pitch_.mref_env_tau);
    sf::params::get_float("smc_pos_asta.pitch.mref_env_base_tau", smc_pitch_.mref_env_base_tau);
    sf::params::get_float("smc_pos_asta.pitch.mref_trend_floor",  smc_pitch_.mref_trend_floor);
    sf::params::get_float("smc_pos_asta.pitch.mref_shrink_ratio", smc_pitch_.mref_shrink_ratio);
    sf::params::get_float("smc_pos_asta.pitch.mref_dwell_time",   smc_pitch_.mref_dwell_time);
    sf::params::get_float("smc_pos_asta.pitch.k1_slew_max",       smc_pitch_.k1_slew_max);
    sf::params::get_float("smc_pos_asta.pitch.phi",        smc_pitch_.phi);
    sf::params::get_float("smc_pos_asta.pitch.lambda_i",   smc_pitch_.lambda_i);
    sf::params::get_float("smc_pos_asta.pitch.e_reset",    smc_pitch_.e_reset);
    sf::params::get_float("smc_pos_asta.pitch.z_leak_tau", smc_pitch_.z_leak_tau);
    sf::params::get_float("smc_pos_asta.pitch.predictor_tau_m",    smc_pitch_.predictor_tau_m);
    sf::params::get_float("smc_pos_asta.pitch.predictor_leak_tau", smc_pitch_.predictor_leak_tau);

    sf::params::get_float("smc_pos_asta.yaw.k1_init",    smc_yaw_.k1_init);
    sf::params::get_float("smc_pos_asta.yaw.k1_min",     smc_yaw_.k1_min);
    sf::params::get_float("smc_pos_asta.yaw.k1_max",     smc_yaw_.k1_max);
    sf::params::get_float("smc_pos_asta.yaw.k2_ratio",   smc_yaw_.k2_ratio);
    sf::params::get_float("smc_pos_asta.yaw.adapt_rate", smc_yaw_.adapt_rate);
    sf::params::get_float("smc_pos_asta.yaw.leak_ratio", smc_yaw_.leak_ratio);
    sf::params::get_float("smc_pos_asta.yaw.dead_band",  smc_yaw_.dead_band);
    sf::params::get_float("smc_pos_asta.yaw.filter_tau", smc_yaw_.filter_tau);
    sf::params::get_float("smc_pos_asta.yaw.mref_tau",          smc_yaw_.mref_tau);
    sf::params::get_float("smc_pos_asta.yaw.mref_env_tau",      smc_yaw_.mref_env_tau);
    sf::params::get_float("smc_pos_asta.yaw.mref_env_base_tau", smc_yaw_.mref_env_base_tau);
    sf::params::get_float("smc_pos_asta.yaw.mref_trend_floor",  smc_yaw_.mref_trend_floor);
    sf::params::get_float("smc_pos_asta.yaw.mref_shrink_ratio", smc_yaw_.mref_shrink_ratio);
    sf::params::get_float("smc_pos_asta.yaw.mref_dwell_time",   smc_yaw_.mref_dwell_time);
    sf::params::get_float("smc_pos_asta.yaw.k1_slew_max",       smc_yaw_.k1_slew_max);
    sf::params::get_float("smc_pos_asta.yaw.phi",        smc_yaw_.phi);
    sf::params::get_float("smc_pos_asta.yaw.lambda_i",   smc_yaw_.lambda_i);
    sf::params::get_float("smc_pos_asta.yaw.e_reset",    smc_yaw_.e_reset);
    sf::params::get_float("smc_pos_asta.yaw.z_leak_tau", smc_yaw_.z_leak_tau);
    sf::params::get_float("smc_pos_asta.yaw.predictor_tau_m",    smc_yaw_.predictor_tau_m);
    sf::params::get_float("smc_pos_asta.yaw.predictor_leak_tau", smc_yaw_.predictor_leak_tau);

    // Same physical torque ceiling the PID rate loop uses -- see
    // firmware/apps/smc_pos_sta/app_controller.cpp's loadRateSmcParams().
    // レートPIDと同じ物理トルク上限 -- 出典は
    // firmware/apps/smc_pos_sta/app_controller.cppのloadRateSmcParams()参照。
    constexpr float kMaxRollPitchTorque = 5.2e-3f;  // [Nm], mirrors pid_controller.hpp
    smc_roll_.output_limit  = kMaxRollPitchTorque;
    smc_pitch_.output_limit = kMaxRollPitchTorque;
    sf::params::get_float("rate.yaw.max_torque", smc_yaw_.output_limit);
}

void AppController::loadVelSmcParams()
{
    sf::params::get_float("smc_pos_asta.velx.k1_init",    smc_vel_x_.k1_init);
    sf::params::get_float("smc_pos_asta.velx.k1_min",     smc_vel_x_.k1_min);
    sf::params::get_float("smc_pos_asta.velx.k1_max",     smc_vel_x_.k1_max);
    sf::params::get_float("smc_pos_asta.velx.k2_ratio",   smc_vel_x_.k2_ratio);
    sf::params::get_float("smc_pos_asta.velx.adapt_rate", smc_vel_x_.adapt_rate);
    sf::params::get_float("smc_pos_asta.velx.leak_ratio", smc_vel_x_.leak_ratio);
    sf::params::get_float("smc_pos_asta.velx.dead_band",  smc_vel_x_.dead_band);
    sf::params::get_float("smc_pos_asta.velx.filter_tau", smc_vel_x_.filter_tau);
    sf::params::get_float("smc_pos_asta.velx.mref_tau",          smc_vel_x_.mref_tau);
    sf::params::get_float("smc_pos_asta.velx.mref_env_tau",      smc_vel_x_.mref_env_tau);
    sf::params::get_float("smc_pos_asta.velx.mref_env_base_tau", smc_vel_x_.mref_env_base_tau);
    sf::params::get_float("smc_pos_asta.velx.mref_trend_floor",  smc_vel_x_.mref_trend_floor);
    sf::params::get_float("smc_pos_asta.velx.mref_shrink_ratio", smc_vel_x_.mref_shrink_ratio);
    sf::params::get_float("smc_pos_asta.velx.mref_dwell_time",   smc_vel_x_.mref_dwell_time);
    sf::params::get_float("smc_pos_asta.velx.k1_slew_max",       smc_vel_x_.k1_slew_max);
    sf::params::get_float("smc_pos_asta.velx.phi",        smc_vel_x_.phi);
    sf::params::get_float("smc_pos_asta.velx.lambda_i",   smc_vel_x_.lambda_i);
    sf::params::get_float("smc_pos_asta.velx.e_reset",    smc_vel_x_.e_reset);
    sf::params::get_float("smc_pos_asta.velx.z_leak_tau", smc_vel_x_.z_leak_tau);
    sf::params::get_float("smc_pos_asta.velx.predictor_tau_m",    smc_vel_x_.predictor_tau_m);
    sf::params::get_float("smc_pos_asta.velx.predictor_leak_tau", smc_vel_x_.predictor_leak_tau);

    sf::params::get_float("smc_pos_asta.vely.k1_init",    smc_vel_y_.k1_init);
    sf::params::get_float("smc_pos_asta.vely.k1_min",     smc_vel_y_.k1_min);
    sf::params::get_float("smc_pos_asta.vely.k1_max",     smc_vel_y_.k1_max);
    sf::params::get_float("smc_pos_asta.vely.k2_ratio",   smc_vel_y_.k2_ratio);
    sf::params::get_float("smc_pos_asta.vely.adapt_rate", smc_vel_y_.adapt_rate);
    sf::params::get_float("smc_pos_asta.vely.leak_ratio", smc_vel_y_.leak_ratio);
    sf::params::get_float("smc_pos_asta.vely.dead_band",  smc_vel_y_.dead_band);
    sf::params::get_float("smc_pos_asta.vely.filter_tau", smc_vel_y_.filter_tau);
    sf::params::get_float("smc_pos_asta.vely.mref_tau",          smc_vel_y_.mref_tau);
    sf::params::get_float("smc_pos_asta.vely.mref_env_tau",      smc_vel_y_.mref_env_tau);
    sf::params::get_float("smc_pos_asta.vely.mref_env_base_tau", smc_vel_y_.mref_env_base_tau);
    sf::params::get_float("smc_pos_asta.vely.mref_trend_floor",  smc_vel_y_.mref_trend_floor);
    sf::params::get_float("smc_pos_asta.vely.mref_shrink_ratio", smc_vel_y_.mref_shrink_ratio);
    sf::params::get_float("smc_pos_asta.vely.mref_dwell_time",   smc_vel_y_.mref_dwell_time);
    sf::params::get_float("smc_pos_asta.vely.k1_slew_max",       smc_vel_y_.k1_slew_max);
    sf::params::get_float("smc_pos_asta.vely.phi",        smc_vel_y_.phi);
    sf::params::get_float("smc_pos_asta.vely.lambda_i",   smc_vel_y_.lambda_i);
    sf::params::get_float("smc_pos_asta.vely.e_reset",    smc_vel_y_.e_reset);
    sf::params::get_float("smc_pos_asta.vely.z_leak_tau", smc_vel_y_.z_leak_tau);
    sf::params::get_float("smc_pos_asta.vely.predictor_tau_m",    smc_vel_y_.predictor_tau_m);
    sf::params::get_float("smc_pos_asta.vely.predictor_leak_tau", smc_vel_y_.predictor_leak_tau);

    // Same physical acceleration ceiling PidController's vel_x_/vel_y_ used
    // before being overridden -- see firmware/apps/smc_pos_sta/
    // app_controller.cpp's loadVelSmcParams() for the full rationale.
    // 差し替え前にPidControllerのvel_x_/vel_y_が使っていたのと同じ物理加速度
    // 上限 -- 詳細な根拠はfirmware/apps/smc_pos_sta/app_controller.cppの
    // loadVelSmcParams()参照。
    const float output_limit = sf::math::kGravity * kMaxPosTilt;
    smc_vel_x_.output_limit = output_limit;
    smc_vel_y_.output_limit = output_limit;

    // §7.54続報6 diagnostic/experimental slew-rate limiter -- see
    // vsp_slew_max_'s doc comment (app_controller.hpp). Default 0 (OFF).
    // §7.54続報6の診断・実験用スルーレート制限——根拠はvsp_slew_max_の
    // ドキュメントコメント参照（app_controller.hpp）。既定0（OFF）。
    sf::params::get_float("smc_pos_asta.vel.sp_slew_max", vsp_slew_max_);
}

void AppController::reset()
{
    pid_.reset();
    smc_roll_.reset();
    smc_pitch_.reset();
    smc_yaw_.reset();
    smc_vel_x_.reset();
    smc_vel_y_.reset();
    vx_sp_limited_ = 0.0f;
    vy_sp_limited_ = 0.0f;
}

void AppController::onModeChange(sf::FlightMode new_mode)
{
    // Reset the velocity-loop adaptive STA instances on every POS_HOLD-
    // boundary mode transition -- carries forward the fix that
    // firmware/apps/smc_pos_sta/app_controller.cpp's onModeChange() added
    // AFTER a real-hardware crash (docs/plans/smc-rate-loop-plan.md
    // §7.27/§7.28: stale integral/z state surviving a mode transition,
    // combined with a freshly re-captured position target, produced rate-
    // tracking error spikes >10x). Applied here FROM THE START (this app
    // has not yet been real-hardware-tested) rather than waiting to
    // rediscover the same failure. Also resets k1/the reference model
    // (AdaptiveSuperTwisting::reset() does both, unlike the fixed-gain
    // SuperTwisting::reset() smc_pos_sta uses) -- a stale adapted k1 or
    // reference-model state carried across a mode boundary is at least as
    // suspect as the stale integral/z that caused the original crash.
    // Rate loop (smc_roll_/smc_pitch_/smc_yaw_) is NOT reset here, matching
    // smc_pos_sta/smc_rate_asta's same design intent (mode-agnostic
    // innermost loop).
    // POS_HOLD境界を跨ぐ全てのモード遷移で速度適応STAインスタンスをリセット
    // する -- firmware/apps/smc_pos_sta/app_controller.cppのonModeChange()が
    // 実機クラッシュ後に追加した修正（docs/plans/smc-rate-loop-plan.md
    // §7.27/§7.28: モード遷移をまたいで残った古い積分/z状態と、新たに
    // 再捕捉された位置目標が組み合わさり、レート追従誤差が10倍超に跳ね上がった）
    // を最初から引き継ぐ（本appはまだ実機投入していないため、同じ失敗を
    // 再発見してから直すのではなく）。k1・規範モデルもリセットする
    // （`AdaptiveSuperTwisting::reset()`は両方リセットする——smc_pos_staが
    // 使う固定ゲイン`SuperTwisting::reset()`とは異なる）——モード境界を
    // またいで持ち越された適応済みk1や規範モデル状態も、元のクラッシュを
    // 招いた古い積分/z状態と少なくとも同程度に疑わしい。レートループ
    // （smc_roll_/smc_pitch_/smc_yaw_）はここではリセットしない
    // ——smc_pos_sta/smc_rate_astaと同じ設計意図（モード非依存の最内周ループ）。
    smc_vel_x_.reset();
    smc_vel_y_.reset();
    // The slew-limiter's internal state is a target-tracking filter, not a
    // controller-internal integral/adaptation state -- reset it alongside
    // the STA it feeds for the same reason (a stale limited-target value
    // from before the mode boundary would otherwise force an artificial
    // ramp toward the freshly-recaptured target).
    // スルーレート制限器の内部状態は目標追従フィルタであり制御則内部の
    // 積分/適応状態ではないが、それが供給するSTAと同じ理由でモード境界越しに
    // リセットする（さもないと、境界前の古い制限済み目標値から新たに
    // 再捕捉された目標へ向かう不自然なランプが生じる）。
    vx_sp_limited_ = 0.0f;
    vy_sp_limited_ = 0.0f;
    pid_.onModeChange(new_mode);
}

void AppController::onLanding()
{
    pid_.onLanding();
}

void AppController::onTakeoff()
{
    pid_.onTakeoff();
}

void AppController::onTakeoffComplete()
{
    pid_.onTakeoffComplete();
}

bool AppController::isTakeoffComplete() const
{
    return pid_.isTakeoffComplete();
}

void AppController::setGuidanceTarget(const sf::GuidanceTarget& target,
                                       const sf::CommandSetpoint& current_sticks)
{
    pid_.setGuidanceTarget(target, current_sticks);
}

bool AppController::isGuidanceActive() const
{
    return pid_.isGuidanceActive();
}

void AppController::startExcitation(const sf::SysidCommand& cmd)
{
    // Forwarded unchanged -- see firmware/apps/smc_pos_sta/app_controller.cpp's
    // startExcitation() for the full rationale.
    // 無改造で転送 -- 詳細な根拠はfirmware/apps/smc_pos_sta/app_controller.cpp
    // のstartExcitation()参照。
    pid_.startExcitation(cmd);
}

bool AppController::fetchSysidResult(sf::SysidFreqResult& out)
{
    return pid_.fetchSysidResult(out);
}

void AppController::reloadParams()
{
    pid_.reloadParams();
    loadRateSmcParams();
    loadVelSmcParams();
}

}  // namespace sf::app
