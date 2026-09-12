/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file params.cpp
 * @brief Parameter system and topic instance implementation
 *        パラメータシステムおよびトピックインスタンス実装
 *
 * @design detailed_design.md §6 — Parameter system                    [OK]
 * @design requirements.md §3 — Parameter management                   [OK]
 */

#include "topics.hpp"
#include "params.hpp"
#include "esp_log.h"
#include "esp_timer.h"   // reload-callback timestamps / 再読込コールバックの時刻印
#include "nvs_flash.h"
#include "nvs.h"
#include <cstring>
#include <cstdio>   // snprintf (NVS key derivation) / snprintf（NVSキー導出）
#include <cmath>

static const char* TAG = "Params";

namespace sf {

// =============================================================================
// Topic instances (defined here, declared extern in topics.hpp)
// トピックインスタンス（ここで定義、topics.hppでextern宣言）
// =============================================================================

Topic<ImuData,         RingBuffer, 8>  sensor_imu;
Topic<TofData,         Queue, 2>       sensor_tof;
Topic<FlowData,        Queue, 2>       sensor_flow;
Topic<MagData,         Queue, 2>       sensor_mag;
Topic<BaroData,        Queue, 2>       sensor_baro;
Topic<PowerData,       Latest, 1>      sensor_power;
Topic<SensorSnapshot,  Latest, 1>      sensor_snapshot;
Topic<StateEstimate,   Latest, 1>      estimate_state;
Topic<CommandSetpoint, Latest, 1>      command_setpoint;
Topic<PilotRequest,    Latest, 1>      pilot_request;
Topic<ButtonEvent,     Queue, 4>       button_event;
Topic<ControlOutput,   Latest, 1>      control_output;
Topic<ControllerStatus, Latest, 1>     controller_status;
Topic<MotorOutput,     Latest, 1>      actuator_motor;
Topic<LogStreamSample, RingBuffer, 32> log_stream;
Topic<FlowData,        RingBuffer, 8>  log_flow;
Topic<SystemMode,      Latest, 1>      system_mode;
Topic<SystemAlert,     Queue, 4>       system_alert;
Topic<SystemStatus,    Latest, 1>      system_status;
Topic<PairingStatus,   Latest, 1>      pairing_state;
Topic<PairingComplete, Latest, 1>      pairing_complete;
Topic<PairingDiag,     Latest, 1>      pairing_diag;
Topic<UiCommand,       Queue, 4>       ui_command;
Topic<MotorTest,       Latest, 1>      motor_test;
Topic<MagCalCommand,   Queue,  2>      mag_command;
Topic<ApiCommand,      Queue,  4>      api_command;
Topic<SysidCommand,    Queue,  2>      sysid_command;
Topic<SysidFreqResult, Latest, 1>      sysid_result;
Topic<MagCalStatus,    Latest, 1>      mag_cal_status;
Topic<EstimatorCommand,  Queue, 4>     estimator_command;
Topic<ControllerCommand, Queue, 4>     controller_command;
Topic<NotifyCommand,     Queue, 8>     notify_command;
Topic<SensorHealth,      Latest, 1>    sensor_health;
Topic<GuidanceTarget,    Latest, 1>    command_target;
Topic<NavigationPath,    Queue, 4>     nav_path;

void topics_init()
{
    sensor_imu.init();
    sensor_tof.init();
    sensor_flow.init();
    sensor_mag.init();
    sensor_baro.init();
    sensor_power.init();
    sensor_snapshot.init();
    estimate_state.init();
    command_setpoint.init();
    pilot_request.init();
    button_event.init();
    control_output.init();
    controller_status.init();
    actuator_motor.init();
    log_stream.init();
    log_flow.init();
    system_mode.init();
    system_alert.init();
    system_status.init();
    pairing_state.init();
    pairing_complete.init();
    pairing_diag.init();
    ui_command.init();
    motor_test.init();
    mag_command.init();
    api_command.init();
    sysid_command.init();
    sysid_result.init();
    mag_cal_status.init();
    estimator_command.init();
    controller_command.init();
    notify_command.init();
    sensor_health.init();
    command_target.init();
    nav_path.init();
}

// =============================================================================
// Parameter System Implementation
// パラメータシステム実装
//
// @design detailed_design.md §6 — Parameter table (SSOT = params.cpp) [OK]
// =============================================================================

// Explicit parameter variable definitions
// 明示的なパラメータ変数定義
namespace param_vars {
    // Rate control
    // Rate gains are PHYSICAL [Nm/(rad/s)] (the mixer is a B^-1 allocation,
    // actuator.cpp). Values are the FLIGHT-PROVEN legacy vehicle/ gains
    // (config.hpp rate_control, physical-units mode) — directly transferable
    // because both firmwares share the same loop structure (Tustin PID with
    // D-on-M, η=0.125), the same B^-1 mixer geometry (d=0.023 m, and the mixer's
    // then-assumed κ=0.00971 — see the 2026-07-17 κ-correction note below) and the
    // same motor curve, so the plant seen by the rate loop is identical.
    // Earlier SILS-derived near-P values (kp = I/τ_resp, ti=20) are superseded.
    // レートゲインは物理 [Nm/(rad/s)]（ミキサーは B^-1 配分）。値は旧 vehicle/ の
    // 「飛行実績ゲイン」（config.hpp rate_control 物理単位モード）— 両ファームは
    // ループ構造（Tustin PID・測定値微分・η=0.125）、ミキサー幾何（d=0.023m と
    // 当時のミキサー仮定 κ=0.00971 — 下の 2026-07-17 κ補正ノート参照）、
    // モータ曲線が同一で、レートループから見たプラントが同じため
    // そのまま移植できる。以前の SILS 由来 near-P 値（kp=I/τ_resp, ti=20）は置換。
    // Values are the original M5StampFly (M5Fly-kanazawa) hand-tuned ACRO rate gains,
    // CONVERTED into this firmware's torque[Nm] form, then VALIDATED in real flight
    // (2026-06-27, converted gains flew well → adopted as default). Conversion bridges
    // the two firmwares' output representations: original output is a motor "voltage"
    // (linear duty = V/V_batt mixer); here the PID output is body torque [Nm] (B^-1 +
    // omega^2 motor curve). Same rate-error input (rad/s), same loop form (Tustin PID,
    // D-on-M, eta=0.125, 400Hz), so Ti/Td transfer 1:1 and only Kp is rescaled by
    // d*g_VT (roll/pitch) or kappa*g_VT (yaw), g_VT=dT/dV@hover=0.0653 N/V. Supersedes
    // the 2026-06-19 on-board autotune values (roll 3.40e-4/ti0.4, pitch 5.16e-4/ti0.4,
    // yaw 5.31e-3/ti1.6): vs those, roll/pitch P is ~2.8x with a longer Ti (more low-freq
    // phase margin in the ~0.9 Hz band), yaw is gentler. See analysis/scripts/
    // acro_gain_conversion.py + the SILS/linear-margin validation report.
    // 値はオリジナル M5StampFly（M5Fly-kanazawa）のハンドチューン ACRO レートゲインを本ファームの
    // トルク[Nm]形へ換算し、実機飛行で検証（2026-06-27 良好→既定採用）。換算は両ファームの出力
    // 表現の橋渡し（オリジナル＝モータ「電圧」線形ミキサ、本機＝トルク[Nm] の B^-1＋ω²曲線）。
    // レート誤差入力（rad/s）・ループ形式（Tustin PID・測定値微分・η=0.125・400Hz）が同一ゆえ
    // Ti/Td はそのまま、Kp のみ d*g_VT（roll/pitch）/ κ*g_VT（yaw）で再尺度（g_VT=0.0653 N/V）。
    // 2026-06-19 autotune 値を置換: roll/pitch は P が約2.8倍＋Ti 長め（~0.9Hz帯の低周波余裕増）、yaw は穏やか。
    // 2026-07-18 roll MANUAL retune (pilot, in-flight hand tuning after the
    // wall-crash session): kp=1.0e-3, td=0.001 — "extremely stable" by pilot
    // assessment. This SUPERSEDES the 2026-07-17 study retune (kp 1.268773e-3,
    // td 0.02) which had shown a 3-8 Hz roughness reduction at the time but
    // whose flights later measured 2.4-2.7x the adoption-time reference even
    // with identical gains (session-to-session drift — craft condition and/or
    // environment; see the 2026-07-18 wall-crash diagnosis). td=0.001 at 400 Hz
    // is effectively D-off (incomplete-derivative alpha≈0.1), i.e. the pilot's
    // stable point is a near-PI roll rate loop. History: study retune 07-17 ←
    // M5StampFly-converted 9.759795e-4 (real-flight validated 2026-06-27) ←
    // autotune 3.40e-4.
    // 2026-07-18 ロール手動再調整（パイロット、壁衝突セッション後の飛行中ハンド
    // チューニング）: kp=1.0e-3, td=0.001 —「極めて安定」（パイロット評価）。
    // 2026-07-17 のスタディ再調整（kp 1.268773e-3, td 0.02）を置換する。同再調整は
    // 当時 3-8Hz ざらつき低減を示したが、その後同一ゲインのまま採用時参照の
    // 2.4-2.7倍へ悪化（セッション間ドリフト=機体コンディション/環境差。2026-07-18
    // 壁衝突診断参照）。td=0.001 は 400Hz では実質 D オフ（不完全微分 α≈0.1）＝
    // パイロットの安定点はほぼ PI のレートループ。履歴: スタディ再調整 07-17 ←
    // M5StampFly 換算 9.759795e-4（2026-06-27 実機検証）← autotune 3.40e-4。
    // 2026-07-20 pilot direction for the v2026.07.2 release default:
    // td 0.001 -> 0.002 (kp/ti unchanged). Still near-D-off at 400 Hz,
    // with twice the derivative time of the 07-18 hand-tune.
    // 2026-07-20 パイロット指示（v2026.07.2 リリース既定値）: td 0.001→0.002
    // （kp/ti は変更なし）。400Hz では依然ほぼ D オフだが、07-18 ハンド
    // チューニングの2倍の微分時間。
    float rate_roll_kp    = 1.0e-3f;   // pilot manual retune 2026-07-18
    float rate_roll_ti    = 0.7f;
    float rate_roll_td    = 0.002f;    // pilot direction 2026-07-20 (release default)
    float rate_pitch_kp   = 1.426432e-3f;  // M5StampFly-converted, real-flight validated 2026-06-27 (was autotune 5.16e-4)
    float rate_pitch_ti   = 0.7f;
    float rate_pitch_td   = 0.025f;
    // 2026-07-17 κ correction: the mixer's torque/thrust ratio was fixed to the
    // MEASURED κ=6.12e-3 m (was 9.71e-3; actuator.cpp KAPPA). With the old κ the
    // mixer delivered only κ_true/κ_mixer = 0.6303 of the commanded yaw torque, so
    // the flight-proven PHYSICAL yaw loop gain was 1.901691e-3 × 0.6303. To keep
    // that exact loop gain with the corrected mixer, kp is rescaled by κ_new/κ_old:
    //   1.901691e-3 × (6.12e-3 / 9.71e-3) = 1.198594e-3  [Nm/(rad/s)]
    // Ti/Td are time constants — unaffected by the κ scale. Roll/pitch use the arm
    // d (unchanged), so only yaw is rescaled. On REAL hardware `param reset` (or
    // re-setting rate.yaw.* explicitly) is REQUIRED after flashing this change: an
    // NVS-saved old kp would run 1.59× the proven loop gain on the corrected mixer.
    //
    // 2026-08-03 κ rescale (PURE rescale, not a new physical finding): Ct's
    // 2026-07-15 thrust-stand value was retracted (no valid simultaneous
    // voltage/RPM/thrust measurement exists for the new propeller), and the
    // provisional Ct restores κ=Cq/Ct to 4.10e-3 (actuator.cpp KAPPA; see its
    // comment for the full provenance chain). Following the SAME
    // physical-gain-preserving pattern as 2026-07-17, kp is rescaled by the
    // exact ratio scale = 4.10/6.12 = 0.6699346:
    //   1.198594e-3 × 0.6699346 = 8.029796e-4  [Nm/(rad/s)]
    // This keeps the closed-loop duty output IDENTICAL to before 2026-08-03 —
    // the mixer's KAPPA and this kp move together, so physical yaw torque per
    // commanded rate error is unchanged. `param reset` (or explicit
    // rate.yaw.* re-set) is REQUIRED again after flashing this change, for the
    // same reason as 2026-07-17.
    // 2026-07-17 κ補正: ミキサーのトルク/推力比を実測 κ=6.12e-3 m へ修正（旧 9.71e-3、
    // actuator.cpp KAPPA）。旧 κ ではミキサーは指令ヨートルクの 0.6303 倍しか物理トルクを
    // 出せておらず、飛行実績の「物理」ヨーループゲインは 1.901691e-3 × 0.6303 だった。
    // 修正後も同一ループゲインを保つため kp を κ_new/κ_old 倍へ再スケール。Ti/Td は
    // 時定数なので不変、ロール/ピッチはアーム長 d（不変）基準なので対象外。実機は書き込み後
    // `param reset`（または rate.yaw.* の明示再設定）必須 — NVS の旧 kp のままだと実績の
    // 1.59 倍のループゲインで飛ぶことになる。
    //
    // 2026-08-03 κ再スケール（純粋な再スケールであり新たな物理的知見ではない）:
    // Ct の2026-07-15 thrust stand値は撤回（新プロペラでの電圧/回転数/推力の有効な
    // 同時計測が存在しないため）、暫定 Ct により κ=Cq/Ct は 4.10e-3 に戻る
    // （actuator.cpp KAPPA、詳細な出所は同ファイルのコメント参照）。2026-07-17 と
    // 同じ「物理ゲイン保存」パターンに従い、kp を厳密な比 scale = 4.10/6.12 =
    // 0.6699346 で再スケール:
    //   1.198594e-3 × 0.6699346 = 8.029796e-4  [Nm/(rad/s)]
    // これにより閉ループの duty 出力は 2026-08-03 以前と完全に恒等——ミキサーの
    // KAPPA と本 kp が連動して動くため、指令レート誤差あたりの物理ヨートルクは
    // 不変。2026-07-17 と同じ理由で、書き込み後の `param reset`（または rate.yaw.*
    // の明示再設定）が再度必須。
    float rate_yaw_kp     = 8.029796e-4f;  // = 1.198594e-3 (2026-07-17 κ-corrected) × 4.10/6.12 (2026-08-03 κ rescale)
    float rate_yaw_ti     = 0.8f;
    float rate_yaw_td     = 0.01f;

    // Yaw torque cap [Nm] for the rate-PID output clamp / anti-windup (loaded by
    // pid_controller loadParams). Runtime-tunable after the NT-Kanazawa yaw-
    // saturation diagnosis (2026-06-27 logs): a constant CW/CCW trim asymmetry
    // plus a few-second aerodynamic disturbance saturated the old cap and the
    // craft was spun ~180° in yaw. 1.83e-3 was the treatment value = the
    // old-unit relaxed cap 2.9e-3 × κ_new/κ_old (closed-loop replay of measured
    // disturbances; see analysis/scripts/yaw_nt_kanazawa/) under the
    // 2026-07-17..2026-08-02 κ=6.12e-3. The flight-proven-equivalent cap under
    // that same κ was 1.387e-3 (= old 2.2e-3 × 0.6303) if a fallback is needed.
    //
    // 2026-08-03 κ rescale (PURE rescale, same reason as rate_yaw_kp above —
    // see actuator.cpp KAPPA comment for the Ct-retraction provenance): default
    // rescaled by 4.10/6.12 = 0.6699346, table max rescaled the same way:
    //   1.83e-3 × 0.6699346 = 1.226e-3   (default)
    //   2.1e-3  × 0.6699346 = 1.41e-3    (table max; was ≈ geometric full-scale
    //                                     2·0.168 N·κ_old = 2.06e-3, now
    //                                     2·0.168 N·κ_new ≈ 1.38e-3)
    // This keeps the physical torque cap IDENTICAL to before 2026-08-03 — the
    // mixer's KAPPA moved together with this cap, so no flight behavior change.
    // ヨートルク上限 [Nm]（レートPID出力クランプ/アンチワインドアップ、pid_controller が
    // 読む）。NT金沢のヨー飽和診断（2026-06-27 ログ）を受けてランタイム調整可能化:
    // CW/CCW トリム非対称＋数秒持続の空力外乱で旧上限が飽和しヨーが約180°回された。
    // 1.83e-3 は治療値＝旧単位の緩和上限 2.9e-3 × κ_new/κ_old（実測外乱の閉ループ
    // 再生シム: analysis/scripts/yaw_nt_kanazawa/）、2026-07-17〜2026-08-02の
    // κ=6.12e-3 下での値。同κ下での飛行実績等価は 1.387e-3（旧 2.2e-3 × 0.6303）。
    //
    // 2026-08-03 κ再スケール（純粋な再スケール、理由は上の rate_yaw_kp と同じ——
    // Ct撤回の経緯は actuator.cpp KAPPA コメント参照）: 既定値・テーブル最大値とも
    // 4.10/6.12 = 0.6699346 で再スケール:
    //   1.83e-3 × 0.6699346 = 1.226e-3   （既定値）
    //   2.1e-3  × 0.6699346 = 1.41e-3    （テーブル最大値。旧κでの幾何フルスケール
    //                                      2·0.168N·κ_old≈2.06e-3 相当だったが、
    //                                      新κでは 2·0.168N·κ_new≈1.38e-3）
    // 物理トルク上限は2026-08-03以前と完全に恒等——ミキサーのKAPPAと本上限が連動して
    // 動くため、飛行挙動の変化はない。
    float rate_yaw_max_torque = 1.226e-3f;

    // Sliding-mode rate-loop gains (firmware/apps/smc_rate, an `sf app`
    // experiment -- docs/plans/smc-rate-loop-plan.md). The DEFAULT vehicle
    // build (no SF_APP_DIR, i.e. PidController) never reads these; they
    // exist only so smc_rate's AppController gets the same sf params/NVS/
    // `sf sils scenario --param` tunability the PID gains above have,
    // without adding a second parameter subsystem.
    //
    // Seed derivation (docs/plans/smc-rate-loop-plan.md §2.4): the reaching
    // law is torque = I_axis*(k*sat(s/phi) + eta*s). Inside the boundary
    // layer (sat(s/phi)=s/phi) this is torque = I_axis*(k/phi + eta)*s, a
    // linear gain on rate error like the PID's kp. Let
    // Omega_axis = rate.<axis>.kp / I_axis (the existing PID's equivalent
    // 1/s bandwidth for that axis) and split it eta=0.7*Omega (most of the
    // proven small-signal behavior) + k/phi=0.3*Omega (extra reaching-phase
    // margin outside the boundary layer), with a shared phi=0.3 rad/s:
    //   roll:  Omega = 1.0e-3/9.16e-6  = 109.17 /s -> eta=76.42, k=0.3*Omega*phi=9.83
    //   pitch: Omega = 1.426432e-3/13.3e-6 = 107.25 /s -> eta=75.07, k=9.65
    //   yaw:   Omega = 8.029796e-4/20.4e-6 = 39.36 /s -> eta=27.55, k=3.54
    //
    // SILS-tuned update (2026-09-11, docs/plans/smc-rate-loop-plan.md §3.1):
    // the derived seed above passed `acro_flight` (rate doublets) but FAILED
    // `stab_flight` (STABILIZE combined roll+pitch step, att_rmse 4.47 deg >
    // the 3.0 deg gate, vs. PID's 2.77 deg). A `sf sils scenario --param`
    // sweep found halving phi (wider reaching-law authority relative to the
    // boundary layer) and roughly doubling k/eta cleared stab_flight
    // (att_rmse 2.85 deg) with NO regression on acro_flight (2.10 deg) --
    // yaw scaled by the same ratio and additionally checked on yaw_hold (an
    // M1-motor-80%-failure heading-hold stress scenario): yaw_band 0.23 deg
    // vs. PID's 8.01 deg, duty_max 0.82 vs. PID's saturated 1.00 (yaw_hold's
    // OWN alt_max failure is a pre-existing xfail-marked SILS yaw-plant-
    // fidelity gap, simulation-policy.md backlog #11/#12 -- NOT attributable
    // to this change; both PID and SMC fail it identically).
    //
    // These values are still SILS-only (noise=off, single seed) --
    // NOT flight-validated, and not yet swept against the perturbation
    // family simulation-policy.md §6 requires before a real-hardware
    // proposal (motor-delay/thrust-eff/torque-authority, --noise n1/n2).
    // Per CLAUDE.md, no control-parameter change is proposed for real
    // hardware without that numerical backing first.
    //
    // スライディングモード・レートループのゲイン（firmware/apps/smc_rate、
    // `sf app`実験 -- docs/plans/smc-rate-loop-plan.md）。既定vehicleビルド
    // （SF_APP_DIR無し、つまりPidController）はこれらを一切読まない。
    // smc_rateのAppControllerが上のPIDゲインと同じsf params/NVS/
    // `sf sils scenario --param`調整能力を、第2のパラメータ機構を作らずに
    // 得るためだけに存在する。
    //
    // 初期値の導出（docs/plans/smc-rate-loop-plan.md §2.4）: 到達則は
    // torque = I_axis*(k*sat(s/phi) + eta*s)。境界層内（sat(s/phi)=s/phi）
    // ではtorque = I_axis*(k/phi + eta)*sとなり、PIDのkpと同様レート誤差に
    // 対する線形ゲインになる。Omega_axis = rate.<axis>.kp / I_axis
    // （そのPIDの等価帯域[1/s]）を、eta=0.7*Omega（実績の線形域挙動の大半）
    // + k/phi=0.3*Omega（境界層外の到達フェーズ余裕）に分配し、
    // phi=0.3 rad/s（全軸共通）とする。
    //
    // SILSチューニングによる更新（2026-09-11、docs/plans/smc-rate-loop-plan.md
    // §3.1）: 上の逆算初期値は`acro_flight`（レートダブレット）はPASSしたが
    // `stab_flight`（STABILIZE roll+pitch複合ステップ）でatt_rmse=4.47°
    // （ゲート3.0°、PIDは2.77°）とFAILした。`sf sils scenario --param`
    // スイープでphiを半減（境界層に対する到達則の相対的な権限を拡大）し
    // k/etaを概ね2倍にしたところ、stab_flightがPASS（att_rmse=2.85°）、
    // acro_flightも劣化なし（2.10°）。yawも同じ倍率でスケールし、
    // yaw_hold（M1モータ80%故障下のヘディングホールド外乱耐性シナリオ）で
    // 追加確認: yaw_band 0.23°（PIDは8.01°）、duty_max 0.82（PIDは飽和1.00）
    // — yaw_hold自体のalt_max FAILはxfailマーク付きの既存SILS yaw軸プラント
    // 忠実度ギャップ（simulation-policy.md backlog #11/#12）であり本変更とは
    // 無関係（PID・SMC両方が同様にFAILする）。
    //
    // これらの値はまだSILSのみ（noise=off、単一シード）での確認であり、
    // 飛行検証済みではなく、実機提案前にsimulation-policy.md§6が求める
    // 摂動族（motor-delay/thrust-eff/torque-authority, --noise n1/n2）での
    // スイープも未実施。CLAUDE.mdの規定どおり、その数値的裏付け無しに
    // 実機向けの制御パラメータ変更は提案しない。
    // Rounds 2-3 (2026-09-11, docs/plans/smc-rate-loop-plan.md SS3.3) tried
    // to fix a round-1 weakness (att_rmse up to 5.18 deg vs. PID's 2.5-2.8
    // deg under simulation-policy.md SS6's --torque-authority 0.4/0.55
    // perturbation) by pushing eta much higher (up to 400/390) and phi much
    // narrower (down to 0.05) -- closer to a classical near-bang-bang
    // reaching law. This DID fix torque-authority (0.4/0.55/0.7 all passed,
    // att_rmse 2.2-2.9 deg) and even improved the nominal case, but caused a
    // CATASTROPHIC regression elsewhere: --motor-delay 15ms produced a full
    // tumble (att_rmse 187 deg, tilt_max 180-201 deg -- not a missed gate,
    // an actual loss of control), and --noise n1 (a MILDER level than n2)
    // failed to even complete takeoff. A middle-ground point (eta 250/245,
    // phi 0.1) still tumbled at motor-delay 15ms (tilt_max 201 deg) while
    // STILL failing torque-authority=0.4 and noise n1/n2 -- proving eta
    // itself, once past some threshold well below 250, already crosses into
    // an unsafe operating region against the actuator's unmodeled ~15ms
    // delay, regardless of phi/k. This is the mirror image of round 1's
    // problem: eta high enough for torque-authority robustness is too high
    // for delay phase margin, and no phi/k adjustment escaped that trade-off
    // in the points tried. PID's integral term does not face this trade-off
    // (it compensates sustained multiplicative torque loss by accumulating,
    // not by raising loop gain), which is the structural advantage this
    // integral-free single-surface reaching law lacks (see smc_rate.hpp's
    // file header).
    //
    // DECISION: safety over gate count. A tumble (round 2/3) is categorically
    // worse than a missed-gate-but-controlled-flight (round 1's 22.8 deg
    // tilt_max at motor-delay=15ms, still short of a crash), even though
    // round 1 passes fewer perturbation checks overall. Reverted to the
    // round-1 values below; torque-authority=0.4/0.55 and noise=n2 on
    // stab_flight remain KNOWN, DOCUMENTED FAILURES -- not silently
    // accepted, just not fixable by gain tuning alone within this design.
    // See docs/plans/smc-rate-loop-plan.md SS3.3 for the full search log
    // (8 distinct gain points tested) and this reasoning. Real-hardware
    // flight is NOT proposed until this is resolved (via a design change --
    // e.g. an integral-like or delay-compensating term -- which is beyond
    // parameter tuning and out of THIS plan's approved scope).
    //
    // ラウンド2〜3（2026-09-11、docs/plans/smc-rate-loop-plan.md §3.3）は
    // ラウンド1の弱点（simulation-policy.md §6の--torque-authority 0.4/0.55
    // 摂動下でatt_rmse最大5.18°、PIDは2.5-2.8°）を、etaを大幅増（400/390まで）・
    // phiを大幅縮小（0.05まで、古典的なほぼバンバン到達則に近づける）で解決
    // しようとした。torque-authority自体は解決（0.4/0.55/0.7全てPASS、
    // att_rmse 2.2-2.9°）し名目ケースも改善したが、**壊滅的な退行**を招いた:
    // --motor-delay 15msで完全な転倒（att_rmse 187°、tilt_max 180-201°——
    // ゲート未達でなく実際の制御喪失）、--noise n1（n2より軽いレベル）では
    // 離陸すら完了しなかった。中間点（eta 250/245, phi 0.1）でもmotor-delay
    // 15msで転倒（tilt_max 201°）したままtorque-authority=0.4・noise n1/n2は
    // 未解決——250程度を超えた時点で既にetaそのものが、phi/kの調整に関わらず
    // アクチュエータの未モデル化な約15ms遅れに対して安全域を外れることの証明。
    // これはラウンド1の問題の鏡像である: torque-authority頑健性に必要な高eta
    // は、遅れに対する位相余裕としては高すぎ、試した範囲ではphi/kの調整で
    // この二律背反を回避できなかった。PIDの積分項はこのトレードオフに直面
    // しない（ループゲインを上げるのでなく積分蓄積で持続的な乗法的トルク損失を
    // 補償する）——これがこの積分無し単一面到達則に欠けている構造的な優位点。
    //
    // 判断: ゲート通過数より安全性を優先する。転倒（ラウンド2/3）は
    // 「ゲート未達だが制御された飛行」（ラウンド1のmotor-delay=15msでの
    // tilt_max=22.8°、墜落には至らない）より明確に悪い——ラウンド1の方が
    // 通過する摂動チェックの総数は少なくても。下記の値をラウンド1に戻した。
    // stab_flightでのtorque-authority=0.4/0.55・noise=n2は**既知・記録済みの
    // 未解決FAIL**のまま——黙って受け入れたのでなく、本設計の範囲内では
    // ゲイン調整だけでは解決できないと判断した。探索の全記録（8通りの異なる
    // ゲイン点を検証）と根拠はdocs/plans/smc-rate-loop-plan.md §3.3参照。
    // これが解消するまで実機飛行は提案しない（積分様項・遅れ補償項等の設計
    // 変更が必要になる可能性が高いが、それはパラメータチューニングの範囲を
    // 超え、本計画で承認された範囲外）。
    // Round 4 (2026-09-11, docs/plans/smc-rate-loop-plan.md SS3.7): joint
    // eta/lambda_i re-sweep, now that lambda_i can carry some of the
    // torque-authority robustness burden. eta raised modestly (150/145 ->
    // 180/175, well short of round 2/3's catastrophic 400/390) together
    // with lambda_i raised substantially (1.5 -> 6) -- NOT eta alone, which
    // rounds 2/3 already showed traded torque-authority for a
    // motor-delay=15ms tumble. This combination is the best point found in
    // a bumpy, non-monotonic search (6 combinations tried): stab_flight
    // torque-authority=0.4 PASSES for the first time (2.78 deg, was FAIL at
    // every eta/lambda_i tried before), nominal improved (2.57 deg), and
    // motor-delay=15ms did NOT regress (24.3 deg tilt_max, same 22-25 deg
    // band as every other point tried since the PI-surface was added --
    // still no repeat of rounds 2/3's 180-201 deg tumbles). torque-
    // authority=0.55 and noise=n1/n2 remain FAILING (see SS3.7's table) --
    // the search space is genuinely non-monotonic (a nearby lambda_i=5
    // point FAILS nominal outright while passing ta=0.55) and no single
    // point tried passes all of stab_flight's perturbation set.
    // ラウンド4（2026-09-11、docs/plans/smc-rate-loop-plan.md §3.7）: eta/
    // lambda_iの同時再探索。lambda_iがtorque-authority頑健性の一部を
    // 担えるようになったため。etaは控えめに引き上げ（150/145→180/175、
    // ラウンド2〜3の壊滅的な400/390とは程遠い）、lambda_iを大幅に引き上げ
    // （1.5→6）——eta単独ではない。ラウンド2〜3が既に示したとおりeta単独は
    // torque-authorityとmotor-delay=15msの転倒を交換するだけ。凹凸のある
    // 非単調な探索（6通り試行）の中で見つかった最良点: stab_flightの
    // torque-authority=0.4が初めてPASS（2.78°、これまで試した全eta/lambda_i
    // でFAILだった）、名目も改善（2.57°）、motor-delay=15msは悪化しなかった
    // （tilt_max 24.3°、PI面追加以降どの点でも同じ22〜25°帯——ラウンド2〜3の
    // 180〜201°の転倒は一度も再発していない）。torque-authority=0.55と
    // noise n1/n2は未解決のまま（§3.7の表参照）——探索空間は本当に非単調
    // （近傍のlambda_i=5点はta=0.55はPASSするが名目自体がFAILする）で、
    // stab_flightの摂動群全てを通す単一点は見つかっていない。
    // Round 5 (2026-09-11, docs/plans/smc-rate-loop-plan.md SS3.8): k swept
    // at round-4's eta/lambda_i/phi (180/175, 6, 0.15) over {20,30,40,45}.
    // NON-MONOTONIC again -- k=20 passes ta=0.4(2.78) but fails ta=0.55
    // badly (3.69, +23%); k=30 fails BOTH worse than either neighbor
    // (4.07/3.08); k=40 passes ta=0.55 comfortably (2.57) and ta=0.4 only
    // barely fails (3.06, +2%); k=45 makes both worse again (3.32/3.16).
    // k=40 kept as the better-balanced point (smaller total gate overshoot)
    // -- still does NOT clear the full stab_flight perturbation set (see
    // SS3.8 for the complete table). No k found that passes both
    // simultaneously; the landscape is genuinely non-convex, not merely
    // under-explored (9 total (eta,lambda_i,phi,k) points tried across
    // SS3.7-3.8).
    // ラウンド5（2026-09-11、docs/plans/smc-rate-loop-plan.md §3.8）: ラウンド4の
    // eta/lambda_i/phi（180/175, 6, 0.15）を固定し、kを{20,30,40,45}で
    // スイープ。再び非単調 -- k=20はta=0.4をPASS(2.78)するがta=0.55は大幅に
    // FAIL(3.69, +23%)、k=30は両方とも隣接点より悪化(4.07/3.08)、k=40は
    // ta=0.55を余裕でPASS(2.57)しta=0.4はわずかにFAIL(3.06, +2%)、k=45は
    // 両方再び悪化(3.32/3.16)。総ゲート超過が小さいk=40をより均衡の取れた点
    // として採用——それでもstab_flightの摂動群全体はクリアしない（完全な表は
    // §3.8参照）。両方同時にPASSするkは見つからなかった——探索不足でなく
    // 探索空間が本質的に非凸である（§3.7〜3.8で計9通りの(eta,lambda_i,phi,k)
    // 点を試行）。
    float smc_roll_k     = 40.0f;    // [rad/s^2] round-5 (was 20.0)
    float smc_roll_eta   = 180.0f;   // [1/s] round-4 (was 150.0, round 1)
    float smc_roll_phi   = 0.15f;    // [rad/s] unchanged from round 1
    float smc_pitch_k    = 40.0f;    // [rad/s^2] round-5 (was 20.0, see roll_k comment above)
    float smc_pitch_eta  = 175.0f;   // [1/s] round-4 (was 145.0, round 1)
    float smc_pitch_phi  = 0.15f;    // [rad/s] unchanged from round 1
    // yaw.phi kept at the ORIGINAL 0.3 (not halved like roll/pitch): narrowing
    // it to 0.15 alongside the higher k/eta won yaw_hold's yaw_band a little
    // more (0.23 vs 0.31 deg, both well inside the 1.5 deg gate) but broke
    // stab_flight (att_rmse 3.32 deg > the 3.0 deg gate, vs. 2.88 deg at
    // phi=0.3) -- confirmed by reverting only yaw.phi and re-testing both
    // scenarios (docs/plans/smc-rate-loop-plan.md §3.1). stab_flight's RC
    // script holds the yaw stick centered, so this is an indirect coupling
    // (yaw reaction torque during the roll/pitch maneuver, handled more
    // twitchily by a narrower yaw boundary layer) rather than a direct yaw
    // command effect -- exactly the kind of single-scenario overfit
    // simulation-policy.md §6 warns against, so phi stays at the wider,
    // dual-scenario-safe value.
    // yaw.phiはroll/pitchのように半減せず、元の0.3のまま据え置く。より
    // 高いk/etaと合わせて0.15まで狭めるとyaw_holdのyaw_bandはやや改善した
    // （0.23° vs 0.31°、どちらもゲート1.5°に対し十分な余裕）が、stab_flight
    // を壊した（att_rmse 3.32°>ゲート3.0°、phi=0.3では2.88°）——yaw.phiだけを
    // 戻して両シナリオを再テストして確認済み（docs/plans/smc-rate-loop-plan.md
    // §3.1）。stab_flightのRCスクリプトはヨースティック中立を保つため、これは
    // ヨー指令の直接効果ではなく間接結合（roll/pitch機動中のヨー反力トルクを、
    // 狭い境界層のヨーがより神経質に処理する）——まさにsimulation-policy.md§6
    // が警告する単一シナリオへの過学習であり、両シナリオで安全な広い方の値を
    // 採用する。
    float smc_yaw_k      = 7.3f;     // [rad/s^2] SILS-tuned 2026-09-11 (was 3.54)
    float smc_yaw_eta    = 54.0f;    // [1/s] SILS-tuned 2026-09-11 (was 27.55)
    float smc_yaw_phi    = 0.3f;     // [rad/s] unchanged -- see note above

    // PI-type sliding surface integral gain (2026-09-11, docs/plans/
    // smc-rate-loop-plan.md SS3.4/SS6 [R3]): s = e + lambda_i*integral(e dt)
    // instead of the original s = e. Added specifically to fix the
    // stab_flight + --torque-authority 0.4/0.55 failure documented above
    // (SS3.2/SS3.3) WITHOUT repeating rounds 2-3's mistake of raising eta
    // (which traded that fix for a motor-delay=15ms tumble) -- integral
    // action compensates a STEADY multiplicative torque-effectiveness loss
    // by accumulating, the same mechanism PID's Ti already relies on, so it
    // should not need to raise the instantaneous loop gain the way eta
    // does. lambda_i=0 recovers the original pure-P surface exactly.
    // Seeded from the existing PID's integral time constants (1/Ti) so the
    // ADDED integral action's own bandwidth is comparable, not a guess:
    //   roll/pitch: 1/rate.{roll,pitch}.ti = 1/0.7 = 1.43 /s -> 1.5
    //   yaw:        1/rate.yaw.ti           = 1/0.8 = 1.25 /s
    // SILS-tuning status (2026-09-11, docs/plans/smc-rate-loop-plan.md SS3.5):
    // VALIDATED as a genuine PARTIAL improvement, NOT a full fix. Swept
    // lambda_i in {1.5, 3, 5} on stab_flight against nominal + the SS3.2/3.3
    // perturbation set:
    //   - torque-authority=0.4/0.55 att_rmse improved from ~4.9-5.2 deg
    //     (lambda_i=0, round-1 eta/k/phi) to ~3.0-3.8 deg -- a real ~30%
    //     reduction, but still short of the 3.0 deg gate at every lambda_i
    //     tried.
    //   - motor-delay=15ms did NOT regress at any lambda_i tested (tilt_max
    //     stayed in the 22-25 deg band across all three values) -- the
    //     integral term does NOT reproduce rounds 2-3's catastrophic
    //     eta-driven phase-margin loss (tilt_max 180-201 deg tumbles). This
    //     is the main achievement: torque-authority robustness gained
    //     WITHOUT paying for it in delay margin.
    //   - The 3 values tried are NOT monotonic across conditions (lambda_i=
    //     1.5 gives the best torque-authority=0.4 and a passing nominal;
    //     lambda_i=3 gives a better torque-authority=0.55 but a marginal
    //     nominal and worse noise-n1; lambda_i=5 fails nominal outright).
    //     noise n1/n2 did not improve at any lambda_i tried.
    // lambda_i=1.5 (below) is kept as the best point found (passing nominal,
    // the largest torque-authority=0.4 improvement) -- torque-authority=
    // 0.4/0.55 and noise=n1/n2 on stab_flight remain KNOWN, DOCUMENTED
    // FAILURES, same real-hardware gate as SS3.3. A joint lambda_i/eta
    // re-sweep (not yet done) is the natural next step -- see SS3.5.
    // PI型スライディング面の積分ゲイン（2026-09-11、docs/plans/
    // smc-rate-loop-plan.md §3.4/§6 [R3]）: 元の s = e の代わりに
    // s = e + lambda_i*積分(e dt) とする。上記（§3.2/§3.3）で記録した
    // stab_flight + --torque-authority 0.4/0.55 の失敗を、ラウンド2〜3の
    // 過ち（etaを上げてmotor-delay=15msの転倒と引き換えにした）を繰り返さず
    // 修正するために追加した -- 積分作用は持続的な乗法的トルク有効度損失を
    // 「積分の蓄積」で補償する。これはPIDのTiが既に依拠する機構と同じで、
    // etaのようにループの瞬時ゲインを上げる必要が無いはず。lambda_i=0で
    // 元の純P面に厳密に一致する。既存PIDの積分時定数（1/Ti）から初期値を
    // 導出（当て推量でなく）:
    //   roll/pitch: 1/rate.{roll,pitch}.ti = 1/0.7 = 1.43/s -> 1.5
    //   yaw:        1/rate.yaw.ti           = 1/0.8 = 1.25/s
    // SILSチューニング状況（2026-09-11、docs/plans/smc-rate-loop-plan.md
    // §3.5）: 「部分的な改善」として検証済み——完全な解決ではない。
    // lambda_iを{1.5, 3, 5}でstab_flightに対しnominal+§3.2/3.3の摂動群で
    // スイープした結果: torque-authority=0.4/0.55のatt_rmseは約4.9〜5.2°
    // （lambda_i=0、ラウンド1のeta/k/phi）から約3.0〜3.8°へ改善（約30%減）
    // したが、どのlambda_iでも3.0°ゲートには届かなかった。motor-delay=15ms
    // はどのlambda_iでも悪化しなかった（tilt_maxは全て22〜25°の帯域に留まり、
    // ラウンド2〜3のような壊滅的な位相余裕喪失（tilt_max 180〜201°の転倒）を
    // 再現しなかった）——これが主な成果: torque-authority頑健性を遅れ耐性を
    // 犠牲にせず獲得できた。試した3値は条件間で非単調（lambda_i=1.5は
    // torque-authority=0.4が最良かつnominalがPASS、lambda_i=3は
    // torque-authority=0.55はより良いがnominalはギリギリでnoise n1は悪化、
    // lambda_i=5はnominal自体がFAIL）。noise n1/n2はどのlambda_iでも
    // 改善しなかった。下記のlambda_i=1.5を最良点として採用（nominalが
    // PASSしtorque-authority=0.4の改善幅が最大）——torque-authority=
    // 0.4/0.55・noise=n1/n2はstab_flightで既知・記録済みの未解決FAILの
    // まま、§3.3と同じ実機投入ゲート未達。lambda_i/etaの同時再探索
    // （未実施）が次の自然な一手——§3.5参照。
    float smc_roll_lambda_i  = 6.0f;   // [1/s] round-4 (was 1.5, SS3.7)
    float smc_pitch_lambda_i = 6.0f;   // [1/s] round-4 (was 1.5, SS3.7)
    float smc_yaw_lambda_i   = 1.25f;  // [1/s]

    // PI-surface integral anti-windup, layers 2-3 (2026-09-11, docs/plans/
    // smc-rate-loop-plan.md SS3.6, added after review of the PI-surface
    // integral's reset/bounding -- layer 1, conditional integration, needed
    // no new param, it is algorithmic). e_reset: |e| threshold above which
    // AppController's SlidingModeRate snaps its integral to 0 outright
    // (layer 3) rather than merely freezing it (layer 1) -- a large
    // transient (aggressive maneuver, big disturbance) makes the
    // accumulated integral stale, and 0 disables this layer. Seeded at
    // 5*phi (SS3.6's convention: "how far outside the linear operating
    // regime counts as a large transient" is phi-relative) using each
    // axis's phi default from ABOVE (roll/pitch 0.15, yaw 0.3):
    //   roll/pitch: 5 * 0.15 = 0.75 rad/s
    //   yaw:        5 * 0.3  = 1.5  rad/s
    // (Layer 2's backstop clamp bound is computed at runtime from
    // output_limit/inertia/eta/lambda_i in smc_rate.hpp -- no separate
    // param needed.) NOT YET SILS-validated -- seed values only.
    // PI面積分のアンチワインドアップ、第2〜3層（2026-09-11、docs/plans/
    // smc-rate-loop-plan.md §3.6、PI面積分のリセット/上限に関するレビューを
    // 受けて追加 -- 第1層の条件付き積分は新規パラメータ不要、アルゴリズム
    // 内蔵のため）。e_reset: これを超える|e|でAppControllerの
    // SlidingModeRateが積分を単に凍結（第1層）でなく0へスナップ（第3層）
    // する閾値 -- 大きな過渡（激しい機動・大外乱）は蓄積積分を古くする、
    // 0でこの層を無効化。5*phi（§3.6の慣例: 「線形動作域からどれだけ外れれば
    // 大きな過渡か」は本質的にphi相対）で初期化、各軸のphi既定値（上記、
    // roll/pitch 0.15、yaw 0.3）から:
    //   roll/pitch: 5 * 0.15 = 0.75 rad/s
    //   yaw:        5 * 0.3  = 1.5  rad/s
    // （第2層のバックストップクランプの境界はsmc_rate.hppで
    // output_limit/inertia/eta/lambda_iから実行時に計算する -- 別パラメータ
    // 不要。）SILS未検証 -- あくまで初期値。
    float smc_roll_e_reset  = 0.75f;  // [rad/s]
    float smc_pitch_e_reset = 0.75f;  // [rad/s]
    float smc_yaw_e_reset   = 1.5f;   // [rad/s]

    // Dead-time predictor (2026-09-11, docs/plans/smc-rate-loop-plan.md's
    // motor-delay root-cause work -- see smc_rate.hpp's SlidingModeRate
    // field comment for the derivation). [ms] for readability (SILS
    // --motor-delay and the real per-axis system-ID measurements are both
    // quoted in ms); AppController converts to seconds when loading.
    // Default 0 = disabled, EXACT existing behavior for smc_rate/smc_pos.
    // Range ceiling 40ms matches SlidingModeRate::kDelayCompMaxSamples
    // (16 samples @ 2.5ms).
    // 無駄時間予測補償器（2026-09-11、docs/plans/smc-rate-loop-plan.mdの
    // motor-delay根本対処 -- 導出はsmc_rate.hppのSlidingModeRateフィールド
    // コメント参照）。読みやすさのため[ms]（SILSの--motor-delayも実機系
    // 同定値もmsで表記される）。AppControllerが読込時に秒へ変換する。
    // 既定0=無効、smc_rate/smc_posの既存挙動そのまま。上限40msは
    // SlidingModeRate::kDelayCompMaxSamples（400Hzで16サンプル=40ms）と一致。
    float smc_roll_delay_comp_ms  = 0.0f;  // [ms]
    float smc_pitch_delay_comp_ms = 0.0f;  // [ms]
    float smc_yaw_delay_comp_ms   = 0.0f;  // [ms]

    // Sliding-surface dead-band (2026-09-11, docs/plans/smc-rate-loop-plan.md
    // §7.10 -- see smc_rate.hpp's SlidingModeRate s_deadband field comment
    // for the derivation). Delay-agnostic alternative to the predictor
    // above: does not need to know the actual dead time L, so a wrong value
    // only under/over-shrinks the dead zone rather than mismatching in a
    // dangerous direction. Default 0 = disabled, unchanged behavior.
    // スライディング面の不感バンド（2026-09-11、docs/plans/
    // smc-rate-loop-plan.md §7.10 -- 導出はsmc_rate.hppのSlidingModeRateの
    // s_deadbandフィールドコメント参照）。上の予測補償器と異なり実際の遅れL
    // を知る必要がない -- 値を間違えても不感帯の過小/過大にしかならず、
    // 危険な方向へミスマッチしない。既定0=無効、挙動は不変。
    float smc_roll_s_deadband  = 0.0f;  // [rad/s]
    float smc_pitch_s_deadband = 0.0f;  // [rad/s]
    float smc_yaw_s_deadband   = 0.0f;  // [rad/s]

    // Super-twisting rate-loop gains (firmware/apps/smc_rate_sta,
    // smc_rate_sta.hpp) -- tried 2026-09-11 against the still-unresolved
    // motor-delay=15ms tilt_max FAIL (docs/plans/smc-rate-loop-plan.md
    // §7.11). Unused by vehicle/smc_rate/smc_pos.
    //
    // Seed derivation (NOT YET SILS-validated -- seed values only):
    // matched to smc_rate's own round-5 first-order design (k/eta/phi) so
    // u1=k1*sqrt(|s|) reaches roughly the same torque at s=phi_old as the
    // old k*sat(1)+eta*phi_old did: k1 = (k+eta*phi_old)/sqrt(phi_old).
    //   roll/pitch: (40+180*0.15)/sqrt(0.15) = 66.25/0.387 ~ 171 -> 170
    //   yaw:        (7.3+54*0.3)/sqrt(0.3) = 23.5/0.548 ~ 43
    // k2 seeded at k1/2 (a common starting ratio; Levant/Moreno-Osorio's
    // own sufficient conditions need a disturbance-derivative bound we do
    // not have -- SILS will show whether this needs to shift).
    // phi here is NOT smc_rate's boundary layer (STA's u1 already vanishes
    // continuously at s=0) -- it is a small numerical-only smoothing width
    // for the discrete-time sign() evaluations, seeded an order of
    // magnitude below the old phi (0.02/0.02/0.04 vs 0.15/0.15/0.3).
    // lambda_i/e_reset carried over unchanged from smc_rate's round-5
    // values -- same PI-surface role (steady-bias rejection / large-
    // transient integral reset), independent of STA's own phi.
    // スーパーツイスティング・レートループのゲイン（firmware/apps/
    // smc_rate_sta, smc_rate_sta.hpp）-- 2026-09-11、未解決のmotor-delay=
    // 15msでのtilt_max FAILに対して試行（docs/plans/smc-rate-loop-plan.md
    // §7.11）。vehicle/smc_rate/smc_posでは未使用。
    //
    // 初期値の導出（SILS未検証 -- あくまで初期値）: smc_rateのラウンド5
    // 一次設計（k/eta/phi）に合わせた——u1=k1*sqrt(|s|)がs=phi_old時に
    // 旧k*sat(1)+eta*phi_oldとほぼ同じトルクに達するよう
    // k1=(k+eta*phi_old)/sqrt(phi_old)で算出:
    //   roll/pitch: (40+180*0.15)/sqrt(0.15) = 66.25/0.387 ~ 171 -> 170
    //   yaw:        (7.3+54*0.3)/sqrt(0.3) = 23.5/0.548 ~ 43
    // k2はk1/2で初期化（よくある出発比。Levant/Moreno-Osorio自身の十分条件
    // は外乱導関数の有界値を要求するが未知——SILSでこの比を動かす必要が
    // あるか分かる）。ここでのphiはsmc_rateの境界層ではない（STAのu1は
    // s=0で既に連続的に消える）——離散時間sign()評価のための純粋に数値的な
    // 平滑化幅で、旧phiより一桁小さく初期化（0.02/0.02/0.04 vs
    // 0.15/0.15/0.3）。lambda_i/e_resetはsmc_rateのラウンド5の値をそのまま
    // 流用——PI面の役割（定常バイアス除去/大過渡での積分リセット）は
    // STA自身のphiとは独立。
    // Round-1 tuning (2026-09-11, docs/plans/smc-rate-loop-plan.md §7.11-7.12):
    // seed k1=170 fixed motor-delay=15ms's tilt_max (21.58deg, still short of
    // the <18deg gate) but a sweep found k1=120/k2=60 clears it cleanly
    // (14.90deg) at the cost of att_rmse regressing across nominal/torque-
    // authority/noise (2.5-3deg baseline -> 3-5.7deg). k1=140/k2=70 is the
    // balance point: BOTH nominal (att_rmse=2.74, first FULL PASS since
    // section 3.3) AND motor-delay=15ms (att_rmse=0.99, tilt_max=17.19,
    // first FULL PASS ever on this condition) clear all 3 gates
    // simultaneously. Remaining FAILs (torque-authority=0.4 att_rmse=3.34
    // mild, noise n1 att_rmse=5.24) match the SAME already-accepted, non-
    // catastrophic limitation class documented for smc_rate's own PID/
    // first-order-SMC baselines -- not a new regression. Yaw scaled by the
    // same 140/170 ratio (untested directly, no yaw-specific sweep done).
    // ラウンド1チューニング（2026-09-11、docs/plans/smc-rate-loop-plan.md
    // §7.11-7.12）: シードk1=170はmotor-delay=15msのtilt_maxを改善したが
    // （21.58°、<18°ゲート未達）、スイープの結果k1=120/k2=60が明確にクリア
    // （14.90°）——ただしnominal/torque-authority/noiseでatt_rmseが悪化
    // （2.5-3°基準→3-5.7°）する代償あり。k1=140/k2=70がバランス点:
    // nominal（att_rmse=2.74、§3.3以降初のフルPASS）とmotor-delay=15ms
    // （att_rmse=0.99, tilt_max=17.19、この条件で史上初のフルPASS）の
    // どちらも3ゲート同時にクリア。残るFAIL（torque-authority=0.4の
    // att_rmse=3.34軽微、noise n1の5.24）はsmc_rate自身のPID/1次SMC基準で
    // 既に受容済みの同種の限界であり、新規退行ではない。yawは140/170と
    // 同じ比でスケール（yaw固有のスイープは未実施、未検証）。
    // Round-2 tuning (2026-09-11, docs/plans/smc-rate-loop-plan.md §7.13-7.14):
    // round-1's k1=140/k2=70 was validated ONLY against stab_flight, and
    // collapsed catastrophically on pos_flight.scn (simultaneous roll+pitch,
    // POS_HOLD) under the identical motor-delay=15ms perturbation --
    // drift=57.2m, tilt_max=35.9deg (tumble-class). A joint sweep re-run
    // against BOTH stab_flight AND pos_flight (plus the new
    // stab_combined_aggressive.scn) simultaneously found k1=60/k2=30 as the
    // only value clearing all three archetypes without a tumble-class
    // failure -- k1=50 and k1=80 both regressed nominal att_rmse (4.21 and
    // 4.16 vs k60's 2.96), confirming a non-monotonic, narrow local optimum
    // near k1=60, not a simple "lower is safer" relationship. See
    // docs/plans/smc-rate-loop-plan.md §7.14 for the full joint-sweep table.
    // Yaw scaled by the same 60/170 ratio relative to the original seed
    // (still untested directly for yaw).
    // ラウンド2チューニング（2026-09-11、docs/plans/smc-rate-loop-plan.md
    // §7.13-7.14）: ラウンド1のk1=140/k2=70はstab_flightだけで検証されており、
    // 同じmotor-delay=15ms摂動下でpos_flight.scn（ロール+ピッチ同時、
    // POS_HOLD）では壊滅的に破綻した——drift=57.2m, tilt_max=35.9°（転倒級）。
    // stab_flightとpos_flight（さらに新規stab_combined_aggressive.scn）の
    // 両方を同時にゲートとする結合スイープをやり直し、k1=60/k2=30だけが
    // 3つのアーキタイプ全てで転倒級の失敗なしにクリアすることを確認した——
    // k1=50・k1=80はどちらもnominalのatt_rmseが悪化（4.21・4.16、k60の
    // 2.96に対して）、「低いほど安全」という単純な関係ではなく、k1=60付近の
    // 狭い局所最適であることを確認。yawは元シードに対する同じ60/170比で
    // スケール（yaw固有の検証は依然未実施）。
    float smc_sta_roll_k1        = 60.0f;   // [rad/s^2 per sqrt(rad/s)] round-2 (was 140 round-1, 170 seed)
    float smc_sta_roll_k2        = 30.0f;   // [rad/s^3] round-2 (was 70 round-1, 85 seed)
    float smc_sta_roll_phi       = 0.02f;   // [rad/s]
    float smc_sta_roll_lambda_i  = 6.0f;    // [1/s]
    float smc_sta_roll_e_reset   = 0.75f;   // [rad/s]
    float smc_sta_pitch_k1       = 60.0f;   // [rad/s^2 per sqrt(rad/s)] round-2 (was 140 round-1, 170 seed)
    float smc_sta_pitch_k2       = 30.0f;   // [rad/s^3] round-2 (was 70 round-1, 85 seed)
    float smc_sta_pitch_phi      = 0.02f;   // [rad/s]
    float smc_sta_pitch_lambda_i = 6.0f;    // [1/s]
    float smc_sta_pitch_e_reset  = 0.75f;   // [rad/s]
    float smc_sta_yaw_k1         = 15.2f;   // [rad/s^2 per sqrt(rad/s)] round-2, scaled by 60/170 (was 35.4 round-1, 43 seed, untested for yaw)
    float smc_sta_yaw_k2         = 7.4f;    // [rad/s^3] round-2, scaled by 60/170 (was 17.3 round-1, 21 seed, untested for yaw)
    float smc_sta_yaw_phi        = 0.04f;   // [rad/s]
    float smc_sta_yaw_lambda_i   = 1.25f;   // [1/s]
    float smc_sta_yaw_e_reset    = 1.5f;    // [rad/s]

    // z leaky-integration time constant (2026-09-11, docs/plans/
    // smc-rate-loop-plan.md §7.18 -- safety mechanism added after §7.17's
    // SILS finding that a sustained same-sign disturbance made z grow
    // unboundedly and the craft tumbled at t=33s). NOT 0 -- unlike this
    // session's other opt-in features, z_leak_tau=0 (pure integration, the
    // pre-§7.18 behavior) is exactly what caused the tumble, so it is not a
    // safe default here. See smc_rate_sta.hpp's z_trial comment for the
    // derivation and |z_eq|=k2*z_leak_tau bound.
    // zの漏れ積分時定数（2026-09-11、docs/plans/smc-rate-loop-plan.md
    // §7.18 -- §7.17のSILS実測（持続的な同符号外乱でzが歯止めなく成長し
    // t=33sで機体が転倒）を受けて追加した安全機構）。0ではない——このセッシ
    // ョンの他のopt-in機能と異なり、z_leak_tau=0（純粋積分、§7.18以前の
    // 挙動）こそが転倒を招いた挙動そのものなので、ここでは安全な既定値に
    // ならない。導出と|z_eq|=k2*z_leak_tauの境界はsmc_rate_sta.hppの
    // z_trialコメント参照。
    //
    // 2026-09-11 roll/pitch UPDATED 1.0 -> 0.5 (docs/plans/
    // smc-rate-loop-plan.md §7.23): a z_leak_tau sweep {0.5, 0.75, 1.0, 1.5}
    // against the §7.21 sustained-disturbance stress test (60s roll hold,
    // --torque-authority 0.4) found 1.0 sits in a narrow, non-monotonic
    // instability notch -- it alone tumbles at t=57.2s, while 0.5/0.75/1.5
    // all survive the full 70s run. 0.5 was chosen and re-verified against
    // all 6 of §7.18's original regression conditions (stab_flight/
    // pos_flight/stab_combined_aggressive x nominal/motor-delay=15ms): all
    // PASS, with att_rmse improving in 3 of 6 and staying well inside gates
    // on the other 3 (worst case pos_flight md15 att_rmse 0.53->1.01,
    // gate is <5.0). yaw is left at 1.0 (untested -- the sweep only
    // exercised roll; changing yaw's value would need its own sustained-
    // disturbance verification first, per CLAUDE.md's simulation-backing
    // rule).
    // 2026-09-11 roll/pitchを1.0→0.5へ更新（docs/plans/
    // smc-rate-loop-plan.md §7.23）: §7.21の持続外乱ストレス試験（60秒
    // ロール保持、--torque-authority 0.4）に対しz_leak_tau掃引
    // {0.5, 0.75, 1.0, 1.5}を実施したところ、1.0だけが狭い非単調な不安定
    // 領域に入っており、単独でt=57.2sに転倒する一方、0.5/0.75/1.5は
    // いずれも70秒完走した。0.5を採用し、§7.18の元の6回帰条件
    // （stab_flight/pos_flight/stab_combined_aggressive×nominal/
    // motor-delay=15ms）全てで再検証——全てPASS、att_rmseは6条件中3条件で
    // 改善、残り3条件もゲート内に十分収まる（最悪ケースはpos_flight md15の
    // att_rmse 0.53→1.01、ゲートは<5.0）。yawは未検証のため1.0のまま据え
    // 置き（掃引はrollのみで実施——yawの値を変えるにはCLAUDE.mdの
    // シミュレーション裏付け原則に従い別途持続外乱検証が必要）。
    float smc_sta_roll_z_leak_tau  = 0.5f;  // [s]
    float smc_sta_pitch_z_leak_tau = 0.5f;  // [s]
    float smc_sta_yaw_z_leak_tau   = 1.0f;  // [s] -- untested, see note above

    // smc_pos_sta gains (firmware/apps/smc_pos_sta, sliding_mode_sta.hpp's
    // generic SuperTwisting struct, shared by 3 rate axes + 2 velocity axes
    // -- docs/plans/smc-rate-loop-plan.md §7 / §7.11-7.24 for the STA design
    // and §7.9 for why the ORIGINAL 1st-order velocity-loop SMC (smc_pos)
    // was shelved, which this app reconsiders using STA instead). Own,
    // independent key space from smc_sta.*/smc.velx.*/smc.vely.* -- unused
    // by the default vehicle/smc_rate/smc_rate_sta/smc_pos builds.
    //
    // SEED VALUES ONLY, NOT YET SILS-TUNED (2026-09-11, just implemented --
    // same status smc_rate_sta's own round-1 seed had before §7.12-7.20's
    // multi-round tuning). Rate axes: copied directly from smc_rate_sta's
    // CURRENT (already multi-round-tuned) values -- reusing a proven
    // starting point rather than re-deriving from scratch. Velocity axes:
    // derived by applying the SAME ratio the rate loop's 1st-order-SMC ->
    // STA transition used (k1/eta_1st ~= 60/180 ~= 0.33, k2/eta_1st ~=
    // 30/180 ~= 0.167, phi ratio ~= 0.02/0.15 ~= 0.133) to the EXISTING
    // 1st-order velocity-loop SMC's k/eta/phi (smc.velx.k=0.2,
    // smc.velx.eta=1.0, smc.velx.phi=0.25) -- k1~=0.33*1.0~=0.3,
    // k2~=0.167*1.0~=0.15, phi~=0.25*0.133~=0.03 (rounded). lambda_i/
    // e_reset carried over unchanged from the 1st-order velocity design
    // (0.5/1.25) since those didn't change between smc_rate's 1st-order and
    // smc_rate_sta's STA design either. z_leak_tau seeded at 0.5s (matches
    // smc_rate_sta's tuned roll/pitch value) for all 5 axes -- enabled from
    // the start (see sliding_mode_sta.hpp's header for why, unlike
    // smc_rate_sta's history of adding it only after §7.17's tumble).
    // EXPECT MULTIPLE SILS TUNING ROUNDS before any real-hardware
    // consideration, matching smc_rate_sta's own §7.11-7.20 process.
    // smc_pos_staのゲイン（firmware/apps/smc_pos_sta、sliding_mode_sta.hppの
    // 汎用SuperTwisting構造体、レート3軸+速度2軸で共有 -- docs/plans/
    // smc-rate-loop-plan.md §7・§7.11-7.24がSTA設計、§7.9が元の1次速度ループ
    // SMC（smc_pos）を見送った理由——本appはSTAで再検討する）。
    // smc_sta.*/smc.velx.*/smc.vely.*とは独立したキー空間——既定vehicle/
    // smc_rate/smc_rate_sta/smc_posビルドでは未使用。
    //
    // シード値のみ、SILS未チューニング（2026-09-11、実装直後——
    // smc_rate_sta自身の§7.12-7.20の複数ラウンドチューニング前のラウンド1
    // シードと同じ位置づけ）。レート軸: smc_rate_staの現行値（複数ラウンド
    // 調整済み）をそのまま流用——ゼロから再導出せず実績ある出発点を再利用。
    // 速度軸: レートループの1次SMC→STA移行で使った同じ比率
    // （k1/eta_1次~=60/180~=0.33、k2/eta_1次~=30/180~=0.167、
    // phi比~=0.02/0.15~=0.133）を既存1次速度ループSMCのk/eta/phi
    // （smc.velx.k=0.2, smc.velx.eta=1.0, smc.velx.phi=0.25）に適用して算出
    // ——k1~=0.33*1.0~=0.3、k2~=0.167*1.0~=0.15、phi~=0.25*0.133~=0.03
    // （四捨五入）。lambda_i/e_resetは1次速度設計（0.5/1.25）からそのまま
    // 引き継ぎ（smc_rateの1次設計→smc_rate_staのSTA設計間でも変えなかった
    // ため）。z_leak_tauは5軸とも0.5s（smc_rate_staのroll/pitch調整済み値と
    // 同じ）で最初から有効（sliding_mode_sta.hppのヘッダ参照——
    // smc_rate_staが§7.17の転倒後に初めて追加した経緯とは異なる）。実機投入を
    // 検討する前に複数ラウンドのSILSチューニングを想定する（smc_rate_sta
    // 自身の§7.11-7.20と同じプロセス）。
    float smc_pos_sta_roll_k1        = 60.0f;   // [rad/s^2 per sqrt(rad/s)] seed = smc_rate_sta current
    float smc_pos_sta_roll_k2        = 30.0f;   // [rad/s^3] seed = smc_rate_sta current
    float smc_pos_sta_roll_phi       = 0.02f;   // [rad/s]
    float smc_pos_sta_roll_lambda_i  = 6.0f;    // [1/s]
    float smc_pos_sta_roll_e_reset   = 0.75f;   // [rad/s]
    float smc_pos_sta_roll_z_leak_tau = 0.5f;   // [s]
    float smc_pos_sta_pitch_k1       = 60.0f;   // [rad/s^2 per sqrt(rad/s)] seed = smc_rate_sta current
    float smc_pos_sta_pitch_k2       = 30.0f;   // [rad/s^3] seed = smc_rate_sta current
    float smc_pos_sta_pitch_phi      = 0.02f;   // [rad/s]
    float smc_pos_sta_pitch_lambda_i = 6.0f;    // [1/s]
    float smc_pos_sta_pitch_e_reset  = 0.75f;   // [rad/s]
    float smc_pos_sta_pitch_z_leak_tau = 0.5f;  // [s]
    float smc_pos_sta_yaw_k1         = 15.2f;   // [rad/s^2 per sqrt(rad/s)] seed = smc_rate_sta current
    float smc_pos_sta_yaw_k2         = 7.4f;    // [rad/s^3] seed = smc_rate_sta current
    float smc_pos_sta_yaw_phi        = 0.04f;   // [rad/s]
    float smc_pos_sta_yaw_lambda_i   = 1.25f;   // [1/s]
    float smc_pos_sta_yaw_e_reset    = 1.5f;    // [rad/s]
    float smc_pos_sta_yaw_z_leak_tau = 0.5f;    // [s]
    // round-2 (docs/plans/smc-rate-loop-plan.md §7.25): the round-1 seed
    // (k1=0.3/k2=0.15) consistently failed pos_roll/pos_pitch/pos_flight's
    // horizontal_drift_max gate (~3.0-3.6 vs <3.0) despite the best
    // tilt_max/att_rmse of all 3 compared controllers. A k1/k2 scale sweep
    // {2x,3x,4x,6x} on pos_roll found 2x (k1=0.6/k2=0.3) already clears the
    // gate with margin (drift 1.85) while keeping tilt_max lowest (3.70 vs
    // 8.62 PID / 6.52 1st-order SMC) -- higher scales reduce drift further
    // but trade away tilt_max toward the other controllers' worse values.
    // Re-verified at 2x across pos_roll/pos_pitch/pos_flight/pos_yaw/
    // pos_reposition/pos_auto_takeoff: all PASS (pos_flight's DISARM/order
    // FAIL is pos_flight.scn's own pre-existing known issue, present in the
    // PID baseline too -- unrelated to this app).
    // ラウンド2（docs/plans/smc-rate-loop-plan.md §7.25）: ラウンド1シード
    // （k1=0.3/k2=0.15）はpos_roll/pos_pitch/pos_flightのhorizontal_drift_max
    // ゲートで一貫してFAIL（~3.0-3.6、ゲートは<3.0）——tilt_max/att_rmseは
    // 比較3者中最良だったにもかかわらず。k1/k2のスケール掃引{2x,3x,4x,6x}を
    // pos_rollで実施したところ、2x（k1=0.6/k2=0.3）で既に余裕を持って
    // ゲート通過（drift=1.85）しつつtilt_maxも最小（3.70、PID 8.62・1次SMC
    // 6.52より良い）——それ以上のスケールはdriftをさらに減らすがtilt_maxが
    // 他の制御則の悪い値へ近づくトレードオフ。2xをpos_roll/pos_pitch/
    // pos_flight/pos_yaw/pos_reposition/pos_auto_takeoffで再検証——全てPASS
    // （pos_flightのDISARM/order FAILはpos_flight.scn自身の既知の既存問題、
    // PIDベースラインでも同じ——本app固有ではない）。
    float smc_pos_sta_velx_k1        = 0.6f;    // [m/s^2 per sqrt(m/s)] round-2
    float smc_pos_sta_velx_k2        = 0.3f;    // [m/s^3] round-2
    float smc_pos_sta_velx_phi       = 0.03f;   // [m/s]
    float smc_pos_sta_velx_lambda_i  = 0.5f;    // [1/s]
    float smc_pos_sta_velx_e_reset   = 1.25f;   // [m/s]
    float smc_pos_sta_velx_z_leak_tau = 0.5f;   // [s]
    float smc_pos_sta_vely_k1        = 0.6f;    // [m/s^2 per sqrt(m/s)] round-2
    float smc_pos_sta_vely_k2        = 0.3f;    // [m/s^3] round-2
    float smc_pos_sta_vely_phi       = 0.03f;   // [m/s]
    float smc_pos_sta_vely_lambda_i  = 0.5f;    // [1/s]
    float smc_pos_sta_vely_e_reset   = 1.25f;   // [m/s]
    float smc_pos_sta_vely_z_leak_tau = 0.5f;   // [s]

    // Adaptive-gain super-twisting rate-loop gains (firmware/apps/
    // smc_rate_asta, AdaptiveSuperTwistingRate) -- see smc_rate_asta.hpp
    // for the control law and docs/plans/smc-rate-loop-plan.md §7.31 for
    // the motivation (§3.7's non-monotonic fixed-gain trade-off between
    // torque-authority=0.4/0.55 robustness and motor-delay=15ms stability
    // in smc_rate_sta) and results. Own, independent key space from
    // smc_sta.*/smc_pos_sta.* -- smc_rate_sta itself is UNCHANGED, this is
    // a separate app per explicit user instruction (2026-09-12).
    //
    // SEED VALUES ONLY, NOT YET SILS-TUNED (2026-09-12, just implemented).
    // k1_init/phi/lambda_i/e_reset/z_leak_tau: copied from smc_rate_sta's
    // CURRENT tuned k1/phi/lambda_i/e_reset/z_leak_tau (reusing a proven
    // starting point). k2_ratio: smc_rate_sta's own k2/k1 ratio (30/60=0.5
    // roll/pitch, 7.4/15.2~=0.487 yaw, rounded to 0.5 for all 3 -- k2 is
    // DERIVED from k1 each cycle, not independently tuned). k1_min: half of
    // k1_init (never adapt below a baseline that still has some authority).
    // k1_max: 2.5x k1_init (a round number giving headroom above the
    // fixed-gain values that section 3.3 found still fell short of fully
    // solving torque-authority=0.4/0.55 -- not derived from a specific
    // verified operating point). adapt_rate/leak_ratio/dead_band: new
    // parameters this design introduces (no fixed-gain analog to copy);
    // dead_band=0.05 rad/s is a rate-error scale guess to avoid growing k1
    // on residual sensor/discretization noise near s=0 (see §7.30続報6/8's
    // UNRELATED finding that a different filter, the ESKF's accel-comp
    // tracker, showed noise-driven gain-like growth risk -- applied here as
    // a design precaution only); adapt_rate=20 (roll/pitch) lets k1 traverse
    // its full [k1_min,k1_max] range in a few seconds; yaw's adapt_rate is
    // scaled down by yaw's smaller k1 (15.2/60~=0.25x) as a rough starting
    // guess. EXPECT SILS TUNING before any real-hardware consideration,
    // matching smc_rate_sta's own §7.11-7.20 process -- see the verification
    // plan in docs/plans/smc-rate-loop-plan.md §7.31.
    // 適応ゲイン・スーパーツイスティング・レートループのゲイン
    // （firmware/apps/smc_rate_asta、AdaptiveSuperTwistingRate）-- 制御則は
    // smc_rate_asta.hpp、動機と結果はdocs/plans/smc-rate-loop-plan.md §7.31
    // 参照（§3.7のtorque-authority=0.4/0.55頑健性とmotor-delay=15ms安定性の
    // 間の非単調な固定ゲイントレードオフ）。smc_sta.*/smc_pos_sta.*とは
    // 独立したキー空間——smc_rate_sta自体は無変更、ユーザーの明示的指示
    // （2026-09-12）により別appとする。
    //
    // シード値のみ、SILS未チューニング（2026-09-12、実装直後）。
    // k1_init/phi/lambda_i/e_reset/z_leak_tau: smc_rate_staの現行調整済み
    // 値をそのまま流用（実績ある出発点の再利用）。k2_ratio: smc_rate_sta
    // 自身のk2/k1比（roll/pitch=30/60=0.5、yaw=7.4/15.2~=0.487、3軸とも
    // 0.5に丸め——k2はk1から毎サイクル導出し独立調整しない）。k1_min:
    // k1_initの半分（権限が全く無くなる下限まで下げない）。k1_max:
    // k1_initの2.5倍（§3.3でtorque-authority=0.4/0.55を完全には解決
    // しきれなかった固定ゲイン値より余裕を持たせたキリの良い数——特定の
    // 検証済み動作点から導出したものではない）。adapt_rate/leak_ratio/
    // dead_band: 本設計で新規導入するパラメータ（コピー元となる固定ゲイン
    // 版の対応値はない）——dead_band=0.05rad/sはs=0近傍の残留センサ/離散化
    // ノイズでk1が成長し続けないようにするレート誤差スケールの推測値
    // （§7.30続報6/8で見つけた別件——ESKFのaccel-compトラッカーにおける
    // ノイズ駆動的なゲイン類似成長リスク——を設計上の予防措置として参考に
    // したのみで無関係）。adapt_rate=20（roll/pitch）はk1が[k1_min,k1_max]
    // 全域を数秒で走査できる値。yawのadapt_rateはyawのk1が小さいこと
    // （15.2/60~=0.25倍）に合わせて粗く縮小した出発値。実機投入検討前に
    // SILSチューニングを想定する（smc_rate_sta自身の§7.11-7.20と同じ
    // プロセス）——検証計画はdocs/plans/smc-rate-loop-plan.md §7.31参照。
    float smc_asta_roll_k1_init    = 60.0f;   // [rad/s^2 per sqrt(rad/s)] seed = smc_rate_sta current k1
    float smc_asta_roll_k1_min     = 30.0f;   // half of k1_init
    float smc_asta_roll_k1_max     = 150.0f;  // 2.5x k1_init
    float smc_asta_roll_k2_ratio   = 0.5f;    // seed = smc_rate_sta's k2/k1
    float smc_asta_roll_adapt_rate = 20.0f;   // [1/s, same units as k1] NEW parameter, unverified
    float smc_asta_roll_leak_ratio = 0.2f;    // NEW parameter, unverified
    // dead_band/filter_tau (2026-09-12, docs/plans/smc-rate-loop-plan.md
    // section 7.31/7.31続報/7.31続報2): a raw-|s| dead-band could not
    // satisfy both torque-authority=0.4 (needs k1 to grow) and noise n1
    // (should NOT grow k1) with a single k1_max. Filtering the SIGNED s
    // (NOT |s| -- an earlier version's sign error) before the dead-band
    // comparison (smc_rate_asta.hpp's s_lpf) separates the two by time
    // structure. A 27-point grid (k1_max x dead_band x filter_tau) plus an
    // 18-point local refinement (110+ SILS runs total) found k1_max is
    // IRRELEVANT once this filter is in place (k1 never reaches even the
    // lowest k1_max tried) and dead_band=0.03/filter_tau=0.3 is the best
    // point: torque-authority=0.4 PASSES (2.69deg) and noise n1 is the
    // closest of any setting tried (3.31deg vs the <3.0deg gate) but still
    // FAILS -- a local optimum, not a full solution. Adopted as the new
    // seed since it's strictly better than the original 0.05/0.05 seed on
    // both axes; yaw untested, left at its original seed.
    // dead_band/filter_tau（2026-09-12、docs/plans/smc-rate-loop-plan.md
    // §7.31/§7.31続報/§7.31続報2）: 生の|s|での不感帯判定では、単一の
    // k1_maxでtorque-authority=0.4（k1を成長させたい）とnoise n1
    // （成長させたくない）を両立できなかった。不感帯判定の前に**符号付き**
    // s（|s|ではない——初期実装の符号の誤り）をフィルタする
    // （smc_rate_asta.hppのs_lpf）ことで時間構造の違いを利用する。27点の
    // 格子探索（k1_max×dead_band×filter_tau）+18点の近傍探索（計110回超の
    // SILS実行）の結果、このフィルタ導入後はk1_maxが無関係と判明（k1が
    // 最も低いk1_max候補にすら到達しない）、dead_band=0.03/filter_tau=0.3
    // が最良点: torque-authority=0.4はPASS（2.69°）、noise n1も全設定中
    // 最良（3.31°、ゲート<3.0°には未達）——局所最適であり完全な解決では
    // ない。両軸で旧来の0.05/0.05シードより明確に優れるため新シードとして
    // 採用。yawは未検証のため旧シードのまま。
    float smc_asta_roll_dead_band  = 0.03f;   // [rad/s] tuned seed (see comment above) -- was 0.05
    float smc_asta_roll_filter_tau = 0.3f;    // [s] tuned seed (see comment above) -- was 0.05
    float smc_asta_roll_phi        = 0.02f;   // [rad/s] seed = smc_rate_sta current
    float smc_asta_roll_lambda_i   = 6.0f;    // [1/s] seed = smc_rate_sta current
    float smc_asta_roll_e_reset    = 0.75f;   // [rad/s] seed = smc_rate_sta current
    float smc_asta_roll_z_leak_tau = 0.5f;    // [s] seed = smc_rate_sta current
    float smc_asta_pitch_k1_init    = 60.0f;
    float smc_asta_pitch_k1_min     = 30.0f;
    float smc_asta_pitch_k1_max     = 150.0f;
    float smc_asta_pitch_k2_ratio   = 0.5f;
    float smc_asta_pitch_adapt_rate = 20.0f;
    float smc_asta_pitch_leak_ratio = 0.2f;
    float smc_asta_pitch_dead_band  = 0.03f;   // tuned seed, see roll's comment above -- was 0.05
    float smc_asta_pitch_filter_tau = 0.3f;    // tuned seed, see roll's comment above -- was 0.05
    float smc_asta_pitch_phi        = 0.02f;
    float smc_asta_pitch_lambda_i   = 6.0f;
    float smc_asta_pitch_e_reset    = 0.75f;
    float smc_asta_pitch_z_leak_tau = 0.5f;
    float smc_asta_yaw_k1_init    = 15.2f;
    float smc_asta_yaw_k1_min     = 7.6f;
    float smc_asta_yaw_k1_max     = 38.0f;
    float smc_asta_yaw_k2_ratio   = 0.5f;
    float smc_asta_yaw_adapt_rate = 5.0f;    // scaled down from roll/pitch by ~yaw's k1/roll's k1
    float smc_asta_yaw_leak_ratio = 0.2f;
    float smc_asta_yaw_dead_band  = 0.05f;
    float smc_asta_yaw_filter_tau = 0.05f;
    float smc_asta_yaw_phi        = 0.04f;
    float smc_asta_yaw_lambda_i   = 1.25f;
    float smc_asta_yaw_e_reset    = 1.5f;
    float smc_asta_yaw_z_leak_tau = 1.0f;

    // Sliding-mode horizontal-VELOCITY-loop gains (firmware/apps/smc_pos,
    // smc_vel.hpp) -- plugged into PidController's vel_x_/vel_y_ stage via
    // setVelocityLawOverride() (pid_controller.hpp). Unused by the default
    // vehicle/smc_rate builds. docs/plans/smc-rate-loop-plan.md §7.
    //
    // Seed derivation (NOT YET SILS-validated -- seed values only, same
    // status as smc_rate's original round-1 seed): matched to the CURRENT,
    // already-hardware-robustified linear PID (position.vel.kp=3.0,
    // position.vel.ti=2.0 -- firmware/vehicle/docs/poshold_journey.md §4.4).
    // Near the origin (|s|<phi) the reaching law's linear gain is
    // k/phi + eta; picking phi=0.15 m/s and splitting k/phi and eta roughly
    // evenly (k=0.45 -> k/phi=3.0, eta=3.0) matches vel.kp's P-gain magnitude
    // while keeping a real switching-term share (unlike smc_rate's
    // eta-dominant split, this app's authors chose an even split as a
    // starting point -- SILS will show whether that needs to shift toward
    // smc_rate's eta-heavy pattern). lambda_i seeded at 1/ti = 0.5 (same
    // "1/T_i" logic used for smc_rate.lambda_i's PI-surface integral gain).
    // e_reset = 5*phi (SS3.6 convention, same as smc_rate) = 0.75 m/s.
    // スライディングモード・水平速度ループのゲイン（firmware/apps/smc_pos,
    // smc_vel.hpp）-- setVelocityLawOverride()（pid_controller.hpp）経由で
    // PidControllerのvel_x_/vel_y_段へ注入される。既定vehicle/smc_rateビルド
    // では未使用。docs/plans/smc-rate-loop-plan.md §7。
    //
    // 初期値の導出（SILS未検証 -- あくまで初期値、smc_rateの最初のラウンド1
    // 初期値と同じ位置づけ）: 現行の、既に実機で頑健化済みの線形PID
    // （position.vel.kp=3.0, position.vel.ti=2.0 --
    // firmware/vehicle/docs/poshold_journey.md §4.4）に合わせた。原点近傍
    // （|s|<phi）では到達則の線形ゲインはk/phi+eta——phi=0.15m/sとし、
    // k/phiとetaをほぼ均等に分割（k=0.45 -> k/phi=3.0, eta=3.0）することで
    // vel.kpのPゲインの大きさに合わせつつ、スイッチング項にも実質的な配分を
    // 残した（smc_rateのeta優勢な配分とは異なり、本appでは均等分割を出発点に
    // 選んだ -- SILSでsmc_rate流のeta優勢パターンへ寄せる必要が出るかは
    // 検証課題）。lambda_iは1/ti=0.5で初期化（smc_rate.lambda_iのPI面積分
    // ゲインと同じ「1/T_i」の考え方）。e_reset=5*phi（§3.6の慣例、smc_rateと
    // 同じ）=0.75 m/s。
    // Round-2 (2026-09-11, docs/plans/smc-rate-loop-plan.md §7.4): round-1's
    // eta=3.0 seed (matched to position.vel.kp) saturated duty (1.00, FAIL
    // pos_flight's <0.9 gate) regardless of k -- a 4-point sweep on
    // pos_flight (k,eta,phi) = (0.3,1.5,0.2)/(0.2,1.0,0.25)/(0.1,2.5,0.3)/
    // (0,3.0,0.15) found only (0.2,1.0,0.25) clears ALL 4 gates
    // (duty_max=0.78, att_rmse=0.51, drift=1.25, tilt=13.9). The pattern: eta
    // needs to be well BELOW the linear PID's kp=3.0 -- a softer velocity
    // loop that leans on the switching term k*sat(s/phi) (wider phi=0.25)
    // rather than a large linear gain, which was what drove duty into
    // saturation. e_reset rescaled with phi (5*phi convention, SS3.6) ->
    // 1.25. lambda_i left at round-1's 0.5 (not yet swept).
    // ラウンド2（2026-09-11、docs/plans/smc-rate-loop-plan.md §7.4）:
    // ラウンド1のeta=3.0シード（position.vel.kpに合わせた値）はkに関わらず
    // duty飽和（1.00、pos_flightの<0.9ゲートにFAIL）——pos_flightで
    // (k,eta,phi)=(0.3,1.5,0.2)/(0.2,1.0,0.25)/(0.1,2.5,0.3)/(0,3.0,0.15)の
    // 4点スイープを行い、(0.2,1.0,0.25)のみが4ゲート全てをクリア
    // （duty_max=0.78, att_rmse=0.51, drift=1.25, tilt=13.9）。パターン:
    // etaは線形PIDのkp=3.0より十分低くする必要がある——duty飽和を招いていた
    // 大きな線形ゲインでなく、スイッチング項k*sat(s/phi)（広いphi=0.25）に
    // 寄りかかる柔らかい速度ループの方が良い。e_resetはphiに合わせ再計算
    // （5*phiの慣例、§3.6）->1.25。lambda_iはラウンド1の0.5のまま
    // （未スイープ）。
    float smc_velx_k        = 0.2f;   // [m/s^2] round-2 (was 0.45)
    float smc_velx_eta      = 1.0f;   // [1/s] round-2 (was 3.0)
    float smc_velx_phi      = 0.25f;  // [m/s] round-2 (was 0.15)
    float smc_velx_lambda_i = 0.5f;   // [1/s]
    float smc_velx_e_reset  = 1.25f;  // [m/s] round-2 (was 0.75, = 5*phi)
    float smc_vely_k        = 0.2f;   // [m/s^2] round-2 (was 0.45)
    float smc_vely_eta      = 1.0f;   // [1/s] round-2 (was 3.0)
    float smc_vely_phi      = 0.25f;  // [m/s] round-2 (was 0.15)
    float smc_vely_lambda_i = 0.5f;   // [1/s]
    float smc_vely_e_reset  = 1.25f;  // [m/s] round-2 (was 0.75, = 5*phi)

    // Dead-time predictor for the velocity-loop SMC (2026-09-11, tried on
    // user request to check whether compensating both the rate loop AND
    // this loop helps beyond rate-loop-only compensation -- see
    // smc_vel.hpp's field comment and docs/plans/smc-rate-loop-plan.md §7.7c
    // for the hypothesis and results). Default 0 = disabled.
    // 速度ループSMCの無駄時間予測補償器（2026-09-11、レートループのみの
    // 補償を超える効果があるかユーザー要請で試行 -- 仮説と結果は
    // smc_vel.hppのフィールドコメントとdocs/plans/smc-rate-loop-plan.md
    // §7.7c参照）。既定0=無効。
    float smc_velx_delay_comp_ms = 0.0f;  // [ms]
    float smc_vely_delay_comp_ms = 0.0f;  // [ms]

    // Scheduled autotune (solo pilot, hands-free): a single operator cannot type
    // `autotune` mid-flight, so SET these on the GROUND, then arm and fly. After the
    // craft has been FLYING for sched_delay seconds, the rate-loop autotune runs
    // automatically on sched_axis (a beep cues the pilot to hold a steady hover).
    // One-shot per flight; -1 = OFF. Disable by setting axis back to -1.
    // スケジュール autotune（ソロ操縦・ハンズフリー）: 飛行中に `autotune` を打てないため、
    // 地上で設定→離陸。FLYING 到達から sched_delay 秒後に sched_axis のレート autotune が
    // 自動起動（ブザーで合図、定位置ホバー保持を促す）。1飛行1回・-1=OFF。
    int32_t autotune_sched_axis  = -1;     // -1=off, 0=roll, 1=pitch, 2=yaw
    float   autotune_sched_delay = 20.0f;  // [s] FLYING dwell before firing

    // Autotune system-identification result, per axis. Written by the onboard autotune
    // whenever the plant FIT succeeds (even if the gain design is then rejected, e.g.
    // a thin-margin yaw) — so the identified model is retained for analysis. NOT applied
    // to control (read-back only). Persisted with `param save`. Identified plant per axis:
    //   G(s) = b * e^{-L s} / (s (T s + 1));  b = gain, tau = T [s], delay = L [s],
    //   resid = fit residual (lower = better). 0 = not yet identified.
    // autotune システム同定結果（軸ごと）。プラントのフィット成功時に必ず記録（ゲイン設計が
    // 棄却される軸=余裕の薄い yaw 等でも同定結果は残す）。制御には未使用（読み出し専用）。
    // `param save` で永続。同定プラント: G(s)=b·e^{-Ls}/(s(Ts+1))、tau=T[s]、delay=L[s]、
    // resid=フィット残差（小さいほど良）。0=未同定。
    float autotune_roll_b    = 0.0f, autotune_roll_tau    = 0.0f,
          autotune_roll_delay  = 0.0f, autotune_roll_resid  = 0.0f;
    float autotune_pitch_b   = 0.0f, autotune_pitch_tau   = 0.0f,
          autotune_pitch_delay = 0.0f, autotune_pitch_resid = 0.0f;
    float autotune_yaw_b     = 0.0f, autotune_yaw_tau     = 0.0f,
          autotune_yaw_delay   = 0.0f, autotune_yaw_resid   = 0.0f;

    // Autotune design-margin result, per axis. Written by the onboard autotune right
    // after the loop-shaping design (tunePid) succeeds — BEFORE the GM-floor / gain-range
    // gates — so the margins are kept even when the design is then REJECTED (e.g. a thin
    // yaw GM): you can read WHY it was rejected. Read-back only, persisted with `param save`.
    //   wc = achieved crossover [rad/s], pm = phase margin [deg], gm = gain margin [dB]
    //   (gm = 99 means no −180° crossing in the sweep, i.e. effectively infinite/safe).
    //   0 = not yet designed.
    // autotune 設計余裕結果（軸ごと）。ループ整形設計(tunePid)成功直後＝GM下限/ゲイン範囲ゲートの
    // 前に記録するため、設計が棄却される軸(余裕の薄い yaw 等)でも余裕が残り「なぜ棄却されたか」が
    // 読める。読み出し専用・`param save` で永続。wc=交差[rad/s]、pm=位相余裕[deg]、gm=ゲイン余裕[dB]
    // （gm=99 は掃引中に −180°交差なし＝実質無限大/安全）。0=未設計。
    float autotune_roll_wc  = 0.0f, autotune_roll_pm  = 0.0f, autotune_roll_gm  = 0.0f;
    float autotune_pitch_wc = 0.0f, autotune_pitch_pm = 0.0f, autotune_pitch_gm = 0.0f;
    float autotune_yaw_wc   = 0.0f, autotune_yaw_pm   = 0.0f, autotune_yaw_gm   = 0.0f;

    // Autotune reject-reason code per axis (read-only diagnostic): 0=applied, 1=insufficient
    // coherent data, 2=bad/NaN fit, 3=residual>0.3, 4=out of physical bounds, 5=design
    // infeasible (wc too high), 6=phase margin below target, 7=gain margin below floor,
    // 8=param-table range. / 自動チューン棄却理由コード（軸別・読出専用）。
    float autotune_roll_reject = 0.0f, autotune_pitch_reject = 0.0f, autotune_yaw_reject = 0.0f;

    // Estimator selection (RESET_PLAN P2: replaceable estimation). The IMU task's
    // factory reads this: 0 = ESKF (15-state), 1 = complementary filter. The SILS
    // bench swaps estimators via this parameter alone — no code change.
    // 推定器の選択（P2: 差し替え可能）。IMU タスクのファクトリが読む: 0=ESKF, 1=相補。
    int32_t estimator_type = 0;

    // Telemetry WiFi mode (boot-time, sf_comm initWifi): 0 = STA — join the
    // router whose SSID/password are stored in NVS via the CLI `wifi` command
    // (unconfigured → ESP-NOW-only, telemetry inert); 1 = SoftAP — the vehicle
    // serves "StampFly-XXXX" on the ESP-NOW channel (no infrastructure needed).
    // ESP-NOW control works in every mode.
    // テレメトリ WiFi モード（起動時, sf_comm initWifi）: 0 = STA — CLI `wifi`
    // コマンドで NVS に保存した SSID/パスワードのルータへ接続（未設定なら
    // ESP-NOW のみ・テレメトリ無効）; 1 = SoftAP — 機体が ESP-NOW チャネル上で
    // "StampFly-XXXX" を提供（インフラ不要）。ESP-NOW 操縦は全モードで動く。
    int32_t wifi_mode = 0;

    // WiFi/ESP-NOW channel (boot-time, sf_comm initWifi): 1-13. Used by the SoftAP and
    // the fixed-channel STA/ESP-NOW radio. Reboot to apply (the radio is not re-channeled
    // in flight). Must match the transmitter, but the controller auto-scans 1-13 on
    // pairing and locks onto the channel our pairing packet advertises — so changing this
    // and re-pairing is enough; no controller reflash. Use 6 or 11 to avoid a busy CH 1.
    // WiFi/ESP-NOW チャンネル（起動時, sf_comm initWifi）: 1-13。SoftAP と固定チャネル
    // STA/ESP-NOW 無線が使う。反映には再起動（無線は飛行中に載せ替えない）。送信機と一致が
    // 必要だが、コントローラはペアリング時に 1-13 をスキャンし、ペアリングパケットが広告する
    // チャンネルにロックする — 変更後に再ペアリングするだけでよい（送信機の再書込み不要）。
    // 混雑する CH 1 を避けるなら 6 か 11。
    int32_t wifi_channel = 1;

    // Blackbox SPIFFS logger enable (0 = OFF default, 1 = ON). DEFAULT OFF because the
    // SPIFFS write done while ARMED triggers a flash erase that disables the flash
    // cache and STALLS BOTH CORES ~37ms every ~0.5s — the control loop freezes and the
    // craft drifts (a periodic yaw "kick"). WiFi telemetry (sf log wifi) already covers
    // analysis. Enable only when the onboard log is truly needed and the periodic stall
    // is acceptable. Proper fix (future): buffer in RAM, write on DISARM only.
    // Blackbox SPIFFS ロガー有効化（0=既定OFF, 1=ON）。既定OFF — ARM 中の SPIFFS 書き込みは
    // フラッシュ消去でフラッシュキャッシュを無効化し両コアを ~0.5 秒ごとに ~37ms 停止させる
    // （制御ループ凍結→機体ドリフト＝周期的ヨーキック）。解析は WiFi テレメトリで足りる。
    // 本当に必要かつ周期ストールを許容できる時のみ ON。恒久対策(将来)=RAM 緩衝し DISARM で書込。
    int32_t log_blackbox_enable = 0;

    // Attitude control. att.ti 4.0->2.0 (2026-06-22): pilot-preferred on hardware — a
    // faster attitude integral firms up the tilt-hold feel. (A wobble-flight ID showed
    // the loop-relevant tilt achievement is ~0.58 at the POS_HOLD band, capped by the
    // real motor torque effectiveness ~0.4-0.7x; ti=2 lifts the low-frequency end. The
    // measured POS_HOLD drift RMS was marginally looser (16->20 mm, within flight-to-
    // flight scatter) but the pilot prefers the firmer ti=2 response.) Keep initializer
    // == table default below.
    // 姿勢制御。att.ti 4.0→2.0（2026-06-22）: 実機でパイロットが好む — 速い姿勢積分で傾き保持の
    // 手応えが締まる。（ウォブル同定で POS_HOLD 帯の傾き達成度 ~0.58、実機トルク効き ~0.4-0.7倍で
    // 頭打ち。ti=2 は低域を持ち上げる。POS_HOLD ドリフト RMS は僅かに緩む計測（16→20mm、飛行間
    // ばらつき内）だが、締まった ti=2 の応答をパイロットが好む。）下の table 既定と一致させる。
    float att_roll_kp     = 5.0f;
    float att_roll_ti     = 2.0f;
    float att_roll_td     = 0.04f;
    float att_pitch_kp    = 5.0f;
    float att_pitch_ti    = 2.0f;
    float att_pitch_td    = 0.04f;

    // Attitude trim (STABILIZE and above): equilibrium roll/pitch tilt [rad] added
    // to the angle-loop SETPOINT (not the rate). The craft holds this small tilt to
    // cancel steady horizontal drift from CG offset / sensor-level bias; the angle
    // loop drives the craft there and the inner rate loop costs no extra thrust.
    // The true equilibrium tilt is unknowable on the ground (it depends on CG and
    // thrust asymmetry), so it is identified by FLYING (sf trim analyze). Applies at
    // the attitude confluence for EVERY mode (STABILIZE / ALT_HOLD / POS_HOLD), so
    // POS_HOLD's position loop is relieved of carrying the equilibrium tilt.
    // Default 0.0; limited to ±0.1 rad (±5.7°).
    // 姿勢トリム（STABILIZE 以上）: 角度ループの「目標」に加算する平衡 roll/pitch 傾き [rad]
    // （レートでなく）。CG オフセットやセンサ水平バイアス由来の定常水平ドリフトを打ち消す
    // 小さな傾きを保つ。角度ループが機体をこの傾きへ駆動し、内側レートループは推力を余分に
    // 食わない。真の平衡傾きは地上で知り得ない（CG と推力非対称に依存）ため飛行で同定する
    // （sf trim analyze）。姿勢合流点で全モードに効く（STABILIZE / ALT_HOLD / POS_HOLD）ので、
    // POS_HOLD の位置ループは平衡傾きを担う負担から解放される。既定 0.0、範囲 ±0.1 rad（±5.7°）。
    float trim_roll  = 0.0f;
    float trim_pitch = 0.0f;
    // Onboard trim-learning enable (1 = learn in hover, 0 = off / manual only). The
    // learner relies on the ESKF horizontal velocity (= optical flow); turn it OFF
    // when the flow is unreliable (low-texture/dark floor, too high) so a bad velocity
    // does not mis-learn the trim, or to tune by hand only. Default 1.
    // オンボード・トリム学習の有効化(1=ホバー中に学習, 0=オフ/手動のみ)。学習器は ESKF
    // 水平速度(=オプティカルフロー)に依存するので、フローが不安定(低テクスチャ/暗い床・
    // 高すぎ)なときはオフにし悪い速度で誤学習させない。手動のみで詰めるときも。既定 1。
    int32_t trim_learn = 1;

    // Heading hold (STABILIZE+, yaw stick neutral): P gain [1/s] on the estimator
    // yaw and the correction turn-rate limit [rad/s]. kp=0 disables the hold.
    // Defaults from the 2026-06-11 flight-log replay (excursion 12.3°→5.7° mean).
    // ヘディングホールド（STABILIZE 以上・ヨースティック中立時）: 推定ヨー角への
    // P ゲイン [1/s] と補正回頭率上限 [rad/s]。kp=0 で無効。既定値は 2026-06-11 の
    // フライトログ再生で決定（方位ずれ平均 12.3°→5.7°）。
    float att_yawhold_kp       = 3.0f;
    float att_yawhold_rate_max = 2.0f;

    // Altitude control — cascade alt → vertical-velocity → thrust [N].
    // Values are the FLIGHT-PROVEN legacy vehicle/ gains (config.hpp
    // altitude_control, "PI-v1": alt 0.6/7.0 → vel 0.1/2.5). The legacy velocity
    // loop also output physical thrust [N] (VEL_OUTPUT_MAX 0.15 N), so the units
    // match and the gains transfer 1:1. The earlier, stronger SILS-tuned values
    // (1.5/8 → 0.3/2) are superseded by the hardware-proven set.
    // 高度制御 — カスケード 高度→鉛直速度→推力[N]。値は旧 vehicle/ の飛行実績ゲイン
    // （config.hpp altitude_control「PI-v1」: alt 0.6/7.0 → vel 0.1/2.5）。旧の速度
    // ループも物理推力[N]出力（VEL_OUTPUT_MAX 0.15N）で単位が一致し、そのまま移植
    // できる。以前の強めの SILS 調整値（1.5/8 → 0.3/2）は実機実績値で置換。
    // alt.kp 0.6->0.45 (2026-06-22): real-flight tuned to damp a slow altitude bob.
    // The bob is the altitude loop being marginally under-damped against the ~110 ms
    // MOTOR/PROP ACTUATION lag (thrust cmd -> actual vertical accel, MEASURED from a
    // hover log; the ToF at 30 Hz and the ESKF vertical velocity are both clean/un-lagged,
    // so the lag is in the thrust path, not sensing). Lowering alt.kp drops the loop
    // crossover -> more phase margin against the lag -> the bob damps. Sweet spot: a sim
    // sweep + flight both put the best damping at 0.45 (0.4/0.35 are worse; raising
    // alt.vel.kp HURTS because the damping path sees the same actuation lag). Real flight:
    // altitude RMS 53->33 mm (-38%), period 5->11.5 s. Residual ~±6 cm long-period bob is
    // the actuation-lag limit (hardware). KEEP EQUAL to the table default. See
    // poshold_accel_compensation.md remaining issue #7.
    // alt.kp 0.6→0.45（2026-06-22）: 遅い高度上下動を減衰させる実機調整。上下動は高度ループが
    // モータ/プロペラの応答遅れ ~110ms（推力指令→実鉛直加速度、ホバーログで実測。ToF 30Hz も
    // ESKF 鉛直速度も健全・無遅れゆえ遅れは推力経路）に対し減衰不足なため。alt.kp を下げると
    // ループのクロスオーバーが下がり位相余裕が増えて上下動が減衰。最適点は sim+実機とも 0.45
    // （0.4/0.35 は悪化、alt.vel.kp を上げるのは逆効果＝減衰経路が同じ遅れを見るため）。実機:
    // 高度 RMS 53→33mm(−38%)、周期 5→11.5s。残る±6cm 長周期はアクチュエーション遅れの限界（ハード）。
    float alt_alt_kp      = 0.45f;
    float alt_alt_ti      = 7.0f;
    float alt_vel_kp      = 0.1f;
    float alt_vel_ti      = 2.5f;
    // Phase-scheduled hover-only vel-loop Ti (VerticalPhase::Airborne).
    // DEFAULT 2.5 = no-op (reverted 2026-07-18 by pilot operational decision,
    // same day as the brief default-1.5 promotion; 1.5 remains the
    // flight-validated opt-in, especially in combination with the DOB).
    // 既定2.5=no-op（2026-07-18 パイロット運用判断で同日中の既定1.5昇格を取り消し。
    // 1.5は実飛行検証済みのopt-in値のまま、特にDOB併用時に有効）。Rationale: real
    // hover shows low-freq battery-sag thrust disturbance that a shorter Ti
    // rejects, but a uniformly shorter alt_vel_ti worsens auto-takeoff
    // capture overshoot (integrator windup, +60% in sim/flight). Splitting
    // Ti by phase (climb=alt_vel_ti, hover=alt_vel_ti_hover) keeps
    // TakeoffClimb unchanged (SILS isolation proof: takeoff overshoot
    // unchanged 9.96→9.22 cm with ti_hover=1.5). Flight A/B 2026-07-17
    // (logs 224906/231940): standalone effect is band reshaping (~neutral
    // total std) — but WITH the altitude DOB (altitude.dob.fc, default-on
    // since 2026-07-18) the 1.5/2.5 choice matters: same-log replay gives
    // DOB+ti1.5 = 94.2 mm vs DOB+ti2.5 = 107.1 mm, and the -67% flight
    // validation (log 022929) flew the 1.5 combination. Promoted so a
    // `param reset` lands on the flight-validated combo.
    // See PidController::applyAltVelTiForPhase() (architecture.md INV-1).
    // フェーズ別 hover 専用の速度ループ Ti（VerticalPhase::Airborne）。
    // 既定1.5（2026-07-18 昇格。旧2.5=no-op）。根拠: 実ホバーの低周波電池サグ
    // 外乱は短いTiで除去できるが、alt_vel_ti の一律短縮は自動離陸の捕捉オーバー
    // シュートを悪化させる（積分巻き上がり、シム/実機+60%）。フェーズ分離
    // （climb=alt_vel_ti, hover=alt_vel_ti_hover）で TakeoffClimb は不変
    // （SILS分離実証: ti_hover=1.5 で離陸OS不変 9.96→9.22cm）。実飛行A/B
    // 2026-07-17（ログ224906/231940）: 単独では帯域再配分（合計stdほぼ中立）
    // だが、高度DOB（altitude.dob.fc、2026-07-18から既定有効）併用では 1.5/2.5
    // の差が効く: 同一ログ再生で DOB+ti1.5=94.2mm vs DOB+ti2.5=107.1mm。
    // −67%の実飛行検証（ログ022929）も 1.5 の組合せで飛行。`param reset` が
    // 実飛行検証済みの組合せに戻るよう既定へ昇格。
    // PidController::applyAltVelTiForPhase() 参照（architecture.md INV-1）。
    float alt_vel_ti_hover = 2.5f;
    // ALT_HOLD manual stick rates (separately tunable). The throttle stick is
    // spring-centred (centre = hold); push up → climb at climb_rate, push down →
    // descend at descent_rate. Mirrors the flight-proven legacy vehicle's
    // MAX_CLIMB_RATE / MAX_DESCENT_RATE (separate constants).
    // ALT_HOLD の手動スティック速度（別々にチューニング可）。スロットルはバネ中央
    // （中央=ホールド）、上=climb_rate で上昇・下=descent_rate で降下。旧 vehicle の
    // MAX_CLIMB_RATE / MAX_DESCENT_RATE（別定数）を踏襲。
    float alt_climb_rate   = 0.5f;   // [m/s]
    float alt_descent_rate = 0.5f;   // [m/s]

    // Acceleration-based disturbance observer (DOB) cutoff for the Airborne
    // altitude vel loop. DEFAULT 0 = DISABLED (reverted 2026-07-18 by pilot
    // operational decision, same day as the brief default-1.5 promotion —
    // back to the pre-DOB vertical behavior; opt in per craft with
    // `param set altitude.dob.fc 1.5`, in-flight over WiFi works too).
    // 既定0=無効（2026-07-18 パイロット運用判断で同日中の既定1.5昇格を取り消し、
    // DOB導入前の鉛直挙動へ復帰。機体ごとに `param set altitude.dob.fc 1.5` で
    // opt-in、飛行中のWiFi設定も可）。
    // Design: 2026-07-18 sim study (flight-log-driven closed-loop replay,
    // analysis/scripts/alt_dob_design/README.md §5) — clean-hover altitude
    // std -37..-56% predicted; cost is a 0.5-5Hz thrust-command RMS increase
    // (~8% of hover thrust, audible as motor-tone modulation; the flight
    // path itself is SMOOTHER than without DOB). Range [0.2, 5.0] when
    // enabled is enforced by PidController::loadParams() (WARN + clamp).
    // FLIGHT-VALIDATED 2026-07-18 (log 022929, hands-off POS_HOLD, fc=1.5):
    // alt std 55.2 mm under the same aircon disturbance the no-DOB baseline
    // held at 167.2 mm (-67%, beats the prediction); d_hat clamp saturation
    // 0%; fc/clamp detune sweep found no better point. Promoted to the
    // compiled default 2026-07-18 (pilot decision; precedent: roll retune /
    // yaw kappa defaults after single-craft A/B): SILS passes ALL flight
    // gates with the DOB enabled, and the inner-loop margins (thrust gain
    // +/-30%, mass +/-10%, delay +50 ms) cover craft-to-craft variation.
    // Venue-class environments not yet flight-validated — if misbehavior is
    // seen there (0.5-3 Hz thrust oscillation, alt excursions), set 0.
    // 高度速度ループ(Airborne)用の加速度ベース外乱オブザーバ(DOB)カットオフ。
    // 既定1.5Hz=有効（0で無効。飛行中でもWiFi経由 `param set altitude.dob.fc 0`
    // で着陸不要の無効化可）。設計: 2026-07-18シム設計スタディ（フライトログ駆動
    // 閉ループ再生、analysis/scripts/alt_dob_design/README.md §5）— 清浄ホバーで
    // 高度std -37〜-56%予測、代償は0.5-5Hz帯の推力指令RMS増加（ホバー推力の約8%、
    // モータ音の変調として聞こえる。飛行経路自体はDOBなしより滑らか）。有効時の
    // 範囲[0.2,5.0]はPidController::loadParams()が強制（範囲外WARN+クランプ）。
    // 実飛行検証 2026-07-18（ログ022929、手放しPOS_HOLD、fc=1.5）: 同一エアコン
    // 外乱下で alt std 55.2mm（DOBなし基準167.2mm、−67%＝予測超え）、d̂クランプ
    // 飽和0%、fc/クランプ掃引に現行超えなし。同日コンパイル既定へ昇格（パイロット
    // 判断。前例: ロール再調整・ヨーκ修正も単機A/B後に既定値化）: SILSはDOB有効で
    // 全飛行ゲートPASS、内部ループ余裕（推力ゲイン±30%・質量±10%・遅れ+50ms）が
    // 個体差をカバー。会場級環境は実飛行未検証 — 異常（0.5-3Hz推力振動・高度逸脱）
    // が出たら0にすること。
    float alt_dob_fc = 0.0f;

    // Hover thrust correction (HOVER_THRUST_CORRECTION): hover_thrust = mg × corr.
    // The idealized motor curve over-promises thrust, so worn hardware needs corr
    // ≈ 1.12 (flight-measured) to actually hover. FRESH/stronger motors produce
    // MORE thrust per duty → corr must DROP (else auto-takeoff over-climbs and the
    // rate loop runs hot). Tune from a hover log: corr_new = 1.12 × (duty_new/duty_old).
    // ホバー推力補正: hover_thrust = mg × corr。理想モータ曲線は推力を過大評価するため、
    // 摩耗ハードは corr≈1.12（飛行実測）でホバー。新品/強いモータは同 duty で推力が大きい
    // → corr を下げる（さもないと自動離陸が過上昇しレートループが過敏化）。ホバーログから
    // corr_new = 1.12 ×（新duty/旧duty）で調整。
    float hover_thrust_corr = 1.00f;

    // Onboard hover-thrust learning enable (1 = learn the true hover thrust in flight and
    // persist into hover.thrust_corr at touchdown, 0 = manual hover.thrust_corr only). Makes
    // altitude hold robust to thrust degradation (motor wear, battery sag) without per-flight
    // corr tuning. See pid_controller learnHoverThrust().
    // オンボード・ホバー推力学習の有効化（1 = 飛行中に真のホバー推力を学習し着地時 hover.thrust_corr
    // へ永続, 0 = 手動 corr のみ）。推力劣化（モータ劣化・電圧サグ）に高度保持をロバスト化し、corr の
    // フライト毎手調整を不要にする。learnHoverThrust() 参照。
    int32_t hover_thrust_learn = 1;

    // Position control. Runtime defaults (these initializers are the boot value
    // when NVS has no saved entry; keep them EQUAL to the table[] default below).
    // Re-tuned from the first real POS_HOLD flight (2026-06-22): the loop-relevant
    // tilt->measured-velocity gain on hardware is only ~0.4 g, which collapses the
    // inner velocity loop below the outer position loop and the closed loop slowly
    // diverges into the wall. vel.kp 0.8->3.0 restores the inner velocity loop's
    // authority (it had collapsed below the outer loop); pos.kp 1.0->0.4 slows the
    // outer loop -> cascade separation restored. Robust over K in [2.8,7] / tau in
    // [50,300] ms; SILS pos_* still pass. Tuned over TWO real flights: 0.3/2.0 first
    // stopped the divergence (held ~13 cm), then 0.4/3.0 tightened it (steady-hold
    // drift RMS 31->16 mm, max 126->83 mm) with no extra tilt buzz. The residual
    // wander is set by the ~0.4 g effective tilt->velocity gain (root cause, separate
    // task: attitude-loop tilt achievement / flow scale). See poshold_loop_design.py.
    // 位置制御。実行時の既定（NVS に保存がなければこの初期化子が起動値。下の table[] 既定と
    // 必ず一致させる）。初の実機 POS_HOLD 飛行（2026-06-22）から再調整: 実機の実効
    // 「傾き→速度」ゲインが約 0.4 g しかなく内/外ループの分離が崩れ閉ループが緩やかに発散
    // して壁へ。vel.kp 0.8→3.0 で内側(速度)ループの権限を回復、pos.kp 1.0→0.4 で外ループを
    // 遅く → カスケード分離を回復。K∈[2.8,7]/τ∈[50,300]ms でロバスト、SILS pos_* 全 PASS。
    // 実機2飛行で調整: 0.3/2.0 でまず発散を止め（~13cm 保持）、0.4/3.0 で締めた（定常保持の
    // ドリフト RMS 31→16mm・最大 126→83mm、傾きのビビり増なし）。残る揺らぎは ~0.4 g の
    // 実効ゲインが律速（根治は別タスク: 姿勢ループの傾き達成度／フロー速度スケール）。
    float pos_pos_kp      = 0.4f;
    float pos_pos_ti      = 5.0f;
    float pos_vel_kp      = 3.0f;
    float pos_vel_ti      = 2.0f;
    // POS_HOLD stick reposition speed [m/s]: deflecting roll/pitch in POS_HOLD drives the
    // craft at up to this speed (deflect to move, release to hold); centre = hold. Gentle default
    // for an indoor room; tune live with `param set position.stick_vel`.
    // POS_HOLD スティック再配置速度 [m/s]: POS_HOLD で roll/pitch を倒すとこの速度まで機体が
    // 動く（倒して動かし、離して保持）、中立=保持。屋内向けに穏やかな既定値。
    float pos_stick_vel   = 0.4f;

    // ESKF process noise
    float eskf_gyro_noise   = 0.009655f;
    float eskf_accel_noise  = 0.3f;
    float eskf_gyro_bias    = 0.000013f;
    float eskf_accel_bias   = 0.0001f;

    // Gyro-bias deviation limit around the boot-calibration nominal [rad/s]
    // (PX4-style bias limiting — bounds how far any sensor anomaly can drag the
    // bias that feeds the rate loop; see EskfConfig::bg_deviation_max).
    // 起動校正ノミナルまわりのジャイロバイアス偏差上限 [rad/s]（PX4 流バイアス制限 —
    // センサ異常がレートループ用バイアスを引きずれる距離を有界化。
    // EskfConfig::bg_deviation_max 参照）。
    float eskf_bg_dev_max   = 0.03f;

    // ESKF observation noise
    // tof_noise lowered 0.03→0.01: the flight-log offline replay showed the ToF innovation
    // NIS ≪ 1 at 0.03 (over-conservative — ToF tracks within <1 cm), so 0.01 trusts ToF more
    // for tighter vertical tracking (altlog REPORT §4).
    // tof_noise を 0.03→0.01: 実機ログ再生で ToF イノベ NIS≪1（0.03 は保守的すぎ、ToF は
    // <1cm で追従）→ 0.01 で ToF を信用し鉛直追従を締める。
    float eskf_tof_noise      = 0.01f;
    float eskf_flow_noise     = 0.30f;
    float eskf_baro_noise     = 0.1f;
    float eskf_mag_noise      = 1.0f;
    // Accel-attitude observation noise σ [m/s²]. History: 0.06→0.8 cured the χ² latch-up
    // (chi2_latchup_finding). Then 0.8→1.2 from the flight-log offline replay: at 0.8 the
    // χ² REJECTION on real data is ~10 % (over-rejecting the x-axis 11.9 Hz airframe
    // vibration), it hits the ideal ~5 % at 1.2, and over-rejects-the-other-way (1.3 %, more
    // accel-bias drift) at 2.0. 1.2 is the data-optimum; pair it with eskf_accel_att_lpf
    // (the SILS n2 vibration is isotropic and could not show this — the real x-axis mode is
    // the driver; see altlog REPORT §3–4).
    // 0.06→0.8 で χ² ラッチアップ解消、さらに 0.8→1.2 を実機ログ再生で確定: 0.8 は実データで
    // χ² 棄却 ~10%（x軸 11.9Hz 機体振動を過剰棄却）、1.2 で理想 ~5%、2.0 で過小棄却＋バイアス
    // ドリフト増。1.2 がデータ最適。eskf_accel_att_lpf と併用（SILS の n2 振動は等方的で実機の
    // x軸モードを欠くため SILS では出ない）。
    float eskf_accel_att      = 1.2f;
    // Accel-attitude LPF cutoff [Hz] (0 = off). 30 Hz cleans the airframe vibration from the
    // gravity reference; the offline sweep cut accel-bias drift 0.28→0.18 (12 Hz notch did
    // NOT help — broadband). Applied to the attitude update only, NOT predict.
    // accel 姿勢 LPF カットオフ[Hz]（0=無効）。30Hz で重力基準から機体振動を清浄化、掃引で
    // バイアスドリフト 0.28→0.18（12Hz ノッチは広帯域ゆえ無効）。姿勢更新のみ、predict には不適用。
    float eskf_accel_att_lpf  = 30.0f;

    // ESKF sensor enable
    bool eskf_use_tof   = true;
    bool eskf_use_flow  = true;
    bool eskf_use_baro  = false;
    bool eskf_use_mag   = false;

    // ESKF gates
    float eskf_mahalanobis  = 15.0f;
    float eskf_tof_innov    = 0.5f;
    float eskf_baro_innov   = 0.5f;
    float eskf_flow_clamp   = 0.3f;
    int32_t eskf_flow_squal = 10;   // min PMW3901 SQUAL to fuse flow (L-1)

    // ESKF accel-attitude (proven firmware/vehicle values). Registered here as the
    // single source of truth so they are NOT silently taken from the struct defaults.
    // 加速度-姿勢（実証済み firmware/vehicle 値）。SSOT として登録し struct 既定値に
    // 暗黙依存しないようにする。
    float eskf_att_k_adaptive = 10.0f;   // adaptive R: R *= (1 + k|a-g|²)
    float eskf_att_chi2_gate  = 7.81f;   // χ²(3, 0.95) accel-attitude outlier gate
    float eskf_att_corr_clamp = 0.05f;   // [rad] per-update roll/pitch correction clamp

    // ESKF acceleration-compensated accel-attitude (POS_HOLD). The accelerometer measures
    // specific force f = a_kin − g; during a horizontal maneuver the kinematic term a_kin
    // is mistaken for a tilt and the attitude sticks at the "apparent gravity" angle
    // atan(a/g), so POS_HOLD flies away. An α-β tracker on the flow velocity estimates
    // a_kin (state = velocity + acceleration; β small so the SUSTAINED drift acceleration
    // is captured, not washed out like a naive derivative), and the accel-attitude update
    // subtracts R^T·a_kin → the residual is the TRUE attitude error.
    // ESKF 運動加速度補償の accel-attitude（POS_HOLD）。加速度計は比力 f=a_kin−g を測り、水平
    // マニューバ中は運動加速度 a_kin を傾きと誤認し姿勢が「見かけの重力」角 atan(a/g) に張付き
    // POS_HOLD が飛び去る。フロー速度の α-β トラッカで a_kin を推定（状態=速度+加速度、β 小で
    // 持続ドリフト加速度を単純微分のように washout せず捕捉）、accel-attitude が R^T·a_kin を
    // 差し引き残差を真の姿勢誤差にする。
    bool  eskf_accel_comp_enable = true;  // on (adopted; SILS clean+N0 all 4 axes hold)
    float eskf_accel_comp_alpha  = 0.2f;  // α-β velocity gain
    float eskf_accel_comp_beta   = 0.02f; // α-β acceleration gain (small = capture DC drift)
    float eskf_accel_comp_max    = 5.0f;  // [m/s²] physical clamp on a_kin

    // Safety
    float safety_accel_g     = 3.0f;
    float safety_gyro_dps    = 800.0f;
    float safety_comm_timeout = 500.0f;
    float safety_low_v       = 3.4f;
    float safety_usb_v       = 3.3f;

    // Calibration — boot gyro/accel bias calibration on/off (ImuTask seeds the
    // estimator at rest before flight). Default on.
    // キャリブレーション — 起動時バイアス校正の ON/OFF（ImuTask が飛行前に静止で推定器へ
    // 種付け）。既定 ON。
    bool calibration_enable = true;
}

namespace params {

using namespace param_vars;

// -----------------------------------------------------------------------------
// Live-reload callbacks — set on the table rows below. A param set publishes a
// ReloadParams verb on the owning task's command topic; the OWNER re-reads its
// parameters in its own context (thread-safe immediate application; the
// callback itself never touches another task's objects).
// ライブ再読込コールバック — 下のテーブル行に設定。param set が所有タスクの
// コマンドトピックへ ReloadParams verb を発行し、「所有者」が自分の文脈で
// パラメータを読み直す（スレッド安全な即時反映。コールバック自身は他タスクの
// オブジェクトに決して触らない）。
//
// @design detailed_design.md §5 — parameter change → immediate apply     [OK]
// -----------------------------------------------------------------------------

static void notifyControllerReload()
{
    controller_command.publish(
        {static_cast<uint8_t>(ControllerCmd::ReloadParams), 0,
         static_cast<uint32_t>(esp_timer_get_time())});
}

static void notifyEstimatorReload()
{
    estimator_command.publish(
        {static_cast<uint8_t>(EstimatorCmd::ReloadParams),
         static_cast<uint32_t>(esp_timer_get_time()), 0});
}

/// Parameter table — the single source of truth (SSOT). Each row binds a name to
/// a param_vars variable with its default/min/max/callback. To add a parameter,
/// add a variable to param_vars (above) and a row here.
/// パラメータテーブル — 唯一の真実源 (SSOT)。各行が名前を param_vars 変数に
/// 既定/最小/最大/コールバック付きで結ぶ。追加は param_vars に変数を、ここに行を。
static const ParamEntry table[] = {
    // Rate control — PHYSICAL gains [Nm/(rad/s)] for the B^-1 mixer (actuator.cpp).
    // kp = I/τ_resp (τ_resp=0.05s); ti large = near-P inner loop. See the variable
    // declarations above for the rationale. Max 0.01 = ~25× headroom over kp.
    // レート制御 — B^-1 ミキサー用の物理ゲイン [Nm/(rad/s)]。kp = 慣性/τ_resp。
    {"rate.roll.kp",    ParamType::FLOAT, &rate_roll_kp,   1.0e-3f,   0.0f,  0.01f,  &notifyControllerReload},
    {"rate.roll.ti",    ParamType::FLOAT, &rate_roll_ti,   0.7f,      0.01f, 100.0f, &notifyControllerReload},
    {"rate.roll.td",    ParamType::FLOAT, &rate_roll_td,   0.002f,    0.0f,  1.0f,   &notifyControllerReload},
    {"rate.pitch.kp",   ParamType::FLOAT, &rate_pitch_kp,  1.426432e-3f, 0.0f, 0.01f,  &notifyControllerReload},
    {"rate.pitch.ti",   ParamType::FLOAT, &rate_pitch_ti,  0.7f,      0.01f, 100.0f, &notifyControllerReload},
    {"rate.pitch.td",   ParamType::FLOAT, &rate_pitch_td,  0.025f,    0.0f,  1.0f,   &notifyControllerReload},
    {"rate.yaw.kp",     ParamType::FLOAT, &rate_yaw_kp,    8.029796e-4f, 0.0f, 0.01f,  &notifyControllerReload},
    {"rate.yaw.ti",     ParamType::FLOAT, &rate_yaw_ti,    0.8f,      0.01f, 100.0f, &notifyControllerReload},
    {"rate.yaw.td",     ParamType::FLOAT, &rate_yaw_td,    0.01f,     0.0f,  1.0f,   &notifyControllerReload},
    // Yaw torque cap — see the param_vars comment (NT-Kanazawa saturation treatment).
    // ヨートルク上限 — param_vars のコメント参照（NT金沢飽和の治療）。
    {"rate.yaw.max_torque", ParamType::FLOAT, &rate_yaw_max_torque, 1.226e-3f, 1.0e-4f, 1.41e-3f, &notifyControllerReload},
    // Sliding-mode rate-loop gains (firmware/apps/smc_rate) -- see the
    // param_vars comment above for the seed derivation. Unused by the
    // default vehicle build; wired to notifyControllerReload the same as
    // the PID rows above so `smc_rate`'s AppController::reloadParams()
    // picks up live edits (sf params set / --param sweeps / NVS).
    // スライディングモード・レートループのゲイン（firmware/apps/smc_rate）
    // -- 初期値の導出は上のparam_varsコメント参照。既定vehicleビルドでは
    // 未使用。上のPID行と同じくnotifyControllerReloadに配線し、
    // `smc_rate`のAppController::reloadParams()がライブ編集
    // （sf params set / --paramスイープ / NVS）を反映できるようにする。
    {"smc.roll.k",    ParamType::FLOAT, &smc_roll_k,    40.0f,  0.0f, 60.0f, &notifyControllerReload},
    {"smc.roll.eta",  ParamType::FLOAT, &smc_roll_eta,  180.0f, 0.0f, 500.0f, &notifyControllerReload},
    {"smc.roll.phi",  ParamType::FLOAT, &smc_roll_phi,  0.15f,  0.01f, 2.0f, &notifyControllerReload},
    {"smc.pitch.k",   ParamType::FLOAT, &smc_pitch_k,   40.0f,  0.0f, 60.0f, &notifyControllerReload},
    {"smc.pitch.eta", ParamType::FLOAT, &smc_pitch_eta, 175.0f, 0.0f, 500.0f, &notifyControllerReload},
    {"smc.pitch.phi", ParamType::FLOAT, &smc_pitch_phi, 0.15f,  0.01f, 2.0f, &notifyControllerReload},
    {"smc.yaw.k",     ParamType::FLOAT, &smc_yaw_k,     7.3f,   0.0f, 15.0f, &notifyControllerReload},
    {"smc.yaw.eta",   ParamType::FLOAT, &smc_yaw_eta,   54.0f,  0.0f, 100.0f, &notifyControllerReload},
    {"smc.yaw.phi",   ParamType::FLOAT, &smc_yaw_phi,   0.3f,   0.01f, 2.0f, &notifyControllerReload},
    // PI-type sliding surface integral gain -- see the param_vars comment
    // above for the seed derivation and SS3.4's status (not yet validated).
    // PI型スライディング面の積分ゲイン -- 初期値の導出と§3.4のステータス
    // （未検証）は上のparam_varsコメント参照。
    {"smc.roll.lambda_i",  ParamType::FLOAT, &smc_roll_lambda_i,  6.0f,  0.0f, 10.0f, &notifyControllerReload},
    {"smc.pitch.lambda_i", ParamType::FLOAT, &smc_pitch_lambda_i, 6.0f,  0.0f, 10.0f, &notifyControllerReload},
    {"smc.yaw.lambda_i",   ParamType::FLOAT, &smc_yaw_lambda_i,   1.25f, 0.0f, 10.0f, &notifyControllerReload},
    // Large-error integral reset threshold (anti-windup layer 3) -- see the
    // param_vars comment above for the seed derivation (5*phi).
    // 偏差ゲートによる積分リセット閾値（アンチワインドアップ第3層）--
    // 初期値の導出（5*phi）は上のparam_varsコメント参照。
    {"smc.roll.e_reset",  ParamType::FLOAT, &smc_roll_e_reset,  0.75f, 0.0f, 5.0f, &notifyControllerReload},
    {"smc.pitch.e_reset", ParamType::FLOAT, &smc_pitch_e_reset, 0.75f, 0.0f, 5.0f, &notifyControllerReload},
    {"smc.yaw.e_reset",   ParamType::FLOAT, &smc_yaw_e_reset,   1.5f,  0.0f, 5.0f, &notifyControllerReload},
    // Dead-time predictor -- see the param_vars comment above.
    // 無駄時間予測補償器 -- 初期値の導出は上のparam_varsコメント参照。
    {"smc.roll.delay_comp_ms",  ParamType::FLOAT, &smc_roll_delay_comp_ms,  0.0f, 0.0f, 40.0f, &notifyControllerReload},
    {"smc.pitch.delay_comp_ms", ParamType::FLOAT, &smc_pitch_delay_comp_ms, 0.0f, 0.0f, 40.0f, &notifyControllerReload},
    {"smc.yaw.delay_comp_ms",   ParamType::FLOAT, &smc_yaw_delay_comp_ms,   0.0f, 0.0f, 40.0f, &notifyControllerReload},
    // Sliding-surface dead-band -- see the param_vars comment above.
    // スライディング面の不感バンド -- 上のparam_varsコメント参照。
    {"smc.roll.s_deadband",  ParamType::FLOAT, &smc_roll_s_deadband,  0.0f, 0.0f, 5.0f, &notifyControllerReload},
    {"smc.pitch.s_deadband", ParamType::FLOAT, &smc_pitch_s_deadband, 0.0f, 0.0f, 5.0f, &notifyControllerReload},
    {"smc.yaw.s_deadband",   ParamType::FLOAT, &smc_yaw_s_deadband,   0.0f, 0.0f, 5.0f, &notifyControllerReload},
    // Super-twisting rate-loop gains (firmware/apps/smc_rate_sta) -- see
    // the param_vars comment above for the seed derivation. Unused by
    // vehicle/smc_rate/smc_pos.
    // スーパーツイスティング・レートループのゲイン（firmware/apps/
    // smc_rate_sta）-- 初期値の導出は上のparam_varsコメント参照。
    // vehicle/smc_rate/smc_posでは未使用。
    {"smc_sta.roll.k1",        ParamType::FLOAT, &smc_sta_roll_k1,        60.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_sta.roll.k2",        ParamType::FLOAT, &smc_sta_roll_k2,        30.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_sta.roll.phi",       ParamType::FLOAT, &smc_sta_roll_phi,       0.02f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_sta.roll.lambda_i",  ParamType::FLOAT, &smc_sta_roll_lambda_i,  6.0f,   0.0f, 10.0f,   &notifyControllerReload},
    {"smc_sta.roll.e_reset",   ParamType::FLOAT, &smc_sta_roll_e_reset,   0.75f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_sta.pitch.k1",       ParamType::FLOAT, &smc_sta_pitch_k1,       60.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_sta.pitch.k2",       ParamType::FLOAT, &smc_sta_pitch_k2,       30.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_sta.pitch.phi",      ParamType::FLOAT, &smc_sta_pitch_phi,      0.02f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_sta.pitch.lambda_i", ParamType::FLOAT, &smc_sta_pitch_lambda_i, 6.0f,   0.0f, 10.0f,   &notifyControllerReload},
    {"smc_sta.pitch.e_reset",  ParamType::FLOAT, &smc_sta_pitch_e_reset,  0.75f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_sta.yaw.k1",         ParamType::FLOAT, &smc_sta_yaw_k1,         15.2f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_sta.yaw.k2",         ParamType::FLOAT, &smc_sta_yaw_k2,         7.4f,   0.0f, 1000.0f, &notifyControllerReload},
    {"smc_sta.yaw.phi",        ParamType::FLOAT, &smc_sta_yaw_phi,        0.04f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_sta.yaw.lambda_i",   ParamType::FLOAT, &smc_sta_yaw_lambda_i,   1.25f,  0.0f, 10.0f,   &notifyControllerReload},
    {"smc_sta.yaw.e_reset",    ParamType::FLOAT, &smc_sta_yaw_e_reset,    1.5f,   0.0f, 5.0f,    &notifyControllerReload},
    // z leaky-integration time constant -- see the param_vars comment above.
    // zの漏れ積分時定数 -- 上のparam_varsコメント参照。
    {"smc_sta.roll.z_leak_tau",  ParamType::FLOAT, &smc_sta_roll_z_leak_tau,  0.5f, 0.0f, 10.0f, &notifyControllerReload},
    {"smc_sta.pitch.z_leak_tau", ParamType::FLOAT, &smc_sta_pitch_z_leak_tau, 0.5f, 0.0f, 10.0f, &notifyControllerReload},
    {"smc_sta.yaw.z_leak_tau",   ParamType::FLOAT, &smc_sta_yaw_z_leak_tau,   1.0f, 0.0f, 10.0f, &notifyControllerReload},
    // Adaptive-gain super-twisting rate-loop gains (firmware/apps/
    // smc_rate_asta) -- see the param_vars comment above for the seed
    // derivation and motivation. smc_rate_sta itself is unchanged; this is
    // a separate, independent app. Unused by vehicle/smc_rate/smc_rate_sta/
    // smc_pos/smc_pos_sta.
    // 適応ゲイン・スーパーツイスティング・レートループのゲイン
    // （firmware/apps/smc_rate_asta）-- シード値の導出・動機は上の
    // param_varsコメント参照。smc_rate_sta自体は無変更、これは別・独立の
    // app。vehicle/smc_rate/smc_rate_sta/smc_pos/smc_pos_staでは未使用。
    {"smc_asta.roll.k1_init",    ParamType::FLOAT, &smc_asta_roll_k1_init,    60.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.roll.k1_min",     ParamType::FLOAT, &smc_asta_roll_k1_min,     30.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.roll.k1_max",     ParamType::FLOAT, &smc_asta_roll_k1_max,     150.0f, 0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.roll.k2_ratio",   ParamType::FLOAT, &smc_asta_roll_k2_ratio,   0.5f,   0.0f, 2.0f,    &notifyControllerReload},
    {"smc_asta.roll.adapt_rate", ParamType::FLOAT, &smc_asta_roll_adapt_rate, 20.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.roll.leak_ratio", ParamType::FLOAT, &smc_asta_roll_leak_ratio, 0.2f,   0.0f, 2.0f,    &notifyControllerReload},
    {"smc_asta.roll.dead_band",  ParamType::FLOAT, &smc_asta_roll_dead_band,  0.03f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_asta.roll.filter_tau", ParamType::FLOAT, &smc_asta_roll_filter_tau, 0.3f,   0.0f, 2.0f,    &notifyControllerReload},
    {"smc_asta.roll.phi",        ParamType::FLOAT, &smc_asta_roll_phi,        0.02f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_asta.roll.lambda_i",   ParamType::FLOAT, &smc_asta_roll_lambda_i,   6.0f,   0.0f, 10.0f,   &notifyControllerReload},
    {"smc_asta.roll.e_reset",    ParamType::FLOAT, &smc_asta_roll_e_reset,    0.75f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_asta.roll.z_leak_tau", ParamType::FLOAT, &smc_asta_roll_z_leak_tau, 0.5f,   0.0f, 10.0f,   &notifyControllerReload},
    {"smc_asta.pitch.k1_init",    ParamType::FLOAT, &smc_asta_pitch_k1_init,    60.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.pitch.k1_min",     ParamType::FLOAT, &smc_asta_pitch_k1_min,     30.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.pitch.k1_max",     ParamType::FLOAT, &smc_asta_pitch_k1_max,     150.0f, 0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.pitch.k2_ratio",   ParamType::FLOAT, &smc_asta_pitch_k2_ratio,   0.5f,   0.0f, 2.0f,    &notifyControllerReload},
    {"smc_asta.pitch.adapt_rate", ParamType::FLOAT, &smc_asta_pitch_adapt_rate, 20.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.pitch.leak_ratio", ParamType::FLOAT, &smc_asta_pitch_leak_ratio, 0.2f,   0.0f, 2.0f,    &notifyControllerReload},
    {"smc_asta.pitch.dead_band",  ParamType::FLOAT, &smc_asta_pitch_dead_band,  0.03f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_asta.pitch.filter_tau", ParamType::FLOAT, &smc_asta_pitch_filter_tau, 0.3f,   0.0f, 2.0f,    &notifyControllerReload},
    {"smc_asta.pitch.phi",        ParamType::FLOAT, &smc_asta_pitch_phi,        0.02f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_asta.pitch.lambda_i",   ParamType::FLOAT, &smc_asta_pitch_lambda_i,   6.0f,   0.0f, 10.0f,   &notifyControllerReload},
    {"smc_asta.pitch.e_reset",    ParamType::FLOAT, &smc_asta_pitch_e_reset,    0.75f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_asta.pitch.z_leak_tau", ParamType::FLOAT, &smc_asta_pitch_z_leak_tau, 0.5f,   0.0f, 10.0f,   &notifyControllerReload},
    {"smc_asta.yaw.k1_init",    ParamType::FLOAT, &smc_asta_yaw_k1_init,    15.2f, 0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.yaw.k1_min",     ParamType::FLOAT, &smc_asta_yaw_k1_min,     7.6f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.yaw.k1_max",     ParamType::FLOAT, &smc_asta_yaw_k1_max,     38.0f, 0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.yaw.k2_ratio",   ParamType::FLOAT, &smc_asta_yaw_k2_ratio,   0.5f,  0.0f, 2.0f,    &notifyControllerReload},
    {"smc_asta.yaw.adapt_rate", ParamType::FLOAT, &smc_asta_yaw_adapt_rate, 5.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_asta.yaw.leak_ratio", ParamType::FLOAT, &smc_asta_yaw_leak_ratio, 0.2f,  0.0f, 2.0f,    &notifyControllerReload},
    {"smc_asta.yaw.dead_band",  ParamType::FLOAT, &smc_asta_yaw_dead_band,  0.05f, 0.0f, 5.0f,    &notifyControllerReload},
    {"smc_asta.yaw.filter_tau", ParamType::FLOAT, &smc_asta_yaw_filter_tau, 0.05f, 0.0f, 2.0f,    &notifyControllerReload},
    {"smc_asta.yaw.phi",        ParamType::FLOAT, &smc_asta_yaw_phi,        0.04f, 0.001f, 2.0f,  &notifyControllerReload},
    {"smc_asta.yaw.lambda_i",   ParamType::FLOAT, &smc_asta_yaw_lambda_i,   1.25f, 0.0f, 10.0f,   &notifyControllerReload},
    {"smc_asta.yaw.e_reset",    ParamType::FLOAT, &smc_asta_yaw_e_reset,    1.5f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_asta.yaw.z_leak_tau", ParamType::FLOAT, &smc_asta_yaw_z_leak_tau, 1.0f,  0.0f, 10.0f,   &notifyControllerReload},
    // smc_pos_sta gains (firmware/apps/smc_pos_sta) -- see the param_vars
    // comment above for the seed derivation. Unused by the default vehicle/
    // smc_rate/smc_rate_sta/smc_pos builds.
    // smc_pos_staのゲイン（firmware/apps/smc_pos_sta）-- シード値の導出は
    // 上のparam_varsコメント参照。既定vehicle/smc_rate/smc_rate_sta/smc_pos
    // ビルドでは未使用。
    {"smc_pos_sta.roll.k1",        ParamType::FLOAT, &smc_pos_sta_roll_k1,        60.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_pos_sta.roll.k2",        ParamType::FLOAT, &smc_pos_sta_roll_k2,        30.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_pos_sta.roll.phi",       ParamType::FLOAT, &smc_pos_sta_roll_phi,       0.02f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_pos_sta.roll.lambda_i",  ParamType::FLOAT, &smc_pos_sta_roll_lambda_i,  6.0f,   0.0f, 10.0f,   &notifyControllerReload},
    {"smc_pos_sta.roll.e_reset",   ParamType::FLOAT, &smc_pos_sta_roll_e_reset,   0.75f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.roll.z_leak_tau", ParamType::FLOAT, &smc_pos_sta_roll_z_leak_tau, 0.5f, 0.0f, 10.0f,   &notifyControllerReload},
    {"smc_pos_sta.pitch.k1",       ParamType::FLOAT, &smc_pos_sta_pitch_k1,       60.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_pos_sta.pitch.k2",       ParamType::FLOAT, &smc_pos_sta_pitch_k2,       30.0f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_pos_sta.pitch.phi",      ParamType::FLOAT, &smc_pos_sta_pitch_phi,      0.02f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_pos_sta.pitch.lambda_i", ParamType::FLOAT, &smc_pos_sta_pitch_lambda_i, 6.0f,   0.0f, 10.0f,   &notifyControllerReload},
    {"smc_pos_sta.pitch.e_reset",  ParamType::FLOAT, &smc_pos_sta_pitch_e_reset,  0.75f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.pitch.z_leak_tau", ParamType::FLOAT, &smc_pos_sta_pitch_z_leak_tau, 0.5f, 0.0f, 10.0f, &notifyControllerReload},
    {"smc_pos_sta.yaw.k1",         ParamType::FLOAT, &smc_pos_sta_yaw_k1,         15.2f,  0.0f, 1000.0f, &notifyControllerReload},
    {"smc_pos_sta.yaw.k2",         ParamType::FLOAT, &smc_pos_sta_yaw_k2,         7.4f,   0.0f, 1000.0f, &notifyControllerReload},
    {"smc_pos_sta.yaw.phi",        ParamType::FLOAT, &smc_pos_sta_yaw_phi,        0.04f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_pos_sta.yaw.lambda_i",   ParamType::FLOAT, &smc_pos_sta_yaw_lambda_i,   1.25f,  0.0f, 10.0f,   &notifyControllerReload},
    {"smc_pos_sta.yaw.e_reset",    ParamType::FLOAT, &smc_pos_sta_yaw_e_reset,    1.5f,   0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.yaw.z_leak_tau", ParamType::FLOAT, &smc_pos_sta_yaw_z_leak_tau, 0.5f,   0.0f, 10.0f,   &notifyControllerReload},
    {"smc_pos_sta.velx.k1",        ParamType::FLOAT, &smc_pos_sta_velx_k1,        0.6f,   0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.velx.k2",        ParamType::FLOAT, &smc_pos_sta_velx_k2,        0.3f,   0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.velx.phi",       ParamType::FLOAT, &smc_pos_sta_velx_phi,       0.03f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_pos_sta.velx.lambda_i",  ParamType::FLOAT, &smc_pos_sta_velx_lambda_i,  0.5f,   0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.velx.e_reset",   ParamType::FLOAT, &smc_pos_sta_velx_e_reset,   1.25f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.velx.z_leak_tau", ParamType::FLOAT, &smc_pos_sta_velx_z_leak_tau, 0.5f, 0.0f, 10.0f,   &notifyControllerReload},
    {"smc_pos_sta.vely.k1",        ParamType::FLOAT, &smc_pos_sta_vely_k1,        0.6f,   0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.vely.k2",        ParamType::FLOAT, &smc_pos_sta_vely_k2,        0.3f,   0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.vely.phi",       ParamType::FLOAT, &smc_pos_sta_vely_phi,       0.03f,  0.001f, 2.0f,  &notifyControllerReload},
    {"smc_pos_sta.vely.lambda_i",  ParamType::FLOAT, &smc_pos_sta_vely_lambda_i,  0.5f,   0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.vely.e_reset",   ParamType::FLOAT, &smc_pos_sta_vely_e_reset,   1.25f,  0.0f, 5.0f,    &notifyControllerReload},
    {"smc_pos_sta.vely.z_leak_tau", ParamType::FLOAT, &smc_pos_sta_vely_z_leak_tau, 0.5f, 0.0f, 10.0f,   &notifyControllerReload},
    // Sliding-mode horizontal-velocity-loop gains (firmware/apps/smc_pos) --
    // see the param_vars comment above for the seed derivation. Unused by
    // the default vehicle/smc_rate builds.
    // スライディングモード・水平速度ループのゲイン（firmware/apps/smc_pos）
    // -- 初期値の導出は上のparam_varsコメント参照。既定vehicle/smc_rate
    // ビルドでは未使用。
    {"smc.velx.k",        ParamType::FLOAT, &smc_velx_k,        0.2f,  0.0f, 5.0f,  &notifyControllerReload},
    {"smc.velx.eta",      ParamType::FLOAT, &smc_velx_eta,      1.0f,  0.0f, 20.0f, &notifyControllerReload},
    {"smc.velx.phi",      ParamType::FLOAT, &smc_velx_phi,      0.25f, 0.01f, 2.0f, &notifyControllerReload},
    {"smc.velx.lambda_i", ParamType::FLOAT, &smc_velx_lambda_i, 0.5f,  0.0f, 5.0f,  &notifyControllerReload},
    {"smc.velx.e_reset",  ParamType::FLOAT, &smc_velx_e_reset,  1.25f, 0.0f, 5.0f,  &notifyControllerReload},
    {"smc.vely.k",        ParamType::FLOAT, &smc_vely_k,        0.2f,  0.0f, 5.0f,  &notifyControllerReload},
    {"smc.vely.eta",      ParamType::FLOAT, &smc_vely_eta,      1.0f,  0.0f, 20.0f, &notifyControllerReload},
    {"smc.vely.phi",      ParamType::FLOAT, &smc_vely_phi,      0.25f, 0.01f, 2.0f, &notifyControllerReload},
    {"smc.vely.lambda_i", ParamType::FLOAT, &smc_vely_lambda_i, 0.5f,  0.0f, 5.0f,  &notifyControllerReload},
    {"smc.vely.e_reset",  ParamType::FLOAT, &smc_vely_e_reset,  1.25f, 0.0f, 5.0f,  &notifyControllerReload},
    // Dead-time predictor for the velocity loop -- see the param_vars comment above.
    // 速度ループの無駄時間予測補償器 -- 上のparam_varsコメント参照。
    {"smc.velx.delay_comp_ms", ParamType::FLOAT, &smc_velx_delay_comp_ms, 0.0f, 0.0f, 40.0f, &notifyControllerReload},
    {"smc.vely.delay_comp_ms", ParamType::FLOAT, &smc_vely_delay_comp_ms, 0.0f, 0.0f, 40.0f, &notifyControllerReload},
    {"autotune.sched.axis",  ParamType::INT,   &autotune_sched_axis,  -1.0f, -1.0f,  2.0f,   nullptr},
    {"autotune.sched.delay", ParamType::FLOAT, &autotune_sched_delay, 20.0f,  3.0f, 120.0f,  nullptr},
    // Autotune sysid results (written by autotune, read-back only). Wide ranges = result store.
    {"autotune.roll.b",      ParamType::FLOAT, &autotune_roll_b,      0.0f,  0.0f, 1.0e9f, nullptr},
    {"autotune.roll.tau",    ParamType::FLOAT, &autotune_roll_tau,    0.0f,  0.0f, 10.0f,  nullptr},
    {"autotune.roll.delay",  ParamType::FLOAT, &autotune_roll_delay,  0.0f,  0.0f, 1.0f,   nullptr},
    {"autotune.roll.resid",  ParamType::FLOAT, &autotune_roll_resid,  0.0f,  0.0f, 1.0e6f, nullptr},
    {"autotune.pitch.b",     ParamType::FLOAT, &autotune_pitch_b,     0.0f,  0.0f, 1.0e9f, nullptr},
    {"autotune.pitch.tau",   ParamType::FLOAT, &autotune_pitch_tau,   0.0f,  0.0f, 10.0f,  nullptr},
    {"autotune.pitch.delay", ParamType::FLOAT, &autotune_pitch_delay, 0.0f,  0.0f, 1.0f,   nullptr},
    {"autotune.pitch.resid", ParamType::FLOAT, &autotune_pitch_resid, 0.0f,  0.0f, 1.0e6f, nullptr},
    {"autotune.yaw.b",       ParamType::FLOAT, &autotune_yaw_b,       0.0f,  0.0f, 1.0e9f, nullptr},
    {"autotune.yaw.tau",     ParamType::FLOAT, &autotune_yaw_tau,     0.0f,  0.0f, 10.0f,  nullptr},
    {"autotune.yaw.delay",   ParamType::FLOAT, &autotune_yaw_delay,   0.0f,  0.0f, 1.0f,   nullptr},
    {"autotune.yaw.resid",   ParamType::FLOAT, &autotune_yaw_resid,   0.0f,  0.0f, 1.0e6f, nullptr},
    // Autotune design margins (written by autotune, read-back only). wc[rad/s] pm[deg] gm[dB].
    {"autotune.roll.wc",     ParamType::FLOAT, &autotune_roll_wc,     0.0f,  0.0f, 5000.0f, nullptr},
    {"autotune.roll.pm",     ParamType::FLOAT, &autotune_roll_pm,     0.0f, -360.0f, 360.0f, nullptr},
    {"autotune.roll.gm",     ParamType::FLOAT, &autotune_roll_gm,     0.0f, -200.0f, 200.0f, nullptr},
    {"autotune.pitch.wc",    ParamType::FLOAT, &autotune_pitch_wc,    0.0f,  0.0f, 5000.0f, nullptr},
    {"autotune.pitch.pm",    ParamType::FLOAT, &autotune_pitch_pm,    0.0f, -360.0f, 360.0f, nullptr},
    {"autotune.pitch.gm",    ParamType::FLOAT, &autotune_pitch_gm,    0.0f, -200.0f, 200.0f, nullptr},
    {"autotune.yaw.wc",      ParamType::FLOAT, &autotune_yaw_wc,      0.0f,  0.0f, 5000.0f, nullptr},
    {"autotune.yaw.pm",      ParamType::FLOAT, &autotune_yaw_pm,      0.0f, -360.0f, 360.0f, nullptr},
    {"autotune.yaw.gm",      ParamType::FLOAT, &autotune_yaw_gm,      0.0f, -200.0f, 200.0f, nullptr},
    {"autotune.roll.reject", ParamType::FLOAT, &autotune_roll_reject, 0.0f, 0.0f, 10.0f, nullptr},
    {"autotune.pitch.reject",ParamType::FLOAT, &autotune_pitch_reject,0.0f, 0.0f, 10.0f, nullptr},
    {"autotune.yaw.reject",  ParamType::FLOAT, &autotune_yaw_reject,  0.0f, 0.0f, 10.0f, nullptr},

    // Estimator selection (0 = ESKF, 1 = complementary) — RESET_PLAN P2.
    {"estimator.type",  ParamType::INT,   &estimator_type, 0.0f,      0.0f,  1.0f,   nullptr},

    // Telemetry WiFi mode (0 = STA, 1 = SoftAP) — boot-time, no live reload
    // (the radio cannot be re-homed mid-flight).
    // テレメトリ WiFi モード（0=STA, 1=SoftAP）— 起動時のみ。ライブ再読込なし
    // （無線は飛行中に載せ替えられない）。
    {"wifi.mode",       ParamType::INT,   &wifi_mode,      0.0f,      0.0f,  1.0f,   nullptr},
    {"wifi.channel",    ParamType::INT,   &wifi_channel,   1.0f,      1.0f,  13.0f,  nullptr},
    {"log.blackbox.enable", ParamType::INT, &log_blackbox_enable, 0.0f, 0.0f, 1.0f, nullptr},

    // Attitude control
    {"attitude.roll.kp",  ParamType::FLOAT, &att_roll_kp,  5.0f,  0.0f,  50.0f,  &notifyControllerReload},
    {"attitude.roll.ti",  ParamType::FLOAT, &att_roll_ti,  2.0f,  0.01f, 100.0f, &notifyControllerReload},
    {"attitude.roll.td",  ParamType::FLOAT, &att_roll_td,  0.04f, 0.0f,  1.0f,   &notifyControllerReload},
    {"attitude.pitch.kp", ParamType::FLOAT, &att_pitch_kp, 5.0f,  0.0f,  50.0f,  &notifyControllerReload},
    {"attitude.pitch.ti", ParamType::FLOAT, &att_pitch_ti, 2.0f,  0.01f, 100.0f, &notifyControllerReload},
    {"attitude.pitch.td", ParamType::FLOAT, &att_pitch_td, 0.04f, 0.0f,  1.0f,   &notifyControllerReload},

    // Attitude trim — equilibrium tilt [rad] added to the angle SETPOINT, all modes
    // (flight-identified by sf trim analyze). Limited to ±0.1 rad (±5.7°).
    // 姿勢トリム — 角度「目標」に加算する平衡傾き [rad]、全モード（sf trim analyze で飛行同定）。
    {"attitude.roll.trim",  ParamType::FLOAT, &trim_roll,  0.0f, -0.1f, 0.1f, &notifyControllerReload},
    {"attitude.pitch.trim", ParamType::FLOAT, &trim_pitch, 0.0f, -0.1f, 0.1f, &notifyControllerReload},
    {"attitude.trim.learn", ParamType::INT,   &trim_learn, 1.0f,  0.0f, 1.0f, &notifyControllerReload},

    // Heading hold (kp=0 disables / kp=0 で無効)
    {"attitude.yawhold.kp",       ParamType::FLOAT, &att_yawhold_kp,       3.0f, 0.0f, 10.0f, &notifyControllerReload},
    {"attitude.yawhold.rate_max", ParamType::FLOAT, &att_yawhold_rate_max, 2.0f, 0.1f, 5.0f,  &notifyControllerReload},

    // Altitude control (SILS-validated; see the variable defaults above)
    {"altitude.alt.kp",   ParamType::FLOAT, &alt_alt_kp,  0.45f, 0.0f, 10.0f,  &notifyControllerReload},
    {"altitude.alt.ti",   ParamType::FLOAT, &alt_alt_ti,  7.0f,  0.1f, 100.0f, &notifyControllerReload},
    {"altitude.vel.kp",   ParamType::FLOAT, &alt_vel_kp,  0.1f,  0.0f, 10.0f,  &notifyControllerReload},
    {"altitude.vel.ti",   ParamType::FLOAT, &alt_vel_ti,  2.5f,  0.1f, 100.0f, &notifyControllerReload},
    {"altitude.vel.ti_hover", ParamType::FLOAT, &alt_vel_ti_hover, 2.5f, 0.1f, 100.0f, &notifyControllerReload},
    {"altitude.climb_rate",   ParamType::FLOAT, &alt_climb_rate,   0.5f, 0.05f, 2.0f, &notifyControllerReload},
    {"altitude.descent_rate", ParamType::FLOAT, &alt_descent_rate, 0.5f, 0.05f, 2.0f, &notifyControllerReload},
    {"altitude.dob.fc",       ParamType::FLOAT, &alt_dob_fc,       0.0f, 0.0f, 5.0f, &notifyControllerReload},
    {"hover.thrust_corr",     ParamType::FLOAT, &hover_thrust_corr, 1.00f, 0.5f, 2.0f, &notifyControllerReload},
    {"hover.thrust.learn",    ParamType::INT,   &hover_thrust_learn, 1.0f, 0.0f, 1.0f, &notifyControllerReload},

    // Position control. Gains re-tuned from the first real POS_HOLD flight
    // (2026-06-22, log 20260622T161055): on hardware the loop-relevant
    // tilt->measured-velocity gain is only ~0.4 g (vs the g the cascade assumes),
    // because the commanded tilt is not fully achieved/measured and the optical
    // flow under-reads velocity. That collapses the inner (velocity) loop bandwidth
    // below the outer (position) loop and the closed loop slowly diverges
    // (observed: growing ~0.1 Hz oscillation, +/-0.37->0.62 m, wall strike). Fix
    // restores cascade timescale separation on the IDENTIFIED plant: raise vel.kp
    // (0.8->3.0, recovering the inner velocity loop's authority) and lower pos.kp
    // (1.0->0.4, slowing the outer loop). Robustly stable over K in [2.8,7],
    // tau in [50,300] ms; SILS pos_* still pass. Real-flight tuned over 2 flights:
    // 0.3/2.0 first stopped the divergence (~13 cm hold), 0.4/3.0 tightened it
    // (steady-hold drift RMS 31->16 mm, max 126->83 mm). KEEP EQUAL to the
    // initializers above. See analysis/scripts/poshold_loop_design.py + the doc.
    // 位置制御。初の実機 POS_HOLD 飛行（2026-06-22）から再調整: 実機の「指令傾き→実測
    // 水平速度」の実効ゲインは約 0.4 g しかなく（傾き未達＋フロー速度の過小読み）、内側
    // (速度) ループ帯域が外側 (位置) ループより下がってカスケードの時間スケール分離が崩れ、
    // 閉ループが緩やかに発散（~0.1Hz 振動が ±0.37→0.62m に成長し壁に激突）。修正は同定
    // プラント上で分離を回復: vel.kp を上げ（0.8→3.0、内側ループの権限回復）、pos.kp を下げ
    // （1.0→0.4、外ループを遅く）。K∈[2.8,7]・τ∈[50,300]ms でロバスト、SILS pos_* 全 PASS。
    // 実機2飛行で調整: 0.3/2.0 で発散停止（~13cm）、0.4/3.0 で締め（ドリフト RMS 31→16mm・
    // 最大 126→83mm）。上の初期化子と必ず一致させる。
    {"position.pos.kp",   ParamType::FLOAT, &pos_pos_kp,  0.4f,  0.0f, 10.0f,  &notifyControllerReload},
    {"position.pos.ti",   ParamType::FLOAT, &pos_pos_ti,  5.0f,  0.1f, 100.0f, &notifyControllerReload},
    {"position.vel.kp",   ParamType::FLOAT, &pos_vel_kp,  3.0f,  0.0f, 10.0f,  &notifyControllerReload},
    {"position.vel.ti",   ParamType::FLOAT, &pos_vel_ti,  2.0f,  0.1f, 100.0f, &notifyControllerReload},
    {"position.stick_vel", ParamType::FLOAT, &pos_stick_vel, 0.4f, 0.05f, 2.0f, &notifyControllerReload},

    // ESKF process noise
    {"eskf.process.gyro_noise",  ParamType::FLOAT, &eskf_gyro_noise,  0.009655f, 0.001f, 1.0f,  &notifyEstimatorReload},
    {"eskf.process.accel_noise", ParamType::FLOAT, &eskf_accel_noise, 0.3f,      0.01f,  10.0f, &notifyEstimatorReload},
    {"eskf.process.gyro_bias",   ParamType::FLOAT, &eskf_gyro_bias,   0.000013f, 1e-7f,  0.01f, &notifyEstimatorReload},
    {"eskf.process.accel_bias",  ParamType::FLOAT, &eskf_accel_bias,  0.0001f,   1e-7f,  0.01f, &notifyEstimatorReload},
    // Min lowered to 0 so eskf.bias.gyro_dev_max=0 FREEZES the gyro bias at the boot
    // still-calibration nominal (no in-flight ESKF update reaches the rate loop) — a
    // diagnostic toggle. Restore 0.03 for normal random-walk tracking.
    // 最小を0に下げ、=0 で起動静止校正値にジャイロバイアスを凍結（飛行中ESKF更新を
    // レートループへ反映しない）。診断用トグル。通常は0.03へ戻す。
    {"eskf.bias.gyro_dev_max",   ParamType::FLOAT, &eskf_bg_dev_max,  0.03f,     0.0f, 1.0f,  &notifyEstimatorReload},

    // ESKF observation noise
    {"eskf.obs.tof_noise",       ParamType::FLOAT, &eskf_tof_noise,     0.01f, 0.001f, 1.0f,  &notifyEstimatorReload},
    {"eskf.obs.flow_noise",      ParamType::FLOAT, &eskf_flow_noise,    0.30f, 0.01f,  5.0f,  &notifyEstimatorReload},
    {"eskf.obs.baro_noise",      ParamType::FLOAT, &eskf_baro_noise,    0.1f,  0.01f,  5.0f,  &notifyEstimatorReload},
    {"eskf.obs.mag_noise",       ParamType::FLOAT, &eskf_mag_noise,     1.0f,  0.01f,  10.0f, &notifyEstimatorReload},
    {"eskf.obs.accel_att_noise", ParamType::FLOAT, &eskf_accel_att,     1.2f,  0.001f, 2.0f,  &notifyEstimatorReload},
    {"eskf.obs.accel_att_lpf",   ParamType::FLOAT, &eskf_accel_att_lpf, 30.0f, 0.0f,   200.0f,&notifyEstimatorReload},

    // ESKF sensor enable
    {"eskf.use_tof",  ParamType::BOOL, &eskf_use_tof,  1.0f, 0.0f, 1.0f, &notifyEstimatorReload},
    {"eskf.use_flow", ParamType::BOOL, &eskf_use_flow, 1.0f, 0.0f, 1.0f, &notifyEstimatorReload},
    {"eskf.use_baro", ParamType::BOOL, &eskf_use_baro, 0.0f, 0.0f, 1.0f, &notifyEstimatorReload},
    {"eskf.use_mag",  ParamType::BOOL, &eskf_use_mag,  0.0f, 0.0f, 1.0f, &notifyEstimatorReload},

    // ESKF gates
    {"eskf.gate.mahalanobis", ParamType::FLOAT, &eskf_mahalanobis, 15.0f, 1.0f,  100.0f, &notifyEstimatorReload},
    {"eskf.gate.tof_innov",   ParamType::FLOAT, &eskf_tof_innov,   0.5f,  0.01f, 5.0f,   &notifyEstimatorReload},
    {"eskf.gate.baro_innov",  ParamType::FLOAT, &eskf_baro_innov,  0.5f,  0.01f, 5.0f,   &notifyEstimatorReload},
    {"eskf.gate.flow_clamp",  ParamType::FLOAT, &eskf_flow_clamp,  0.3f,  0.01f, 5.0f,   &notifyEstimatorReload},
    {"eskf.gate.flow_squal",  ParamType::INT,   &eskf_flow_squal,  10.0f, 0.0f,  255.0f, &notifyEstimatorReload},

    // ESKF accel-attitude (SSOT — proven firmware/vehicle values)
    {"eskf.att.k_adaptive",   ParamType::FLOAT, &eskf_att_k_adaptive, 10.0f, 0.0f,  100.0f, &notifyEstimatorReload},
    {"eskf.att.chi2_gate",    ParamType::FLOAT, &eskf_att_chi2_gate,  7.81f, 0.0f,  100.0f, &notifyEstimatorReload},
    {"eskf.att.corr_clamp",   ParamType::FLOAT, &eskf_att_corr_clamp, 0.05f, 0.001f, 1.0f,  &notifyEstimatorReload},

    // ESKF acceleration-compensated accel-attitude (POS_HOLD; α-β flow-acceleration tracker)
    {"eskf.accel_comp.enable", ParamType::BOOL,  &eskf_accel_comp_enable, 1.0f,  0.0f,  1.0f,  &notifyEstimatorReload},
    {"eskf.accel_comp.alpha",  ParamType::FLOAT, &eskf_accel_comp_alpha,  0.2f,  0.01f, 1.0f,  &notifyEstimatorReload},
    {"eskf.accel_comp.beta",   ParamType::FLOAT, &eskf_accel_comp_beta,   0.02f, 0.0f,  1.0f,  &notifyEstimatorReload},
    {"eskf.accel_comp.max",    ParamType::FLOAT, &eskf_accel_comp_max,    5.0f,  0.5f,  20.0f, &notifyEstimatorReload},

    // Safety
    {"safety.impact.accel_g",  ParamType::FLOAT, &safety_accel_g,     3.0f,   1.0f,   10.0f,   nullptr},
    {"safety.impact.gyro_dps", ParamType::FLOAT, &safety_gyro_dps,    800.0f, 100.0f, 2000.0f, nullptr},
    {"safety.comm.timeout_ms", ParamType::FLOAT, &safety_comm_timeout, 500.0f, 100.0f, 5000.0f, nullptr},
    {"safety.battery.low_v",   ParamType::FLOAT, &safety_low_v,       3.4f,   3.0f,   4.2f,    nullptr},
    {"safety.battery.usb_v",   ParamType::FLOAT, &safety_usb_v,       3.3f,   2.5f,   3.5f,    nullptr},

    // Calibration — boot gyro/accel bias calibration on/off
    {"calibration.enable",     ParamType::BOOL,  &calibration_enable, 1.0f,   0.0f,   1.0f,    nullptr},
};

static constexpr int TABLE_SIZE = sizeof(table) / sizeof(table[0]);
static const char* NVS_NAMESPACE = "sf_params";

// =============================================================================
// NVS key derivation
// NVS キー導出
//
// NVS limits key names to 15 characters; 31 of the 54 parameter names exceed
// that (e.g. "eskf.process.accel_noise" = 24), so storing under the raw name
// fails with ESP_ERR_NVS_KEY_TOO_LONG. We derive a fixed-length 9-character
// key "p" + 8-hex FNV-1a hash of the name. Hash keys are stable across table
// reordering (unlike index-based keys, which would silently load the wrong
// value after an insertion). init() verifies there are no hash collisions.
// NVS のキー名は15文字まで。54個中31個のパラメータ名がこれを超え（例:
// "eskf.process.accel_noise" は24文字）、生の名前では ESP_ERR_NVS_KEY_TOO_LONG で
// 保存に失敗する。そこで名前の FNV-1a ハッシュから固定長9文字のキー
// "p"+16進8桁 を導出する。ハッシュキーはテーブルの並べ替えに不変（インデックス
// ベースだと行挿入後に黙って別の値を読んでしまう）。init() で衝突がないことを検証。
// =============================================================================

static uint32_t fnv1aHash(const char* s)
{
    uint32_t hash = 2166136261u;            // FNV offset basis
    while (*s) {
        hash ^= static_cast<uint8_t>(*s++);
        hash *= 16777619u;                  // FNV prime
    }
    return hash;
}

static void nvsKeyFor(const char* name, char out[16])
{
    snprintf(out, 16, "p%08lx",
             static_cast<unsigned long>(fnv1aHash(name)));
}

// =============================================================================
// Find parameter by name
// 名前でパラメータを検索する
// =============================================================================

static const ParamEntry* find(const char* name)
{
    for (int i = 0; i < TABLE_SIZE; i++) {
        if (strcmp(table[i].name, name) == 0) {
            return &table[i];
        }
    }
    return nullptr;
}

// =============================================================================
// Public API Implementation
// 公開API実装
// =============================================================================

void init()
{
    // One-time NVS-key collision check: two names hashing to the same key would
    // silently alias their stored values. With ~100 names on a 32-bit hash the
    // probability is ~1e-6, but verify anyway — this is the kind of failure that
    // is invisible until a parameter "mysteriously" loads someone else's value.
    // NVS キー衝突の一回限り検査: 2つの名前が同じキーにハッシュされると保存値が
    // 黙って混線する。約100名×32bitハッシュで確率は ~1e-6 だが念のため検証 — これは
    // パラメータが「謎に」他人の値を読むまで見えない種類の故障。
    for (int i = 0; i < TABLE_SIZE; i++) {
        for (int j = i + 1; j < TABLE_SIZE; j++) {
            if (fnv1aHash(table[i].name) == fnv1aHash(table[j].name)) {
                ESP_LOGE(TAG, "NVS key collision: '%s' vs '%s' — rename one!",
                         table[i].name, table[j].name);
            }
        }
    }

    load();
    ESP_LOGI(TAG, "Parameter system initialized (%d params)", TABLE_SIZE);
}

bool get_float(const char* name, float& out)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::FLOAT) return false;
    out = *static_cast<float*>(e->value_ptr);
    return true;
}

bool get_bool(const char* name, bool& out)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::BOOL) return false;
    out = *static_cast<bool*>(e->value_ptr);
    return true;
}

bool get_int(const char* name, int32_t& out)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::INT) return false;
    out = *static_cast<int32_t*>(e->value_ptr);
    return true;
}

bool set_float(const char* name, float value)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::FLOAT) {
        ESP_LOGW(TAG, "set_float: '%s' not found", name);
        return false;
    }

    // Validate range
    // 範囲を検証
    if (value < e->min_val || value > e->max_val) {
        ESP_LOGW(TAG, "set_float: '%s' = %f out of range [%f, %f]",
                 name, value, e->min_val, e->max_val);
        return false;
    }

    *static_cast<float*>(e->value_ptr) = value;
    ESP_LOGI(TAG, "set: %s = %f", name, value);

    // Call callback if registered
    // コールバックが登録されていれば呼ぶ
    if (e->callback) {
        e->callback();
    }

    return true;
}

bool set_bool(const char* name, bool value)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::BOOL) return false;

    *static_cast<bool*>(e->value_ptr) = value;
    ESP_LOGI(TAG, "set: %s = %s", name, value ? "true" : "false");

    if (e->callback) {
        e->callback();
    }
    return true;
}

bool set_int(const char* name, int32_t value)
{
    const ParamEntry* e = find(name);
    if (!e || e->type != ParamType::INT) return false;

    if (value < static_cast<int32_t>(e->min_val) ||
        value > static_cast<int32_t>(e->max_val)) {
        ESP_LOGW(TAG, "set_int: '%s' = %ld out of range", name, (long)value);
        return false;
    }

    *static_cast<int32_t*>(e->value_ptr) = value;
    ESP_LOGI(TAG, "set: %s = %ld", name, (long)value);

    if (e->callback) {
        e->callback();
    }
    return true;
}

// Write a single entry's current RAM value into an already-open NVS handle,
// under its hashed key. Shared by save() (all entries) and save_one()
// (single entry) so the type-dispatch logic lives in one place.
// 開いている NVS ハンドルへ、1エントリの現在の RAM 値をハッシュキーで書き込む。
// save()（全件）と save_one()（1件）で共用し、型分岐ロジックを一箇所にまとめる。
static esp_err_t saveEntryToNvs(nvs_handle_t handle, const ParamEntry& e)
{
    char key[16];
    nvsKeyFor(e.name, key);

    if (e.type == ParamType::FLOAT) {
        float val = *static_cast<float*>(e.value_ptr);
        // Store float as uint32_t bit pattern
        // floatをuint32_tビットパターンとして保存
        uint32_t raw;
        memcpy(&raw, &val, sizeof(float));
        return nvs_set_u32(handle, key, raw);
    } else if (e.type == ParamType::BOOL) {
        bool val = *static_cast<bool*>(e.value_ptr);
        return nvs_set_u8(handle, key, val ? 1 : 0);
    } else if (e.type == ParamType::INT) {
        return nvs_set_i32(handle, key, *static_cast<int32_t*>(e.value_ptr));
    }
    return ESP_ERR_INVALID_ARG;
}

void save()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }

    int saved = 0;
    int failed = 0;
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        esp_err_t set_err = saveEntryToNvs(handle, e);

        // Count failures instead of silently claiming success — the old code
        // ignored every nvs_set_* error and logged a bogus "Saved N parameters".
        // 失敗を数える（黙って成功を装わない）— 旧実装は nvs_set_* のエラーを全て
        // 無視し、偽の「Saved N parameters」を出していた。
        if (set_err == ESP_OK) {
            saved++;
        } else {
            failed++;
            ESP_LOGE(TAG, "save: '%s' failed: %s", e.name, esp_err_to_name(set_err));
        }
    }

    esp_err_t commit_err = nvs_commit(handle);
    nvs_close(handle);
    if (commit_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(commit_err));
    } else if (failed > 0) {
        ESP_LOGW(TAG, "Saved %d parameters to NVS (%d FAILED)", saved, failed);
    } else {
        ESP_LOGI(TAG, "Saved %d parameters to NVS", saved);
    }
}

bool has_saved(const char* name)
{
    // Return true if this parameter has a value saved in NVS.
    // このパラメータが NVS に保存済みなら true を返す。
    const ParamEntry* e = find(name);
    if (!e) return false;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // Namespace not created yet — nothing has ever been saved.
        // namespace が未作成 — まだ何も保存されていない。
        return false;
    }

    char key[16];
    nvsKeyFor(e->name, key);
    bool found = (nvs_find_key(handle, key, nullptr) == ESP_OK);
    nvs_close(handle);
    return found;
}

bool save_one(const char* name)
{
    // Save ONLY this parameter to NVS (unlike save(), which writes all).
    // このパラメータ1件だけを NVS へ保存する（save() は全件書くのと対照）。
    const ParamEntry* e = find(name);
    if (!e) {
        ESP_LOGW(TAG, "save_one: '%s' not found", name);
        return false;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return false;
    }

    esp_err_t set_err = saveEntryToNvs(handle, *e);
    esp_err_t commit_err = ESP_OK;
    if (set_err == ESP_OK) {
        commit_err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (set_err != ESP_OK) {
        ESP_LOGE(TAG, "save_one: '%s' failed: %s", name, esp_err_to_name(set_err));
        return false;
    }
    if (commit_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(commit_err));
        return false;
    }
    return true;
}

void load()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // NVS not initialized or no data — use defaults
        // NVS未初期化またはデータなし — デフォルトを使用
        ESP_LOGI(TAG, "No saved parameters, using defaults");
        return;
    }

    int loaded = 0;
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        char key[16];
        nvsKeyFor(e.name, key);

        if (e.type == ParamType::FLOAT) {
            uint32_t raw;
            if (nvs_get_u32(handle, key, &raw) == ESP_OK) {
                float val;
                memcpy(&val, &raw, sizeof(float));
                // Validate range before applying
                // 適用前に範囲を検証
                if (val >= e.min_val && val <= e.max_val && !std::isnan(val)) {
                    *static_cast<float*>(e.value_ptr) = val;
                    loaded++;
                }
            }
        } else if (e.type == ParamType::BOOL) {
            uint8_t raw;
            if (nvs_get_u8(handle, key, &raw) == ESP_OK) {
                *static_cast<bool*>(e.value_ptr) = (raw != 0);
                loaded++;
            }
        } else if (e.type == ParamType::INT) {
            int32_t raw;
            if (nvs_get_i32(handle, key, &raw) == ESP_OK) {
                if (raw >= static_cast<int32_t>(e.min_val) &&
                    raw <= static_cast<int32_t>(e.max_val)) {
                    *static_cast<int32_t*>(e.value_ptr) = raw;
                    loaded++;
                }
            }
        }
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Loaded %d parameters from NVS", loaded);
}

void reset_all()
{
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        if (e.type == ParamType::FLOAT) {
            *static_cast<float*>(e.value_ptr) = e.default_val;
        } else if (e.type == ParamType::BOOL) {
            *static_cast<bool*>(e.value_ptr) = (e.default_val != 0.0f);
        } else if (e.type == ParamType::INT) {
            *static_cast<int32_t*>(e.value_ptr) = static_cast<int32_t>(e.default_val);
        }
    }

    // Fire each DISTINCT change callback once so the owning tasks re-read the
    // restored defaults live (same path as `param set`). Firing per-row would
    // flood the small command queues with dozens of identical ReloadParams verbs.
    // 「異なる」変更コールバックを1回ずつ発火し、所有タスクに復元後の既定値を
    // ライブで読み直させる（`param set` と同じ経路）。行ごとに発火すると小さな
    // コマンドキューが同一の ReloadParams で溢れる。
    for (int i = 0; i < TABLE_SIZE; i++) {
        if (table[i].callback == nullptr) continue;
        bool seen = false;
        for (int j = 0; j < i; j++) {
            if (table[j].callback == table[i].callback) { seen = true; break; }
        }
        if (!seen) table[i].callback();
    }

    ESP_LOGI(TAG, "All %d parameters reset to defaults", TABLE_SIZE);
}

void list()
{
    ESP_LOGI(TAG, "=== Parameters (%d) ===", TABLE_SIZE);
    for (int i = 0; i < TABLE_SIZE; i++) {
        const ParamEntry& e = table[i];
        if (e.type == ParamType::FLOAT) {
            ESP_LOGI(TAG, "  %-30s = %f  [%f, %f]",
                     e.name, *static_cast<float*>(e.value_ptr),
                     e.min_val, e.max_val);
        } else if (e.type == ParamType::BOOL) {
            ESP_LOGI(TAG, "  %-30s = %s",
                     e.name, *static_cast<bool*>(e.value_ptr) ? "true" : "false");
        } else if (e.type == ParamType::INT) {
            ESP_LOGI(TAG, "  %-30s = %ld  [%ld, %ld]",
                     e.name, (long)*static_cast<int32_t*>(e.value_ptr),
                     (long)e.min_val, (long)e.max_val);
        }
    }
}

int count()
{
    return TABLE_SIZE;
}

const ParamEntry* entry(int index)
{
    if (index < 0 || index >= TABLE_SIZE) return nullptr;
    return &table[index];
}

}  // namespace params
}  // namespace sf
