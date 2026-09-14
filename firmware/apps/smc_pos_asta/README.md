# smc_pos_asta — レート+速度ループの適応スーパーツイスティング法(ASTA)実験

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

Tier: L1 (Topic API) / Type: embedded / 由来: `firmware/apps/smc_rate_asta`, `firmware/apps/smc_pos_sta`

## 1. 目的

`smc_rate_asta`（レートループへの適応スイッチングゲイン付きスーパーツイスティング法、
規範モデル+トレンド判定、`docs/plans/smc-rate-loop-plan.md` §7.36で最終確定）の
適応則を、**POS_HOLDの水平速度ループにも初めて適用**する。

適応STA（ASTA）はこれまでレートループでしか検証されておらず、位置/速度ループへの
適用は一度も試されていなかった——固定ゲインSTA（`smc_pos_sta`、既にレート+速度両方で
実装済み）との比較でこの空白を埋めるため、2026-09-13のセッションで新規作成した
（同セッションの結論: 位置制御へのSMC適用は`smc_pos`/`smc_pos_sta`とも2回断念済み
——§7.9「安全だが既存PIDを明確に上回らない」、§7.27/7.28「実機クラッシュ」——
その上で「適応版はまだ試していない」という指摘を受けて着手）。

`AppController`は`PidController`へ`thrust`・高度・**位置ループ（pos_x_/pos_y_）**・
離着陸フェーズ・誘導・トリム/ホバー推力学習・DOBを丸ごと委譲しつつ、以下の2箇所
だけを適応STAへ置き換える:

- roll/pitch/yawの最終レートループ（`smc_rate_asta`と同じ制御則）
- 水平速度ループ（vx/vy→ax/ay、`setVelocityLawOverride()`経由）

5インスタンス（レート3軸+速度2軸）全てが`adaptive_sliding_mode_sta.hpp`の
`AdaptiveSuperTwisting`構造体（`smc_rate_asta.hpp`の適応則を`output_scale`
フィールドで汎用化したもの）を共有する——`smc_pos_sta`が固定ゲイン版
`SuperTwisting`（`sliding_mode_sta.hpp`）を汎用化したのと同じ方法。

## 2. 現状（重要）

**全パラメータは未検証のシード値**——レート軸は`smc_asta.*`の既定値、速度軸は
`smc_pos_sta.velx/vely`の固定ゲイン（k1=0.6/k2=0.3）に合わせ、適応則の時定数
（`filter_tau`/`mref_*`）はレートループの既定値のまま、`adapt_rate`等のゲイン
スケール依存パラメータのみk1と同じ比率でスケールダウンした値。SILSでの
チューニングは未実施。

初期検証（`docs/plans/smc-rate-loop-plan.md` §7.50）では、`stab_flight`は
健全に動作したが、POS_HOLD本体（`pos_flight`系）では未調整のまま固定ゲイン版
`smc_pos_sta`に劣った（`pos_flight`nominalでdrift 3.03m FAIL vs 固定版2.09m
PASS）。`smc_rate_asta`が最初に作られたとき（§7.31）と同じパターンで、
多シナリオ同時チューニング（§7.31〜7.36相当）を経ずに固定ゲイン版と比較する
段階には至っていない。

**【2026-09-14更新・下記「実機投入は行わない」は撤回】** §7.51のチューニング後、
実際には§7.52以降で**実機に複数回投入・飛行済み**——120秒飛行（§7.53、事故は
DISARM操作によるものと判明、制御則自体は健全）、Smith予測器型むだ時間補償
（§7.54続報10）投入後は計139秒・3回の健全な飛行を実機テレメトリで確認済み
（§7.54続報13、tilt_max 11〜13°）。実機投入時は**必ずSmith予測器を実測遅延
ベースのパラメータ**（`predictor_tau_m`: roll 0.062・pitch 0.061・yaw 0.13・
velx/vely 0.06、既定は0=無効）**で有効化**した上で飛行させること——無効の
まま（既定ビルド）投入すると、`--motor-delay 60`相当の実測遅延下でSILS上
180°完全転倒するレベルの自励振動が起き得る（§7.54続報11の横並び比較）。
実機投入前チェックリストは`docs/plans/smc-rate-loop-plan.md` §7.54続報10
「今後の方針」参照。

以下は2026-09-13時点の初期方針（撤回済み・履歴として残す）:

> **実機投入は行わない**——`smc_pos_sta`と同じ理由（§7.27/7.28: 位置制御への
> STA適用が実機クラッシュを起こした前例があり、ユーザー判断で実機投入を中止、
> SILS限定の実験としている）。本appも同じ方針を最初から適用する。

## 3. 関連ドキュメント

| ドキュメント | 内容 |
|---|---|
| `docs/plans/smc-rate-loop-plan.md` §7.36 | 適応則（規範モデル+トレンド判定）の設計根拠、レートループでの検証経緯 |
| `docs/plans/smc-rate-loop-plan.md` §7.50 | 本appの新規作成・初期検証結果 |
| `docs/plans/smc-rate-loop-plan.md` §7.54続報10 | Smith予測器型むだ時間補償の実装（実機投入に必須） |
| `docs/plans/smc-rate-loop-plan.md` §7.54続報12-13 | 実機投入・実機テレメトリでの検証結果 |
| `docs/plans/smc-rate-loop-plan.md` §7.55続報 | 位置ループ（`pid_.pos_x_/pos_y_`、本app無改造のまま委譲）のゲイン変更は実機発散の既往あり、触らないこと |
| `firmware/apps/smc_rate_asta/smc_rate_asta.hpp` | 適応則の元設計（設計変遷§7.31→7.36の全経緯） |
| `firmware/apps/smc_pos_sta/` | 固定ゲイン版（比較対象、レート+速度両方に実装済み） |

---

<a id="english"></a>

## 1. Purpose

Applies `smc_rate_asta`'s adaptive switching-gain super-twisting law (reference
model + trend-based divergence gate, finalized in `docs/plans/smc-rate-loop-plan.md`
section 7.36) to the POS_HOLD horizontal velocity loop for the **first time**.

Adaptive STA (ASTA) had only ever been validated on the rate loop; it had never
been applied to the position/velocity loop. This app fills that gap for comparison
against the fixed-gain STA (`smc_pos_sta`, already implemented for both rate and
velocity) -- created in the 2026-09-13 session after that session established that
SMC for position control had been shelved twice (`smc_pos`/`smc_pos_sta` -- section
7.9 "safe but not clearly superior to PID", sections 7.27/7.28 "real hardware
crash"), and the user then asked whether the adaptive variant, never tried, might
change that picture.

`AppController` delegates thrust, altitude, the **position loop** (pos_x_/pos_y_),
takeoff/landing phases, guidance, trim/hover-thrust learning, and DOB entirely to
`PidController`, replacing only:

- The final rate-loop torque (roll/pitch/yaw) -- same control law as `smc_rate_asta`
- The horizontal velocity loop (vx/vy -> ax/ay, via `setVelocityLawOverride()`)

All 5 instances (3 rate axes + 2 velocity axes) share `adaptive_sliding_mode_sta
.hpp`'s `AdaptiveSuperTwisting` struct (a generalization of `smc_rate_asta.hpp`'s
adaptive law with an `output_scale` field) -- the same way `smc_pos_sta` generalized
the fixed-gain `SuperTwisting` struct (`sliding_mode_sta.hpp`).

## 2. Current Status (Important)

**All parameters are unverified seed values** -- rate axes from `smc_asta.*`'s own
defaults; velocity axes matched to `smc_pos_sta.velx/vely`'s fixed-gain k1/k2
(k1=0.6/k2=0.3), with the adaptive law's time constants (`filter_tau`/`mref_*`)
left unchanged from the rate-loop defaults and gain-scale-dependent parameters
(`adapt_rate` etc.) scaled down by the same ratio as k1 itself. No SILS tuning has
been done.

Initial verification (`docs/plans/smc-rate-loop-plan.md` section 7.50): `stab_flight`
flies normally, but the POS_HOLD capstone (`pos_flight` family) underperforms the
fixed-gain `smc_pos_sta` with untuned seeds (nominal drift 3.03m FAIL vs. the
fixed version's 2.09m PASS) -- the same pattern seen when `smc_rate_asta` was
first created (section 7.31), before its own multi-scenario tuning cycle (sections
7.31-7.36). This app has not yet gone through an equivalent tuning cycle, so no
fair comparison against the fixed-gain baseline exists yet.

**[Updated 2026-09-14 -- the "not cleared" line below is retracted]** After the
section 7.51 tuning, this app was actually **deployed and flown on real
hardware multiple times** starting in section 7.52 -- a 120 s flight (section
7.53; the one incident traced to a pilot DISARM mid-air-cut, not a control-law
divergence), and after the Smith-predictor dead-time compensator (section
7.54 follow-up 10) went in, three healthy flights totaling 139 s were
confirmed on real telemetry (section 7.54 follow-up 13, tilt_max 11-13 deg).
Real-hardware deployment **requires** enabling the Smith predictor with the
measured-delay parameters (`predictor_tau_m`: roll 0.062 / pitch 0.061 / yaw
0.13 / velx/vely 0.06 -- default is 0, disabled) -- flying the default build
(predictor disabled) under the real measured delay (~60 ms, matching
`--motor-delay 60`) can self-excite oscillation up to a full 180-degree
tumble in SILS (section 7.54 follow-up 11's side-by-side comparison). See the
pre-flight checklist in `docs/plans/smc-rate-loop-plan.md` section 7.54
follow-up 10's "Next steps".

The original (2026-09-13) policy below is retracted; kept for history:

> **Not cleared for real-hardware deployment** -- same reasoning as `smc_pos_sta`
> (sections 7.27/7.28: STA applied to position control caused a real crash; the
> user decided to stop real-hardware deployment of position-control SMC and keep
> it SILS-only). This app adopts that same policy from the start.

## 3. Related Documents

| Document | Content |
|---|---|
| `docs/plans/smc-rate-loop-plan.md` section 7.36 | Adaptive-law design rationale (reference model + trend gate), rate-loop validation history |
| `docs/plans/smc-rate-loop-plan.md` section 7.50 | This app's creation and initial verification results |
| `docs/plans/smc-rate-loop-plan.md` section 7.54 follow-up 10 | Smith-predictor dead-time compensator (required for real-hardware deployment) |
| `docs/plans/smc-rate-loop-plan.md` section 7.54 follow-ups 12-13 | Real-hardware deployment and real-telemetry validation |
| `docs/plans/smc-rate-loop-plan.md` section 7.55 follow-up | The position loop (`pid_.pos_x_/pos_y_`, left untouched by this app) has a known real-hardware divergence history -- do not retune it |
| `firmware/apps/smc_rate_asta/smc_rate_asta.hpp` | The adaptive law's original design (full section 7.31->7.36 design history) |
| `firmware/apps/smc_pos_sta/` | The fixed-gain counterpart (comparison baseline, already implemented for both rate and velocity) |
