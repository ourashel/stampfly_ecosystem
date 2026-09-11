# smc_pos_sta — レート+速度ループのスーパーツイスティング法(STA)実験

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

Tier: L1 (Topic API) / Type: embedded / 由来: `firmware/apps/smc_rate_sta`, `firmware/apps/smc_pos`

## 1. 目的

`smc_rate_sta`（レートループへのスーパーツイスティング法(STA)適用、1次SMCに対し
チャタリング抑制・z漏れ積分による安全性の両面で改善を達成、実機2回検証済み）の
知見を**POS_HOLDの水平速度ループにも適用**する。`smc_pos`（1次SMCの速度ループ、
`docs/plans/smc-rate-loop-plan.md` §7.9で「安全だが既存PIDを明確に上回らない」と
見送り済み）とは別の新規appとして、1次SMCではなくSTA（2次スライディングモード）
で速度ループを再検討する。

`AppController`は`PidController`へ`thrust`・高度・**位置ループ（pos_x_/pos_y_）**・
離着陸フェーズ・誘導・トリム/ホバー推力学習・DOBを丸ごと委譲しつつ、以下の2箇所
だけを置き換える:

- roll/pitch/yawの最終レートループ（`smc_rate_sta`と同じ制御則、STA）
- 水平速度ループ（`PidController::setVelocityLawOverride()`経由で新規注入、STA）

設計の全体像・見送りの経緯・今回再検討する理由は
[`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md) §7・
§7.11-7.24参照。

**レート軸・速度軸を1つの汎用構造体`SuperTwisting`（`sliding_mode_sta.hpp`）で
共通化**している——`smc_rate_sta.hpp`の`SuperTwistingRate`（トルク出力）と、
別々に実装していたら生まれていたはずの速度版とは、出力スケール乗数
（レート軸=慣性、速度軸=1.0）の有無だけが違う同一の制御則だったため。詳細は
`sliding_mode_sta.hpp`のファイルヘッダ参照。

制御則（各軸独立。単位以外はレート・速度で同一）:
```
e = sp − meas                            （レート軸: rad/s、速度軸: m/s）
s = e + lambda_i・∫e dt                   （PI型面）
u1 = k1・sqrt(|s|)・sign(s)                （連続到達則の比例項）
z_dot = -k2・sign(s) − z/z_leak_tau        （漏れ積分、既定から有効）
output = output_scale・(u1+z)             （レート軸=トルク[Nm]、速度軸=加速度[m/s^2]）
```

## 2. 組み込み型であること

`type: embedded`（`app.yaml`参照）— `smc_rate_sta`/`smc_pos`と同じく単独ビルド不可、
`sf app`コマンドが`SF_APP_DIR`としてvehicle本体のmainコンポーネントへ直接
コンパイルする。

## 3. 使い方

SILSで動作を確認する:

```bash
sf app sils smc_pos_sta
```

4者比較（PIDベースライン / レートのみSTA / レート+速度1次SMC / レート+速度STA）:

```bash
sf sils scenario simulator/sils/scenarios/<scenario>.scn                        # PIDベースライン
sf app sils smc_rate_sta simulator/sils/scenarios/<scenario>.scn                # レートのみSTA
sf app sils smc_pos      simulator/sils/scenarios/<scenario>.scn                # レート+速度1次SMC
sf app sils smc_pos_sta  simulator/sils/scenarios/<scenario>.scn                # レート+速度STA
```

ゲインは`sf params`/`--param`でその場調整できる:

```bash
sf app sils smc_pos_sta --param smc_pos_sta.velx.k1=0.6 --param smc_pos_sta.velx.k2=0.3
```

実機向けにビルド・書き込み（**SILS検証・実機投入ゲートをクリアしてから**）:

```bash
sf app build smc_pos_sta
sf app flash smc_pos_sta -m
```

## 4. ファイル構成

| ファイル | 内容 |
|---------|------|
| `sliding_mode_sta.hpp` | `SuperTwisting` struct（汎用、新規）——`output_scale`パラメータでレート軸（トルク出力）・速度軸（加速度出力）の両方に使う |
| `app_controller.hpp`/`.cpp` | `IController`実装。`torque[0..2]`（3軸レートSTA）と`PidController`の速度ループ（`setVelocityLawOverride()`経由、2軸速度STA）を差し替える。5インスタンス全て`SuperTwisting`から生成 |

ゲイン（`smc_pos_sta.{roll,pitch,yaw,velx,vely}.{k1,k2,phi,lambda_i,e_reset,z_leak_tau}`、
計30個）は`firmware/vehicle/components/sf_core/params.cpp`に定義（`smc_rate_sta`・
`smc_pos`とは独立したキー空間、既定vehicle/他appビルドの挙動には影響しない追加のみの変更）。

## 5. 注意

| 項目 | 内容 |
|------|------|
| 委譲範囲 | 高度・**位置ループ**・離着陸フェーズ・誘導・トリム学習・DOB・ヘディングホールドは全て`PidController`委譲のまま — `architecture.md`のINV-1を壊さない |
| 位置ループは無改造 | `pos_x_`/`pos_y_`（位置誤差→目標速度）は既存PIDのまま。速度ループ（目標速度→加速度）だけを差し替える |
| **既にPIDで頑健化済み** | `position.vel.kp/ti`は実機プラント同定（`firmware/vehicle/docs/poshold_journey.md` §4）に基づき既に再設計・実機検証済み（K∈[2.8,7]で安定）。本appは「既存が壊れているから」ではなく、`smc_rate_sta`で実証されたSTA（チャタリング抑制＋z漏れ積分の安全性）が§7.9時点の1次SMCでは示せなかった優位性を出せるかをA/B/C/D比較で検証する目的で存在する |
| z漏れ積分は最初から有効 | `smc_rate_sta`が§7.17で持続外乱下の転倒を経験してから追加した経緯とは異なり、本appは同じ構造的リスクが最初から分かっているため`z_leak_tau`を既定非ゼロでスタートする |
| 無駄時間予測補償器は非搭載 | §7.9の見送り理由の1つがこの機構の安全マージン不足だったため、意図的に含めていない——素のSTA到達則自体の優位性を確認する |
| ゲインの位置づけ | `params.cpp`の初期値はSILSチューニングの「出発点」であり飛行検証済みではない（実装直後、チューニング未実施） |
| 実機投入ゲート | `smc_rate_sta`と同じ考え方——SILSが一通り揃い、既知の失敗モードを文書化した上でユーザーと協議してから`sf app flash smc_pos_sta -m`に進む |

---

<a id="english"></a>

## 1. Purpose

Applies the lessons from `smc_rate_sta` (super-twisting algorithm (STA) for
the rate loop -- a clear improvement over 1st-order SMC in both chattering
mitigation and, via the leaky z integrator, safety; validated on 2 real
flights) to **POS_HOLD's horizontal velocity loop** as well. Distinct from
`smc_pos` (1st-order SMC velocity loop, shelved by
`docs/plans/smc-rate-loop-plan.md` §7.9 as "safe but not clearly superior to
PID"), this new app reconsiders the velocity loop using STA (2nd-order
sliding mode) instead of the 1st-order design.

`AppController` delegates `thrust`, altitude, the **position loop**
(`pos_x_`/`pos_y_`), takeoff/landing phases, guidance, trim/hover-thrust
learning, and the DOB entirely to `PidController`, and replaces only:

- the final roll/pitch/yaw rate loop (same control law as `smc_rate_sta`, STA)
- the horizontal velocity loop (newly plugged in via
  `PidController::setVelocityLawOverride()`, STA)

See [`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md)
§7 and §7.11-7.24 for the full design rationale, the reasons the 1st-order
version was shelved, and why STA is being reconsidered now.

**The rate and velocity axes share one generic struct, `SuperTwisting`**
(`sliding_mode_sta.hpp`) -- `smc_rate_sta.hpp`'s `SuperTwistingRate` (torque
output) and what would have been a separately-written velocity version
differ ONLY in whether an output-scale multiplier applies (rate axis =
inertia, velocity axis = 1.0) -- otherwise the identical control law. See
`sliding_mode_sta.hpp`'s file header for the full rationale.

Control law (independent per axis; identical between rate and velocity axes
except for units):
```
e = sp - meas                            (rate axis: rad/s; velocity axis: m/s)
s = e + lambda_i * integral(e dt)        (PI-type surface)
u1 = k1 * sqrt(|s|) * sign(s)             (continuous reaching-law proportional term)
z_dot = -k2 * sign(s) - z/z_leak_tau      (leaky integral, enabled from the start)
output = output_scale * (u1 + z)          (rate axis: torque [Nm]; velocity axis: accel [m/s^2])
```

## 2. It is an embedded template

`type: embedded` (see `app.yaml`) — like `smc_rate_sta`/`smc_pos`, not
independently buildable; the `sf app` command compiles it into the vehicle
body's main component via `SF_APP_DIR`.

## 3. Usage

Verify it in SILS:

```bash
sf app sils smc_pos_sta
```

Four-way comparison (PID baseline / rate-only STA / rate+velocity 1st-order
SMC / rate+velocity STA):

```bash
sf sils scenario simulator/sils/scenarios/<scenario>.scn                        # PID baseline
sf app sils smc_rate_sta simulator/sils/scenarios/<scenario>.scn                # rate-only STA
sf app sils smc_pos      simulator/sils/scenarios/<scenario>.scn                # rate+velocity 1st-order SMC
sf app sils smc_pos_sta  simulator/sils/scenarios/<scenario>.scn                # rate+velocity STA
```

Tune the gains live via `sf params`/`--param`:

```bash
sf app sils smc_pos_sta --param smc_pos_sta.velx.k1=0.6 --param smc_pos_sta.velx.k2=0.3
```

Build/flash for real hardware (**only after SILS validation and the
hardware-flight gate**):

```bash
sf app build smc_pos_sta
sf app flash smc_pos_sta -m
```

## 4. Files

| File | Contents |
|------|----------|
| `sliding_mode_sta.hpp` | `SuperTwisting` struct (generic, new) -- an `output_scale` parameter makes it usable for both rate axes (torque output) and velocity axes (acceleration output) |
| `app_controller.hpp`/`.cpp` | `IController` implementation. Replaces `torque[0..2]` (3-axis rate STA) and `PidController`'s velocity loop (via `setVelocityLawOverride()`, 2-axis velocity STA). All 5 instances built from `SuperTwisting` |

The gains (`smc_pos_sta.{roll,pitch,yaw,velx,vely}.{k1,k2,phi,lambda_i,e_reset,z_leak_tau}`,
30 total) live in `firmware/vehicle/components/sf_core/params.cpp` (an
independent key space from `smc_rate_sta`/`smc_pos`, an additive-only change
with no effect on the default vehicle/other-app builds).

## 5. Notes

| Item | Detail |
|------|--------|
| Delegation scope | Altitude, the **position loop**, takeoff/landing phases, guidance, trim learning, DOB, and heading hold all remain delegated to `PidController` — does not break `architecture.md`'s INV-1 |
| Position loop unchanged | `pos_x_`/`pos_y_` (position error → target velocity) stays the existing PID. Only the velocity loop (target velocity → acceleration) is replaced |
| **Already hardware-robustified** | `position.vel.kp/ti` was already redesigned and hardware-validated from a real plant identification (`firmware/vehicle/docs/poshold_journey.md` §4, stable over K∈[2.8,7]). This app does NOT exist because the baseline is known broken — it exists to A/B/C/D-test whether the STA improvements proven on `smc_rate_sta` (chattering mitigation + leaky-integrator safety) can show the clear superiority the 1st-order design (§7.9) could not |
| Leaky z from the start | Unlike `smc_rate_sta` (which added the leak only after §7.17's real tumble discovery), this app starts with `z_leak_tau` nonzero from day one, since the same structural risk is already known |
| No dead-time predictor | One of §7.9's shelving reasons was that mechanism's insufficient safety margin, so it is deliberately omitted here -- this lets the bare STA reaching law's own merit be evaluated |
| Gain status | The seed values in `params.cpp` are SILS-tuning starting points, not flight-validated gains (just implemented, not yet tuned) |
| Real-hardware gate | Same approach as `smc_rate_sta` -- once SILS validation is complete and known failure modes are documented, discuss with the user before `sf app flash smc_pos_sta -m` |
