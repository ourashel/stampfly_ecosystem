# smc_pos — レート+速度ループのスライディングモード制御(SMC)実験

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

Tier: L1 (Topic API) / Type: embedded / 由来: `firmware/apps/smc_rate`

## 1. 目的

`smc_rate`（レートループのみのSMC化）を土台に、**POS_HOLDの水平速度ループ
（vx/vy速度誤差→ax/ay加速度指令）もSMC化**する。`AppController`は`PidController`へ
`thrust`・高度・**位置ループ（pos_x_/pos_y_）**・離着陸フェーズ・誘導・トリム/ホバー推力学習・
DOBを丸ごと委譲しつつ、以下の2箇所だけを置き換える:

- roll/pitch/yawの最終レートループ（`smc_rate`から無変更で流用）
- 水平速度ループ（`PidController::setVelocityLawOverride()`経由で新規注入）

設計の全体像・「既にPIDで頑健化済み」という留保を含む調査根拠は
[`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md) §7参照。

制御則（速度ループ、各軸独立。レートループは`smc_rate`と同一）:
```
s = vel_sp − 測定NED速度                 （vel_spは位置ループ pos_x_/pos_y_ の出力、無改造）
accel = k·sat(s/φ) + η·s                  （sat()はチャタリング抑制の境界層）
```

## 2. 組み込み型であること

`type: embedded`（`app.yaml`参照）— `smc_rate`と同じく単独ビルド不可、`sf app`コマンドが
`SF_APP_DIR`としてvehicle本体のmainコンポーネントへ直接コンパイルする。

## 3. 使い方

SILSで動作を確認する:

```bash
sf app sils smc_pos
```

3者比較（PIDベースライン / レートのみSMC / レート+速度SMC）:

```bash
sf sils scenario simulator/sils/scenarios/<scenario>.scn                    # PIDベースライン
sf app sils smc_rate simulator/sils/scenarios/<scenario>.scn                # レートのみSMC
sf app sils smc_pos  simulator/sils/scenarios/<scenario>.scn                # レート+速度SMC
```

速度ループのゲインは`sf params`/`--param`でその場調整できる:

```bash
sf app sils smc_pos --param smc.velx.k=0.6 --param smc.velx.eta=4.0
```

実機向けにビルド・書き込み（**SILS検証・実機投入ゲートをクリアしてから**）:

```bash
sf app build smc_pos
sf app flash smc_pos -m
```

## 4. ファイル構成

| ファイル | 内容 |
|---------|------|
| `smc_rate.hpp` | `SlidingModeRate` struct（`firmware/apps/smc_rate`から無変更で流用） |
| `smc_vel.hpp` | `SlidingModeVelocity` struct（新規。`SlidingModeRate`と同型、単位を加速度/速度に再導出） |
| `app_controller.hpp`/`.cpp` | `IController`実装。`torque[0..2]`（3軸レートSMC）と`PidController`の速度ループ（`setVelocityLawOverride()`経由、2軸速度SMC）を差し替える |

速度ループのゲイン（`smc.{velx,vely}.{k,eta,phi,lambda_i,e_reset}`、計10個）は
`firmware/vehicle/components/sf_core/params.cpp`に定義（既定vehicle/smc_rateビルドの挙動には
影響しない追加のみの変更）。

## 5. 注意

| 項目 | 内容 |
|------|------|
| 委譲範囲 | 高度・**位置ループ**・離着陸フェーズ・誘導・トリム学習・DOB・ヘディングホールドは全て`PidController`委譲のまま — `architecture.md`のINV-1を壊さない |
| 位置ループは無改造 | `pos_x_`/`pos_y_`（位置誤差→目標速度）は既存PIDのまま。速度ループ（目標速度→加速度）だけを差し替える |
| **既にPIDで頑健化済み** | `position.vel.kp/ti`は実機プラント同定（`firmware/vehicle/docs/poshold_journey.md` §4）に基づき既に再設計・実機検証済み（K∈[2.8,7]で安定）。本appは「既存が壊れているから」ではなく、実測されたモータ非対称性（`docs/plans/smc-rate-loop-plan.md` §3.15）に対する追加ロバスト性をA/B/C比較で検証する目的で存在する |
| ゲインの位置づけ | `params.cpp`の初期値はSILSチューニングの「出発点」であり飛行検証済みではない |
| **実機投入: 見送り（2026-09-11、`docs/plans/smc-rate-loop-plan.md` §7.9）** | 素の速度ループSMCは安全だが既存PIDを明確に上回らず、本来の差別化要因だった無駄時間予測補償器も安全マージンが狭すぎると判明したため、これ以上の実機投入・チューニングは行わない。コードは参考実装として残す |

---

<a id="english"></a>

## 1. Purpose

Building on `smc_rate` (rate-loop-only SMC), this app also converts POS_HOLD's
**horizontal velocity loop** (vx/vy velocity error → ax/ay acceleration
command) to sliding mode. `AppController` delegates `thrust`, altitude, the
**position loop** (`pos_x_`/`pos_y_`), takeoff/landing phases, guidance,
trim/hover-thrust learning, and the DOB entirely to `PidController`, and
replaces only:

- the final roll/pitch/yaw rate loop (reused unchanged from `smc_rate`)
- the horizontal velocity loop (newly plugged in via
  `PidController::setVelocityLawOverride()`)

See [`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md)
§7 for the full design rationale, including the caveat that the existing
PID was already hardware-robustified.

Control law (velocity loop, independent per axis; rate loop identical to
`smc_rate`):
```
s = vel_sp − measured NED velocity   (vel_sp is the position loop's (pos_x_/pos_y_, unchanged) output)
accel = k·sat(s/φ) + η·s              (sat() is the boundary layer, for chattering mitigation)
```

## 2. It is an embedded template

`type: embedded` (see `app.yaml`) — like `smc_rate`, not independently
buildable; the `sf app` command compiles it into the vehicle body's main
component via `SF_APP_DIR`.

## 3. Usage

Verify it in SILS:

```bash
sf app sils smc_pos
```

Three-way comparison (PID baseline / rate-only SMC / rate+velocity SMC):

```bash
sf sils scenario simulator/sils/scenarios/<scenario>.scn                    # PID baseline
sf app sils smc_rate simulator/sils/scenarios/<scenario>.scn                # rate-only SMC
sf app sils smc_pos  simulator/sils/scenarios/<scenario>.scn                # rate+velocity SMC
```

Tune the velocity-loop gains live via `sf params`/`--param`:

```bash
sf app sils smc_pos --param smc.velx.k=0.6 --param smc.velx.eta=4.0
```

Build/flash for real hardware (**only after SILS validation and the
hardware-flight gate**):

```bash
sf app build smc_pos
sf app flash smc_pos -m
```

## 4. Files

| File | Contents |
|------|----------|
| `smc_rate.hpp` | `SlidingModeRate` struct (reused unchanged from `firmware/apps/smc_rate`) |
| `smc_vel.hpp` | `SlidingModeVelocity` struct (new; same shape as `SlidingModeRate`, re-derived in acceleration/velocity units) |
| `app_controller.hpp`/`.cpp` | `IController` implementation. Replaces `torque[0..2]` (3-axis rate SMC) and `PidController`'s velocity loop (via `setVelocityLawOverride()`, 2-axis velocity SMC) |

The velocity-loop gains (`smc.{velx,vely}.{k,eta,phi,lambda_i,e_reset}`, 10
total) live in `firmware/vehicle/components/sf_core/params.cpp` (an
additive-only change with no effect on the default vehicle/smc_rate builds).

## 5. Notes

| Item | Detail |
|------|--------|
| Delegation scope | Altitude, the **position loop**, takeoff/landing phases, guidance, trim learning, DOB, and heading hold all remain delegated to `PidController` — does not break `architecture.md`'s INV-1 |
| Position loop unchanged | `pos_x_`/`pos_y_` (position error → target velocity) stays the existing PID. Only the velocity loop (target velocity → acceleration) is replaced |
| **Already hardware-robustified** | `position.vel.kp/ti` was already redesigned and hardware-validated from a real plant identification (`firmware/vehicle/docs/poshold_journey.md` §4, stable over K∈[2.8,7]). This app does NOT exist because the baseline is known broken — it exists to A/B/C-test additional robustness against the measured motor asymmetry (`docs/plans/smc-rate-loop-plan.md` §3.15) against that already-robustified baseline |
| Gain status | The seed values in `params.cpp` are SILS-tuning starting points, not flight-validated gains |
| **Real-hardware deployment: not pursued (2026-09-11, `docs/plans/smc-rate-loop-plan.md` §7.9)** | The bare velocity-loop SMC is safe but never clearly beat the existing PID, and the dead-time predictor that could have been the real differentiator turned out to have too narrow a safety margin. No further tuning or hardware deployment planned. Code kept as a reference implementation |
