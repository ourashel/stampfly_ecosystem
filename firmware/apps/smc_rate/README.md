# smc_rate — レートループのスライディングモード制御(SMC)実験

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

Tier: L1 (Topic API) / Type: embedded / 由来: `examples/11_app_controller`

## 1. 目的

`IController`を実装した`AppController`は、`PidController`（カスケードPID制御器）へ
`thrust`・高度・位置・離着陸フェーズ・誘導・トリム/ホバー推力学習・DOBを丸ごと委譲しつつ、
**roll/pitch/yawの最終レートループ（トルク出力）だけ**を1次スライディング面+境界層付き
到達則に置き換える。設計の全体像・調査根拠は
[`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md)参照。

制御則（各軸独立）:
```
s = rate_ref − 測定角速度                    （rate_refはPidController自身の最終レート目標）
torque = I_axis · ( k·sat(s/φ) + η·s )        （sat()はチャタリング抑制の境界層）
```

## 2. 組み込み型であること

`type: embedded`（`app.yaml`参照）— 単独ビルド可能な例題09/10とは異なり、`CMakeLists.txt`も
`main/`も持たない。ここに置いた`*.cpp`/`*.hpp`は`sf app`コマンドが`SF_APP_DIR`として
vehicle本体のmainコンポーネントへ直接コンパイルする。この ディレクトリ単体を`idf.py build`
することはできない。

## 3. 使い方

SILS（実機を使わないPC上の検証）で動作を確認する:

```bash
sf app sils smc_rate
```

既存のACRO/姿勢系シナリオでPIDベースラインと直接比較する（同じ`.expect`ゲートが使える）:

```bash
sf app sils smc_rate simulator/sils/scenarios/<scenario>.scn
sf sils scenario simulator/sils/scenarios/<scenario>.scn   # PIDベースライン（appなし既定ターゲット）
```

ゲイン（境界層幅・到達則ゲイン）は`sf params`/`--param`でその場調整できる:

```bash
sf app sils smc_rate --param smc.roll.k=15 --param smc.roll.eta=90
```

実機向けにビルド・書き込み（**SILS検証・実機投入ゲートをクリアしてから**）:

```bash
sf app build smc_rate
sf app flash smc_rate -m
```

## 4. ファイル構成

| ファイル | 内容 |
|---------|------|
| `smc_rate.hpp` | `SlidingModeRate` struct（1軸分のスライディングモード則。`pid.hpp`の`PID` structと同形） |
| `app_controller.hpp`/`.cpp` | `IController`実装。`pid_.compute()`の`torque[0..2]`だけを3軸の`SlidingModeRate`で上書きする |

ゲイン（`smc.{roll,pitch,yaw}.{k,eta,phi}`、計9個）は`firmware/vehicle/components/sf_core/params.cpp`
に定義（既定vehicleビルドの挙動には影響しない追加のみの変更）。

## 5. 注意

| 項目 | 内容 |
|------|------|
| 委譲範囲 | 高度・位置・離着陸フェーズ・誘導・トリム学習・DOB・ヘディングホールドは全て`PidController`委譲のまま — `architecture.md`のINV-1（全フェーズが単一の姿勢+レートパイプラインを共有する）を壊さない |
| 実機投入 | SILS摂動族テスト（ゲインスイープ・`--motor-delay`/`--thrust-eff`等）をクリアし、ユーザーの明示的判断を得てから進める。development_roadmap.mdの層別検証（ACRO起点）を踏襲 |
| ゲインの位置づけ | `params.cpp`の初期値はSILSチューニングの「出発点」であり飛行検証済みではない |

---

<a id="english"></a>

## 1. Purpose

`AppController` implements `IController` and delegates `thrust`, altitude,
position, the takeoff/landing phase state machine, guidance, trim/hover-
thrust learning, and the DOB entirely to `PidController` — but replaces
**only the final roll/pitch/yaw rate-loop torque** with a first-order
sliding-surface + boundary-layer reaching law. See
[`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md)
for the full design rationale and codebase research behind it.

Control law (independent per axis):
```
s = rate_ref − measured angular rate    (rate_ref is PidController's OWN final rate target)
torque = I_axis · ( k·sat(s/φ) + η·s )   (sat() is the boundary layer, for chattering mitigation)
```

## 2. It is an embedded template

`type: embedded` (see `app.yaml`) — unlike the standalone examples 09/10, it
has no `CMakeLists.txt` and no `main/`. The `*.cpp`/`*.hpp` files here are
compiled directly into the vehicle body's main component by the `sf app`
command via `SF_APP_DIR`. You cannot `idf.py build` this directory on its own.

## 3. Usage

Verify it in SILS (PC-side simulation, no hardware needed):

```bash
sf app sils smc_rate
```

Compare directly against the PID baseline using an existing ACRO/attitude
scenario (the same `.expect` gates apply to both):

```bash
sf app sils smc_rate simulator/sils/scenarios/<scenario>.scn
sf sils scenario simulator/sils/scenarios/<scenario>.scn   # PID baseline (default target, no app)
```

Tune the gains (boundary-layer width, reaching-law gains) live via `sf
params`/`--param`:

```bash
sf app sils smc_rate --param smc.roll.k=15 --param smc.roll.eta=90
```

Build/flash for real hardware (**only after SILS validation and the
hardware-flight gate in the plan doc**):

```bash
sf app build smc_rate
sf app flash smc_rate -m
```

## 4. Files

| File | Contents |
|------|----------|
| `smc_rate.hpp` | `SlidingModeRate` struct (one axis' sliding-mode law, same shape as `pid.hpp`'s `PID` struct) |
| `app_controller.hpp`/`.cpp` | `IController` implementation. Overwrites only `pid_.compute()`'s `torque[0..2]` with 3 axes of `SlidingModeRate` |

The gains (`smc.{roll,pitch,yaw}.{k,eta,phi}`, 9 total) live in
`firmware/vehicle/components/sf_core/params.cpp` (an additive-only change
with no effect on the default vehicle build's behavior).

## 5. Notes

| Item | Detail |
|------|--------|
| Delegation scope | Altitude, position, takeoff/landing phases, guidance, trim learning, DOB, and heading hold all remain delegated to `PidController` — does not break `architecture.md`'s INV-1 (every phase shares the ONE attitude+rate pipeline) |
| Before real hardware | Clear the SILS perturbation-family sweep (gain sweeps, `--motor-delay`/`--thrust-eff`, etc.) and get explicit user sign-off first. Follows development_roadmap.md's layered validation (starting at ACRO) |
| Gain status | The seed values in `params.cpp` are SILS-tuning starting points, not flight-validated gains |
