# smc_rate_sta — レートループのスーパーツイスティング法(STA)実験

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

Tier: L1 (Topic API) / Type: embedded / 由来: `firmware/apps/smc_rate`

## 1. 目的

`firmware/apps/smc_rate`（1次PI型面SMC）で未解決だった`motor-delay=15ms`条件の
`tilt_max` FAIL（`docs/plans/smc-rate-loop-plan.md` §3.3以降、PID・1次SMCともほぼ同じ
天井[23.65°/24.06°]にぶつかる）に対し、**2次スライディングモード（スーパーツイスティング法、
STA）**を試す実験app。`AppController`の構造は`smc_rate`と同一（`PidController`へ全委譲、
最終レートループのトルクだけ差し替え）で、差し替え則だけが異なる。

制御則（各軸独立）:
```
s = rate_ref − 測定角速度 + λ_i·∫(誤差)dt
u1 = k1·√|s|·sign(s)          （連続な項、s=0で連続的に消える）
z  = ∫ -k2·sign(s) dt          （積分項）
torque = I_axis · (u1 + z)
```

設計根拠・1次SMCとの違いの仮説は[`smc_rate_sta.hpp`](smc_rate_sta.hpp)、
検証結果は[`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md) §7.11参照。

## 2. 組み込み型であること

`type: embedded`（`app.yaml`参照）— `smc_rate`と同じく単独ビルド不可。

## 3. 使い方

```bash
sf app sils smc_rate_sta
sf sils scenario simulator/sils/scenarios/stab_flight.scn --target apps/smc_rate_sta --motor-delay 15
sf app sils smc_rate_sta --param smc_sta.roll.k1=80 --param smc_sta.roll.k2=40
sf app build smc_rate_sta
sf app flash smc_rate_sta -m   # SILS検証・実機投入ゲートをクリアしてから
```

## 4. ファイル構成

| ファイル | 内容 |
|---------|------|
| `smc_rate_sta.hpp` | `SuperTwistingRate` struct（1軸分のSTA則） |
| `app_controller.hpp`/`.cpp` | `IController`実装。`smc_rate`と同一構造 |

ゲイン（`smc_sta.{roll,pitch,yaw}.{k1,k2,phi,lambda_i,e_reset}`、計15個）は
`firmware/vehicle/components/sf_core/params.cpp`に定義。

## 5. 注意

| 項目 | 内容 |
|------|------|
| 委譲範囲 | `smc_rate`と同一 — INV-1を壊さない |
| 実機投入 | 未実施。SILS摂動族テストとユーザーの明示的判断が前提 |
| ゲインの位置づけ | 初期値はSILSチューニングの出発点、飛行検証済みではない |

---

<a id="english"></a>

## 1. Purpose

An experiment app trying **second-order sliding mode (the Super-Twisting
Algorithm, STA)** against the `motor-delay=15ms` `tilt_max` FAIL that
`firmware/apps/smc_rate`'s first-order PI-surface SMC never resolved
(`docs/plans/smc-rate-loop-plan.md` section 3.3 onward -- both PID and the
first-order SMC hit nearly the same ceiling, 23.65/24.06deg). `AppController`
has the identical structure to `smc_rate` (fully delegates to
`PidController`, replaces only the final rate-loop torque) -- only the
replacement law differs.

Control law (independent per axis):
```
s = rate_ref − measured angular rate + lambda_i * integral(error dt)
u1 = k1 * sqrt(|s|) * sign(s)   (continuous term, vanishes continuously at s=0)
z  = integral of -k2*sign(s) dt (integral term)
torque = I_axis * (u1 + z)
```

See [`smc_rate_sta.hpp`](smc_rate_sta.hpp) for the design rationale and the
hypothesis for why this might differ from the first-order design, and
[`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md)
section 7.11 for results.

## 2. It is an embedded template

`type: embedded` (see `app.yaml`) -- not independently buildable, same as `smc_rate`.

## 3. Usage

```bash
sf app sils smc_rate_sta
sf sils scenario simulator/sils/scenarios/stab_flight.scn --target apps/smc_rate_sta --motor-delay 15
sf app sils smc_rate_sta --param smc_sta.roll.k1=80 --param smc_sta.roll.k2=40
sf app build smc_rate_sta
sf app flash smc_rate_sta -m   # only after SILS validation and the hardware-flight gate
```

## 4. Files

| File | Contents |
|------|----------|
| `smc_rate_sta.hpp` | `SuperTwistingRate` struct (one axis' STA law) |
| `app_controller.hpp`/`.cpp` | `IController` implementation, identical structure to `smc_rate` |

Gains (`smc_sta.{roll,pitch,yaw}.{k1,k2,phi,lambda_i,e_reset}`, 15 total)
live in `firmware/vehicle/components/sf_core/params.cpp`.

## 5. Notes

| Item | Detail |
|------|--------|
| Delegation scope | Identical to `smc_rate` -- does not break INV-1 |
| Before real hardware | Not yet attempted. Requires the SILS perturbation-family sweep and explicit user sign-off |
| Gain status | Seed values are SILS-tuning starting points, not flight-validated |
