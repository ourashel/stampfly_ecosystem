# smc_rate_asta — レートループの適応スーパーツイスティング法（Adaptive STA）実験

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

Tier: L1 (Topic API) / Type: embedded / 由来: `firmware/apps/smc_rate_sta`

## 1. 目的

`firmware/apps/smc_rate_sta`（2次スライディングモード、STA、固定ゲイン）のチューニング過程
（`docs/plans/smc-rate-loop-plan.md` §3.7）で見つかった**非単調なトレードオフ**——
到達則ゲインを上げると`torque-authority=0.4/0.55`（モータ効き低下）への頑健性は改善するが、
同じ方向にゲインを上げすぎると`motor-delay=15ms`（別の不確かさ）で機体が転倒する——に対し、
**適応スイッチングゲイン**を試す実験app。単一の固定ゲインは複数の不確かさが矛盾する要求を
課すと両立できないが、スライディング変数`s`が収束できていないときだけゲインを増やし、
収束できているときは緩やかに減らす適応則は、事前の最悪値見積りなしにこのトレードオフを
回避しうる（Plestan et al. 2010 [R7]）。

**`AppController`の構造は`smc_rate_sta`と同一**（`PidController`へ全委譲、最終レートループの
トルクだけ差し替え）——差し替え則の`k1`（スイッチングゲイン）だけがオンラインで適応する点が異なる。

**`smc_rate_sta`自体はユーザーの明示的指示（2026-09-12）により固定ゲイン版の最終バージョンとして
変更しない**。本appは別・独立の実験。

制御則（各軸独立）:
```
s = rate_ref − 測定角速度 + λ_i·∫(誤差)dt
u1 = k1·√|s|·sign(s)          （連続な項、s=0で連続的に消える。k1は下記の適応則で時変）
z  = ∫ -k2·sign(s) dt          （積分項、k2 = k2_ratio·k1でk1に連動）
torque = I_axis · (u1 + z)

適応則（k1、Plestan型[R7]）:
  |s| > dead_band なら:  k1_dot = +adapt_rate                （未収束 → ゲイン増加）
  それ以外なら:          k1_dot = -adapt_rate · leak_ratio    （収束済 → 緩やかに減少）
  k1 = clamp(k1, k1_min, k1_max)
```

設計根拠は[`smc_rate_asta.hpp`](smc_rate_asta.hpp)、検証結果は
[`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md) §7.31参照。

## 2. 組み込み型であること

`type: embedded`（`app.yaml`参照）— `smc_rate_sta`と同じく単独ビルド不可。

## 3. 使い方

```bash
sf app sils smc_rate_asta
sf sils scenario simulator/sils/scenarios/stab_flight.scn --target apps/smc_rate_asta --motor-delay 15
sf app sils smc_rate_asta --param smc_asta.roll.k1_max=200
sf app build smc_rate_asta
sf app flash smc_rate_asta -m   # SILS検証・実機投入ゲートをクリアしてから
```

## 4. ファイル構成

| ファイル | 内容 |
|---------|------|
| `smc_rate_asta.hpp` | `AdaptiveSuperTwistingRate` struct（1軸分の適応STA則） |
| `app_controller.hpp`/`.cpp` | `IController`実装。`smc_rate_sta`と同一構造 |

ゲイン（`smc_asta.{roll,pitch,yaw}.{k1_init,k1_min,k1_max,k2_ratio,adapt_rate,
leak_ratio,dead_band,phi,lambda_i,e_reset,z_leak_tau}`、計33個）は
`firmware/vehicle/components/sf_core/params.cpp`に定義。`smc_sta.*`（固定ゲイン版）とは
独立したキー空間。

## 5. 注意

| 項目 | 内容 |
|------|------|
| 委譲範囲 | `smc_rate_sta`と同一 — INV-1を壊さない |
| `smc_rate_sta`との関係 | `smc_rate_sta`は無変更（最終版）。本appは別・独立の実験 |
| ゲインの位置づけ | 初期値は`smc_rate_sta`の調整済み値からのシード、適応則自体のパラメータ（adapt_rate/leak_ratio/dead_band/k1_min/k1_max）は未検証——SILSチューニングの出発点 |
| 実機投入 | 未実施。SILS摂動族テスト（§3.7と同一条件）とユーザーの明示的判断が前提 |

---

<a id="english"></a>

## 1. Purpose

An experiment app trying an **adaptive switching gain** against the
**non-monotonic trade-off** found while tuning `firmware/apps/smc_rate_sta`
(2nd-order sliding mode, STA, fixed gains) -- `docs/plans/smc-rate-loop-
plan.md` section 3.7: raising the reaching-law gain improved robustness to
`torque-authority=0.4/0.55` (reduced motor authority) but the same increase
caused a NEW tumble under `motor-delay=15ms` (a different uncertainty). A
single fixed gain cannot satisfy both when their required gains conflict; an
adaptive law -- growing only while the sliding variable `s` is not actually
converging, decaying slowly otherwise -- can in principle track this moving
target without needing an a priori worst-case bound (Plestan et al. 2010
[R7]).

`AppController` has the **identical structure to `smc_rate_sta`** (fully
delegates to `PidController`, replaces only the final rate-loop torque) --
only the replacement law's `k1` (switching gain) now adapts online.

**`smc_rate_sta` itself is UNCHANGED (the final fixed-gain version, per
explicit user instruction 2026-09-12)**. This app is a separate, independent
experiment.

Control law (independent per axis):
```
s = rate_ref − measured angular rate + lambda_i * integral(error dt)
u1 = k1 * sqrt(|s|) * sign(s)   (continuous term, vanishes at s=0; k1 is time-varying, see below)
z  = integral of -k2*sign(s) dt (integral term; k2 = k2_ratio*k1, tied to k1)
torque = I_axis * (u1 + z)

Adaptive law (k1, Plestan-type [R7]):
  if |s| > dead_band:  k1_dot = +adapt_rate               (not converged -> grow)
  else:                k1_dot = -adapt_rate * leak_ratio  (converged -> decay slowly)
  k1 = clamp(k1, k1_min, k1_max)
```

See [`smc_rate_asta.hpp`](smc_rate_asta.hpp) for the design rationale and
[`docs/plans/smc-rate-loop-plan.md`](../../../docs/plans/smc-rate-loop-plan.md)
section 7.31 for results.

## 2. It is an embedded template

`type: embedded` (see `app.yaml`) -- not independently buildable, same as `smc_rate_sta`.

## 3. Usage

```bash
sf app sils smc_rate_asta
sf sils scenario simulator/sils/scenarios/stab_flight.scn --target apps/smc_rate_asta --motor-delay 15
sf app sils smc_rate_asta --param smc_asta.roll.k1_max=200
sf app build smc_rate_asta
sf app flash smc_rate_asta -m   # only after SILS validation and the hardware-flight gate
```

## 4. Files

| File | Contents |
|------|----------|
| `smc_rate_asta.hpp` | `AdaptiveSuperTwistingRate` struct (one axis' adaptive STA law) |
| `app_controller.hpp`/`.cpp` | `IController` implementation, identical structure to `smc_rate_sta` |

Gains (`smc_asta.{roll,pitch,yaw}.{k1_init,k1_min,k1_max,k2_ratio,adapt_rate,
leak_ratio,dead_band,phi,lambda_i,e_reset,z_leak_tau}`, 33 total) live in
`firmware/vehicle/components/sf_core/params.cpp` -- an independent key space
from `smc_sta.*` (the fixed-gain version).

## 5. Notes

| Item | Detail |
|------|--------|
| Delegation scope | Identical to `smc_rate_sta` -- does not break INV-1 |
| Relation to `smc_rate_sta` | `smc_rate_sta` is UNCHANGED (final version). This app is a separate, independent experiment |
| Gain status | Initial values seeded from `smc_rate_sta`'s tuned gains; the adaptive law's own parameters (adapt_rate/leak_ratio/dead_band/k1_min/k1_max) are unverified -- SILS-tuning starting points |
| Before real hardware | Not yet attempted. Requires the SILS perturbation-family sweep (same conditions as section 3.7) and explicit user sign-off |

## 参考文献 / References

| ID | 文献 |
|----|------|
| R5 | A. Levant, "Sliding order and sliding accuracy in sliding mode control," *International Journal of Control*, vol. 58, no. 6, pp. 1247–1263, 1993. |
| R6 | J. A. Moreno and M. Osorio, "Strict Lyapunov functions for the super-twisting algorithm," *IEEE Trans. Automatic Control*, vol. 57, no. 4, pp. 1035–1040, 2012. |
| R7 | F. Plestan, Y. Shtessel, V. Bregeault, and A. Poznyak, "New methodologies for adaptive sliding mode control," *International Journal of Control*, vol. 83, no. 9, pp. 1907–1919, 2010. |
