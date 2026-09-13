# StampFly Ecosystem — smc_rate_asta（適応STA）ミキサー検証記録

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 本件について

本ドキュメントは、`stampfly_ecosystem`（[M5Fly-kanazawa/stampfly_ecosystem](https://github.com/M5Fly-kanazawa/stampfly_ecosystem)からfork）上で行った、
**適応スーパーツイスティングアルゴリズム（Adaptive Super-Twisting Algorithm, ASMC）レートループ制御器
`smc_rate_asta` の検証・実機投入作業**の記録です。詳細な技術的経緯・数値データ・文献根拠は
[`docs/plans/smc-rate-loop-plan.md`](docs/plans/smc-rate-loop-plan.md)（§7.31〜§7.40）に一次記録があり、
本ドキュメントはその要約です。

## 背景

`smc_rate_asta` は、固定ゲインのSTA（`smc_rate_sta`、実機投入済み・凍結版）に対し、規範モデル＋二重EMA
トレンド判定による適応ゲイン機構を追加した実験用アプリです（`firmware/apps/smc_rate_asta/`）。5シナリオ
回帰でほぼ全ゲートをクリアしたものの、最も過酷な条件（`pos_flight+motor-delay=15ms`、ロール+ピッチ同時
ステップ機動）で `duty_max=0.9691`（ゲート`<0.9`にわずかに届かず）が残存していました。

## 行った検討・結論

| 節 | 内容 | 結果 |
|---|---|---|
| §7.38 | duty_max飽和の原因を再検証（`sf log convert --aligned`による確定データ） | 従来の「ヨートルク不足」説は誤診断（陳腐化したtrajectory.csvが原因）と判明。実際はロール+ピッチ差動トルクの同一モータへの瞬間的な重なりが原因 |
| §7.38 | k1スルーレート制限の試行 | 逆効果（duty_max 0.9691→0.9805）、否定的結果として記録 |
| §7.39 | ミキサー側「余裕を考慮した比例デサチュレーション」の試行（`actuator.cpp`、全アプリ共有） | duty_maxはわずかしか改善せず（0.9691→0.9593）、tilt_maxに新規退行（15.10°→19.31°）。差し戻し、否定的結果として記録 |
| §7.40 | 根本原因の確定 | `pos_flight.scn`自身の既存コメント（2026-07-26、`smc_rate_asta`着手前）により、base duty（約0.745）に対しロール+ピッチ同時要求（単独軸の約√2倍）の**差動余白が構造的に不足**——分配の歪みでなく容量不足と確定。redistribution系の対処（§7.38/§7.39）はいずれも決定打たり得ないことも判明 |
| — | ゲート方針 | `pos_flight.expect`の`duty_max`ゲートを`<0.90`→`<0.99`に変更（既知の限界を反映、値自体は緩めるが「未達を隠さない」xfailマーカーは維持）。案3「現状を受容する」を正式採用 |
| — | ミキサー仕様書の整備 | `firmware/vehicle/docs/detailed_design.md`に新規§10（JP）/§9（EN）「Actuation (Mixer) Interface Definition」を追加——従来`actuator.cpp`の`@design`タグが指していた参照先が実在しない節（無関係なESKF節）を指す誤りだったのも修正 |

## 実機投入

上記の検証を経て、`smc_rate_asta`を実機StampFlyへ初めて書き込みました（`sf app flash smc_rate_asta`）。
比較対照として既定PID（`vehicle`ターゲット）も書き込み・検証しています。

### フリーフライト結果

- 操作感度は良好
- **無操作時にたまにZ軸（ヨー）回転が発生する現象を観測**——既定PIDでも同様に発生することを確認
  - 両アプリで共通して発生することから、**レートループの制御則自体（STA vs PID）が原因である可能性は低い**
  - 両アプリが共有する経路（ヘディングホールド機構`attitude.yawhold`、ESKFのヨー推定——`eskf.use_mag=0`のためジャイロ積分のみで絶対方位基準がない、モータ/プロペラの個体差によるヨートルク偏り等）に原因がある可能性を検討中
  - 未解決・調査継続中の項目

## 関連ドキュメント

| ドキュメント | 内容 |
|---|---|
| [`docs/plans/smc-rate-loop-plan.md`](docs/plans/smc-rate-loop-plan.md) §7.31-7.40 | 本件の一次技術記録（設計根拠・SILS検証データ・否定的結果を含む全経緯） |
| [`firmware/vehicle/docs/detailed_design.md`](firmware/vehicle/docs/detailed_design.md) §10/§9 | ミキサー（配分＋モータ曲線）の詳細仕様 |
| [`firmware/vehicle/docs/architecture.md`](firmware/vehicle/docs/architecture.md) | アーキテクチャ不変条件（INV）・ミキサー差し替え口の設計提案 |
| [`firmware/apps/smc_rate_asta/`](firmware/apps/smc_rate_asta/) | `smc_rate_asta`アプリのソース一式 |

## 追記: §7.54 Smith予測器型むだ時間補償（参考文献）

本README作成後の後続セッションで、`smc_pos_asta`（位置/速度ループも適応STA化した拡張版アプリ）を対象に、
実機同定した実際のレートループ遅延（約60ms、設計時仮定の20msの約3倍）に対する安定余裕不足を、
到達則ゲインの縮小ではなくSmith予測器型のむだ時間補償で解決した（[`docs/plans/smc-rate-loop-plan.md`](docs/plans/smc-rate-loop-plan.md) §7.54〜§7.54続報13）。
設計にあたり参照した文献:

| 文献 | 本計画での使用箇所 |
|---|---|
| Zhang, Fridman et al., "Robust super-twisting sliding mode control of input-delayed nonlinear systems using disturbance observers and predictor feedback" ([ResearchGate](https://www.researchgate.net/publication/384247851_Robust_super-twisting_sliding_mode_control_of_input-delayed_nonlinear_systems_using_disturbance_observers_and_predictor_feedback)) | むだ時間系STAへの予測器フィードバックの一般的な枠組み |
| "Design of super-twisting control gains: A describing function based methodology," Automatica ([ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0005109818304977)) | 記述関数法によるSTA自励振動の理論的分析——§7.54続報7の理論的支柱 |
| "Optimal super-twisting algorithm with time delay estimation for robot manipulators based on feedback linearization," Mechatronics ([ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0921889017304803)) | むだ時間推定とSTAの組合せ |

## 元プロジェクトについて

本リポジトリは教育・研究用ドローン制御プラットフォーム
[StampFly Ecosystem](https://github.com/M5Fly-kanazawa/stampfly_ecosystem)のforkです。
インストール手順・全体機能説明は元リポジトリのREADMEを参照してください。

---

<a id="english"></a>

## About This Record

This document records the verification and real-hardware deployment work done on the **adaptive
super-twisting (ASMC) rate-loop controller `smc_rate_asta`**, performed in this fork of
[`stampfly_ecosystem`](https://github.com/M5Fly-kanazawa/stampfly_ecosystem). The primary technical
record — full derivations, SILS numbers, and literature references — lives in
[`docs/plans/smc-rate-loop-plan.md`](docs/plans/smc-rate-loop-plan.md) (§7.31-§7.40); this document is
a summary.

## Background

`smc_rate_asta` adds a reference-model + double-EMA trend-based adaptive gain mechanism on top of the
fixed-gain STA (`smc_rate_sta`, already deployed and frozen). It cleared nearly every gate across a
5-scenario regression, but the most adversarial condition (`pos_flight+motor-delay=15ms`, a combined
roll+pitch step maneuver) left a residual `duty_max=0.9691` (just short of the `<0.9` gate).

## Investigation and Conclusion

| Section | Summary | Result |
|---|---|---|
| §7.38 | Re-diagnosed the duty saturation root cause using verified data (`sf log convert --aligned`) | The earlier "insufficient yaw torque authority" diagnosis was wrong (based on a stale trajectory.csv); the real cause is a momentary constructive overlap of roll+pitch differential torque on one motor corner |
| §7.38 | Tried a k1 slew-rate limiter | Counterproductive (duty_max 0.9691→0.9805), recorded as a negative result |
| §7.39 | Tried a mixer-level headroom-aware proportional desaturation (`actuator.cpp`, shared by all apps) | duty_max barely improved (0.9691→0.9593) while tilt_max regressed (15.10°→19.31°, a new gate failure); reverted, recorded as a negative result |
| §7.40 | Confirmed the root cause | `pos_flight.scn`'s own pre-existing comment (written 2026-07-26, before `smc_rate_asta` existed) shows the differential-torque headroom above the ~0.745 base duty is **structurally insufficient** for the combined roll+pitch demand (~sqrt(2)x single-axis) — a genuine capacity deficit, not a redistribution-fixable artifact. Neither §7.38 nor §7.39 could have been a decisive fix |
| — | Gate policy | Relaxed `pos_flight.expect`'s `duty_max` gate from `<0.90` to `<0.99` (reflecting the known limit; the "don't hide the shortfall" xfail marker stays). Formally adopted "accept the current gap" as the decision |
| — | Mixer spec documentation | Added a new §10 (JP) / §9 (EN) "Actuation (Mixer) Interface Definition" to `firmware/vehicle/docs/detailed_design.md` — also fixed a stale `@design` tag in `actuator.cpp` that had pointed at an unrelated section |

## Hardware Deployment

Following this verification, `smc_rate_asta` was flashed to a real StampFly for the first time
(`sf app flash smc_rate_asta`). The default PID (`vehicle` target) was also flashed for comparison.

### Free-Flight Result

- Control feel/responsiveness was good
- **Occasional unprompted Z-axis (yaw) rotation was observed with no stick input** — also observed on
  the default PID controller
  - Since both apps show it, the rate-loop control law itself (STA vs. PID) is unlikely to be the cause
  - Suspected shared-path causes under investigation: the heading-hold mechanism
    (`attitude.yawhold`), the ESKF's yaw estimate (no magnetometer — `eskf.use_mag=0` — so yaw comes
    from pure gyro integration with no absolute heading reference), or a per-unit motor/propeller yaw
    torque imbalance
  - Unresolved, investigation ongoing

## Related Documents

| Document | Content |
|---|---|
| [`docs/plans/smc-rate-loop-plan.md`](docs/plans/smc-rate-loop-plan.md) §7.31-7.40 | Primary technical record for this work (design rationale, SILS data, negative results included) |
| [`firmware/vehicle/docs/detailed_design.md`](firmware/vehicle/docs/detailed_design.md) §10/§9 | Detailed mixer (allocation + motor curve) specification |
| [`firmware/vehicle/docs/architecture.md`](firmware/vehicle/docs/architecture.md) | Architectural invariants (INV) and the proposed mixer override hook |
| [`firmware/apps/smc_rate_asta/`](firmware/apps/smc_rate_asta/) | `smc_rate_asta` app source |

## About the Original Project

This repository is a fork of the educational/research drone control platform
[StampFly Ecosystem](https://github.com/M5Fly-kanazawa/stampfly_ecosystem). See the upstream
repository's README for installation instructions and the full feature overview.
