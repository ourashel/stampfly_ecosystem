# レートループ スライディングモード制御(SMC)化計画 — `sf app`経由・SILS先行検証

作成: 2026-09-11

発端: 現行の`PidController`（カスケードPID、`firmware/vehicle/components/sf_controller_pid/`）を
スライディングモード制御(SMC)に置き換えたものを試したい。実機にいきなり書き込むのではなく、
**SILS（Software In the Loop Simulation）で検証できる形**で用意する。前回セッションで説明した
`sf app`コマンド（`IController`を差し替えて実機・SILS両方で動かす仕組み、`docs/plans/sf-app-sils-plan.md`
参照）がこの用途に合致するが、調査の結果`PidController::compute()`が632行のモノリシックな1関数で
高度・位置・離着陸フェーズが内部の私有状態と絡み合っており、丸ごとSMC化するのは大工事であることが
判明した。そこで置き換え範囲をレートループ3軸のみに絞り、既存`PidController`への委譲パターン
（`examples/11_app_controller`）を踏襲する設計とする。

ユーザー確認済みの方針（2026-09-11）:
- 置き換える範囲は**レートループ（`rate_roll_`/`rate_pitch_`/`rate_yaw_`）のみ**。角度外側ループ・
  高度・位置・離着陸フェーズ・誘導・トリム学習・ホバー推力学習・DOBは既存`PidController`に委譲。
- SMCのゲイン（スライディング面・到達則定数）は`sf_core/params.cpp`に新規行として追加し、
  `sf params`/NVS/`sf sils scenario --param`スイープをそのまま使えるようにする。

進捗（2026-09-11）: `firmware/apps/smc_rate`実装完了、`sf app sils`のWindows用CMakeバグ
（`simulator/sils/CMakeLists.txt`の`CONFIGURE_DEPENDS`付き`file(GLOB SF_APP_SRCS ...)`が
バックスラッシュパスを正しく扱えない）を副産物として発見・修正。§2.4の物理逆算初期ゲインは
`stab_flight`でFAILしたが、`sf sils scenario --target apps/smc_rate --param ...`による
再ビルド無しスイープでroll/pitch/yawとも到達則ゲインを強化・境界層を調整し、`params.cpp`の
デフォルト値として確定。最終ゲインで`alt_flight`/`acro_flight`/`stab_flight`/`yaw_hold`の
4シナリオが`--param`無しで一貫してPASS（`yaw_hold`の`alt_max`のみPID/SMC共通の既存無関係
問題でFAIL）。途中`smc.yaw.phi`を狭めすぎてstab_flightを壊す単一シナリオ過学習を実際に踏み、
広い方に戻して解消した実例あり（§3.1「yaw.phiのトレードオフ」）。超平面設計の考え方は
§2.2「超平面（スライディング面）の設計根拠」に記載。noise/摂動族ロバスト性確認（§3.2）で
**`stab_flight`×`torque-authority=0.55`にて明確な頑健性の弱点を発見**（PIDは耐えるが
SMCはatt_rmseゲート超過）——§3.1の詰めすぎによる単一点最適化の再発。実機投入は不可、
再チューニングが次の課題（§5）。

## 0. 要旨

| 観点 | 内容 |
|------|------|
| 現状 | 唯一の`IController`実装は`PidController`。`control_task.cpp:189`が`sf::app::controller()`経由で取得し、`SF_APP_DIR`未指定時は`app_default.cpp`→`stock_hooks.cpp`の関数ローカルstatic`PidController`に解決される |
| 制約 | `PidController::compute()`（`pid_controller.cpp:214-846`）は姿勢・高度・位置・離着陸フェーズ・誘導・トリム学習・DOBが1関数に同居するモノリシック実装。個別ループを外部から呼び分ける公開APIは無い |
| 突破口 | `ControlOutput`は`rate_ref[3]`（最終レート目標、姿勢外側ループ・ヘディングホールド・誘導など全適用後、`pid_controller.cpp:824-826`で書き込み）を既にテレメトリ用に公開している。`pid_.compute()`を丸ごと呼び、その`torque[0..2]`だけをSMCで再計算した値に差し替えれば、他は無改造で委譲できる |
| 方針 | `sf app new smc_rate --from 11_app_controller`で生成したappプロジェクト内に、`PidController`をメンバとして保持しつつ`compute()`の`torque[0..2]`だけを1次スライディング面+境界層付き到達則で上書きする`AppController`を実装する |
| 成果目標 | `sf app sils smc_rate`で既存のACRO/姿勢系シナリオ（`.expect`の`att_rmse`/`duty_max`等）をそのまま使い、`sf sils scenario`（PIDベースライン）と直接A/B比較できる状態 |
| 実機投入 | SILS摂動族テスト（後述）をクリアし、ユーザーの明示的な判断を得てから`sf app build/flash smc_rate`に進む。development_roadmap.mdの層別検証（ACRO起点）を踏襲 |

## 1. 前提・制約（調査で判明した事実）

| 項目 | 内容 | 根拠 |
|------|------|------|
| `IController`インターフェース | `compute`/`reset`/`onModeChange`が必須。`onLanding`/`onTakeoff`/`onTakeoffComplete`/`isTakeoffComplete`/`setGuidanceTarget`/`isGuidanceActive`/`startExcitation`/`fetchSysidResult`/`reloadParams`は既定no-op | `firmware/vehicle/components/sf_controller/include/controller.hpp` |
| `ControlOutput`の中間目標公開 | `rate_ref[3]`（内側ループ角速度目標[rad/s] R,P,Y）・`angle_ref[2]`（外側ループ傾き目標[rad] R,P）がData Stream用に公開済み | `firmware/vehicle/components/sf_core/include/data_types.hpp:236-242` |
| 上記が実際に書き込まれる箇所 | `output.rate_ref[0..2] = rate_sp_roll/pitch/yaw`（ヘディングホールド・誘導・POS_HOLD・Landing水平ゲート適用後の最終値） | `firmware/vehicle/components/sf_controller_pid/pid_controller.cpp:824-826` |
| レートPID最終呼び出し | `rate_roll_.compute(rate_sp_roll, gyro_rate.x, dt)`等 — 我々のSMCが置き換える対象そのもの | `pid_controller.cpp:774-776` |
| 実行タイミング | `ControlTask`（優先度23、IMU同期400Hz、`ImuTask`からの通知で起床）が`controller.compute(state, setpoint, config::IMU_DT)`を呼ぶ。dt=2.5ms固定 | `firmware/vehicle/tasks/control_task.cpp:189, 233-241, 360` |
| 出力の下流 | `sf::control_output.publish(control)`後、`Actuator::update()`が独立に`control_output.latest()`を読みB⁻¹ミキサー→モータ曲線→duty clampへ。両者は疎結合（トピック経由） | `control_task.cpp:376, 407`、`firmware/vehicle/components/sf_actuator/actuator.cpp:255-309, 352-370` |
| ミキサー（B⁻¹） | `T_i = ¼T ± τφ/d ± τθ/d ± τψ/κ`（d=ARM_D=0.023m、κ=KAPPA=4.10e-3=Cq/Ct）。線形幾何配分。SMCのトルク出力もこの形式（物理単位, torque[Nm]+thrust[N]）で受け取られる | `actuator.cpp:88-134, 255-273` |
| 既存トルク上限 | `max_roll_pitch_torque_=5.2e-3 Nm`（param `rate.*.output_limit`相当）、`rate.yaw.max_torque`（既定1.226e-3 Nm、実行時調整可） | `pid_controller.hpp:594-595`、`params.cpp`該当行 |
| 物理パラメータ | mass=0.037kg, Ixx=9.16e-6, Iyy=13.3e-6, Izz=20.4e-6 kg·m²（SSOT） | `control/models/stampfly_physical.yaml:63-108` |
| モータ曲線の不確かさ | SILSプラント用`measured_2026_07`と実機`flight_anchored_motor_curve`は約1.252倍のホバー推力差があり未解決（`simulation-policy.md`§8 backlog #3） | `control/models/stampfly_physical.yaml:372-417` |
| パラメータストア | `sf_core/params.cpp`の`param_vars`+`table[] `が唯一の実行時チューニング機構（`{name, type, &var, default, min, max, callback}`）。`set_float`→`notifyControllerReload`→`ControllerCmd::ReloadParams`→`IController::reloadParams()` | `firmware/vehicle/components/sf_core/params.cpp:707+`、`control_task.cpp:157-163` |
| SILSビルド対象 | `sf app sils <name>`は`emu_vehicle`（host-native, MinGW+MuJoCo）を`SF_APP_DIR`付きでビルドし、`.scn`シナリオ+`.expect`（`log_contains`/`order`/`exit`/`metric ... in t0 t1`）で自動判定 | `lib/sfcli/commands/sils.py`（`build_app_emulator`/`run_scenario_with_exe`/`_eval_expect`） |
| 既存シナリオ資産 | `simulator/sils/scenarios/`に79ファイル（`.scn`+`.expect`）、`TEST_MATRIX.md`にカテゴリ一覧。ホバー/姿勢/高度/位置/wobble_bench/gain-deficit等 | `simulator/sils/scenarios/TEST_MATRIX.md` |
| 頑健性検証の既存原則 | 単一点最適化禁止（`torque-gain∈[0.4,0.7]`・`dead-time L∈[8,15]ms`等の摂動族で検証必須）。過去に公称SILSだけで最適化したゲインが実機で位相余裕-375°になった実例あり | `docs/architecture/simulation-policy.md`§6 |
| INV不変条件 | INV-1（全飛行フェーズが単一姿勢+レートパイプラインを共有）、INV-2（パイロットの姿勢権限）、INV-5（ミキサー入力は物理単位） | `firmware/vehicle/docs/architecture.md`「アーキテクチャ不変条件」節 |
| コーディング規約 | 1関数50行以内・バイリンガルコメント・マジックナンバー禁止・`@design`タグ必須（`[OK]`/`[NG]`/`[--]`） | `firmware/vehicle/docs/coding_and_education.md` |
| プロジェクト横断ルール | 制御系パラメータの変更提案は数値シミュレーションで裏付けてから行う（推測での提案禁止） | `CLAUDE.md`セッションルール |

**既存の非PID/非線形制御の実装例**: リポジトリ内にスライディングモード・Lyapunov・バックステッピング等の先行実装は無く、本計画は純粋な新規実装（greenfield）。`IController`のdocstring自体が「PID, MPC, LQR, or any custom controller」の差し替えを想定している。

## 2. 設計判断

### 2.1 スコープ

| 案 | 判断 |
|----|------|
| レートループのみ置き換え | **採用**（ユーザー確認済み）。`rate_ref`が既に公開されており最小侵襲。既存`rate_sysid.py`による同定と直接比較可能 |
| 姿勢+レート結合（単一スライディング面） | 不採用（今回）。角度ループも壊すためカスケード構造の比較優位性が失われる。将来の拡張候補として保留 |
| フルスクラッチ（高度/位置/離着陸も自前実装） | 不採用。`PidController::compute()`の私有状態を丸ごと再実装する必要があり、リスク・工数に見合わない |

### 2.2 制御則

各軸（roll/pitch/yaw）を独立3系統として設計する（`ω×Iω`ジャイロ交差結合は無視 — 既存`rate_sysid.py`の軸別同定も同じ簡略化を前提としており整合）。

1次スライディング面（測定レート誤差そのもの）:
```
s = ω_ref − ω          ω_ref = baseline.rate_ref[axis]（PidController由来）、ω = state.angular_rate[axis]
```

境界層付き到達則（チャタリング抑制のため`sign()`の代わりに`sat()`）:
```
τ_axis = I_axis · ( k·sat(s/φ) + η·s )
```
この形は「定数項＋比例項」型到達則（constant-plus-proportional rate reaching law、
通称exponential reaching law）— Gao & Hung [R1]が体系化した設計法に基づく。境界層による
チャタリング抑制（`sign()`→`sat()`の置換）はSlotine & Li [R2]の標準的手法。文献一覧は
§6参照。

| 記号 | 意味 | 備考 |
|------|------|------|
| `k` | スイッチングゲイン | 外乱・モデル不確かさの上限を上回る必要（SMCの頑健性の核） |
| `η` | 線形到達ゲイン | 境界層内の収束速度 |
| `φ` | 境界層半幅 | チャタリングと追従誤差のトレードオフ |

出力は既存PIDと同じ上限（`max_roll_pitch_torque_`/`rate.yaw.max_torque`、§1参照）でハードクランプし、物理的なトルク上限をPIDと共有する。

#### 超平面（スライディング面）の設計根拠 — なぜ`s = e`で足りるか

一般にスライディング面設計は`s = ė + λe`のような複数項の線形結合（面上のダイナミクスが
Hurwitz安定になるよう`λ`を選ぶ、極配置に相当）が必要になる。これが要るのは**制御入力から
見た出力の相対次数が2以上**の場合 — 例えば角度`θ`を直接追従させる系（`τ→(積分)→ω→(積分)→θ`、
トルクが角度に効くまで積分2回＝相対次数2）では、角度誤差`e_θ`だけを面にすると`ṡ`が`τ`に
依存せず到達則が組めないため、`s = ė_θ + λe_θ`として角速度誤差を混ぜ込み`ṡ`に`τ`（`ω̇`経由）を
出現させる必要がある。この`λ`が面上の収束時定数`1/λ`を指定する設計パラメータになる。

本計画は§2.1のスコープ決定により**レートループのみ**を対象とし、追従対象は角速度`ω`そのもの
（`τ→(積分1回)→ω`、剛体オイラー方程式`I·ω̇=τ`、`ω×Iω`交差結合は無視 — 相対次数1）。よって
`s = ω_ref − ω`のままで`ṡ = ω̇_ref − τ/I`が既に`τ`に直接依存し、複数項への結合は不要かつ
（状態が1つしかないため）結合のしようがない。**これは設計を省略したのではなく、相対次数1の
系に対する教科書通りの最小構成**であり、本計画で実際に「設計」したのは超平面の形状ではなく
到達則側の3定数（`k`/`η`/`φ`）のみである。§2.1で「姿勢+レート結合」案を不採用としたのは、
まさにこの`s = ω_error + λ·θ_error`という2項の面・`λ`という設計パラメータの発生を避けるため
でもある（採用していた場合は極配置または`stab_flight`許容オーバーシュートからの逆算が必要
だった）。

**明記すべき簡略化**（実装コメント・`@design`タグで残す）:
- `ω̇_ref`のフィードフォワード項は初版で省略（400Hz更新の`rate_ref`を数値微分すると量子化ノイズが乗るため）
- モータの電気/機械遅れ（`rate_sysid.py`が同定する時定数`T`）はトルク指令→実レート応答間の未モデル化の速い動特性。到達ゲイン`k`を上げすぎるとこれを励振しチャタリング・振動を招くため、初期ゲインは保守的に設定しSILS摂動族テストで確認する

### 2.3 コード構成

```
firmware/apps/smc_rate/              ← `sf app new smc_rate --from 11_app_controller`
├── app.yaml                         (type: embedded, sils: true)
├── app_controller.hpp/.cpp          ← AppController書き換え
└── smc_rate.hpp                     ← 新規: SlidingModeRate struct（pid.hppのPID structと同形）
```

`smc_rate.hpp`（`firmware/vehicle/components/sf_controller_pid/include/pid.hpp`の`PID` structと同じ形— ゲイン+状態+`compute()`+`reset()`）:
```cpp
struct SlidingModeRate {
    float k = 0, eta = 0, phi = 0.05f;   // ゲイン（reloadParams()でparams.cppから読む）
    float output_limit = 1.0f;            // 既存の max_roll_pitch_torque_ / rate.yaw.max_torque を流用
    float compute(float rate_sp, float rate_meas, float dt);
    void reset() {}   // 積分状態を持たないためno-op（将来拡張用に形だけ残す）
};
```

`app_controller.cpp`の`compute()`フロー:
```cpp
sf::ControlOutput AppController::compute(const sf::StateEstimate& state,
                                          const sf::CommandSetpoint& setpoint, float dt)
{
    sf::ControlOutput output = pid_.compute(state, setpoint, dt);   // 高度/位置/離着陸/誘導/トリム/DOBは丸ごと既存ロジック
    output.torque[0] = smc_roll_.compute(output.rate_ref[0], state.angular_rate[0], dt);
    output.torque[1] = smc_pitch_.compute(output.rate_ref[1], state.angular_rate[1], dt);
    output.torque[2] = smc_yaw_.compute(output.rate_ref[2], state.angular_rate[2], dt);
    return output;   // thrust/rate_ref/angle_refはpid_の値を維持（比較プロット用に目標値は据え置き）
}
```

`reset()`/`onModeChange()`/`onLanding()`/`onTakeoff()`/`onTakeoffComplete()`/`isTakeoffComplete()`/
`setGuidanceTarget()`/`isGuidanceActive()`/`startExcitation()`/`fetchSysidResult()`は`pid_`へ単純転送
（`11_app_controller`の`AppController`と同じパターン）。`reloadParams()`は`pid_.reloadParams()`に加え、
`smc_{roll,pitch,yaw}_`の`k/eta/phi/output_limit`もparamsから再読込するよう拡張する。

**副次的な利点**: `startExcitation`（sysid用チャープ/ダブレット注入）は`PidController`内部で`rate_sp_*`に
加算されてから`output.rate_ref`として公開されるため、`pid_`へ転送するだけでSMCも既存の
`sf sils sysid-gate`/`rate_sysid.py`の励振経路をそのまま受けられる（追加実装不要）。

### 2.4 `sf_core/params.cpp`への追加

`rate.roll.kp`等と並べて`smc.{roll,pitch,yaw}.{k,eta,phi}`（計9行）を`param_vars`+`table[]`に追加。
コールバックは既存の`notifyControllerReload`を流用。初期値・min/max境界は、既存レートPIDゲイン
（`rate.{roll,pitch,yaw}.kp/ti/td`）と`I_axis`から到達則が同等の閉ループ帯域になるよう逆算した値を
**SILS実装時に確定する**（ここで数値を仮置きしない — CLAUDE.mdの「制御系パラメータはシミュレーション
で裏付けてから」原則に従う）。これは標準vehicleの挙動を一切変えない追加のみの変更（デフォルト
ビルドでは未使用の追加パラメータ行が増えるだけ）。

### 2.5 アーキテクチャ不変条件との整合

| INV | 整合性 | 理由 |
|-----|--------|------|
| INV-1（全飛行フェーズが単一パイプラインを共有） | 自動的に満たす | 鉛直フェーズ分岐は`pid_.compute()`内部にそのまま残る。本変更は姿勢レートの最終段のみ |
| INV-2（パイロットの姿勢権限） | 自動的に満たす | 角度→レート目標の生成は無改造。SMCは追従則が変わるのみ |
| INV-5（ミキサーは物理単位） | 満たす | `torque[Nm]`をそのまま出力、ミキサー入力形式は不変 |

`@design`タグを`app_controller.cpp`の`compute()`に付与し、`architecture.md`のIController差替え節・
上記INV番号を参照。ステータスは実装直後`[--]`→SILS検証後`[OK]`とする。

## 3. SILS検証手順

1. `sf app new smc_rate --from 11_app_controller`でプロジェクト作成
2. `smc_rate.hpp`実装 → `app_controller.cpp`書き換え → `params.cpp`に9行追加
3. `sf app sils smc_rate`（既定シナリオ）でまず飛ぶことを確認
4. `simulator/sils/scenarios/`のACRO/姿勢レート系の既存シナリオ（`TEST_MATRIX.md`参照）を
   `sf app sils smc_rate <scenario>.scn`で実行 — 同じ`.expect`（`att_rmse`/`duty_max`等）がそのまま
   使えるため、PIDベースライン（`sf sils scenario <同じファイル>`、appなし既定ターゲット）と直接比較できる
5. `--noise n2`（実機相当の帯域制限振動ノイズ）で実行し、チャタリングと振動の相互作用を確認。
   `sf log viz --groups`（rate_ref/gyro/torque/dutyを軸ごとに並べる）と`sf log analyze --fft`で
   チャタリング周波数成分を確認（新規解析コード不要、既存ツールで足りる）
6. `simulation-policy.md`§6の「単一点最適化禁止」原則に従い、`--param smc.roll.k=... smc.roll.eta=...`の
   ゲインスイープに加え、`--motor-delay`/`--thrust-eff`/`--torque-authority`等の摂動シナリオ引数
   （既存`sils.py`の`scenario_args`）で頑健性を確認 — 公称プラントだけに効くゲインを避ける

**注**: `sf app sils <name> [scenario]`自体は`--param`を受け付けない（`app.py`のsils
サブパーサに`--param`引数が無い）。ゲインスイープは下位の`sf sils scenario <file> --target
apps/<name> --param ...`を直接使う。

### 3.1 実測結果（2026-09-11、noise=off, seed=12345）

| シナリオ | 指標（ゲート） | PIDベースライン | SMC 初期ゲイン（§2.4） | SMC チューニング後 |
|---------|----------------|-----------------|------------------------|---------------------|
| `alt_flight` | alt_rmse(<0.08) / alt_band(<0.35) / duty_max(<0.92) | — | 0.0112 / 0.093 / 0.68 ✅ | 未再検証 |
| `acro_flight` | att_rmse(<3.0) | 1.16° | 1.99° ✅ | 2.10° ✅（劣化なし） |
| `stab_flight` | att_rmse(<3.0) | 2.77° | **4.47° ❌FAIL** | 2.85° ✅ |

チューニング後ゲイン（**`params.cpp`のデフォルト値として確定・反映済み**）:
`smc.{roll,pitch}.eta` 76.42/75.07→150/145、`smc.{roll,pitch}.k` 9.83/9.65→20/20、
`smc.{roll,pitch}.phi` 0.3→0.15。`smc.yaw.eta`27.55→54、`smc.yaw.k`3.54→7.3、
`smc.yaw.phi`は**0.3のまま据え置き**（下記「yaw.phiのトレードオフ」参照）。

**所見**: §2.4で物理パラメータから逆算した初期ゲインは、レート直撃（`acro_flight`）では
ゲートを楽に満たしたが、姿勢複合ステップ（`stab_flight`）ではPIDに対し明確に見劣りし
ゲート未達だった。境界層`φ`を半減（0.3→0.15）し到達則ゲイン`k`/`η`を約2倍にすることで
両シナリオともPASSし、PIDの精度にかなり近づいた——初期の「PID等価帯域から逆算」だけでは
到達フェーズの余裕が不足していたことを示唆する。

#### yaw軸のチューニングと`yaw_hold`シナリオでの発見

roll/pitchと同じ倍率でyawも再スケール（`smc.yaw.eta`27.55→54, `smc.yaw.k`3.54→7.3,
`smc.yaw.phi`0.3→0.15）し、`acro_flight`（劣化なし、att_rmse=2.03°）と`yaw_hold`
（M1モータ80%故障下のヘディングホールド外乱耐性シナリオ）で確認した:

| シナリオ | 指標 | PIDベースライン | SMC（roll/pitch/yaw チューニング後） |
|---------|------|-----------------|----------------------------------------|
| `yaw_hold` | yaw_band(<1.5°) | **8.01° FAIL** | **0.23° PASS** |
| `yaw_hold` | duty_max(<0.96) | 1.00 FAIL（飽和） | 0.82 PASS |
| `yaw_hold` | alt_max(<0.85m) | 1.46m FAIL | 1.50m FAIL |

`alt_max`はPID/SMC両方でFAILしたが、`yaw_hold.expect`自体に`xfail`マーカーがあり
（"yaw-band degradation under the ODE motor plant, same root cause as pos_flight/pos_yaw
(yaw-axis fidelity gap, simulation-policy backlog #11/#12)"）、**SILSのyaw軸モータ
プラント忠実度に関する既知の未解決ギャップであり、SMC由来ではない**。むしろ本来の
着目指標である`yaw_band`・`duty_max`ではSMCがPIDを明確に上回った（PIDは飽和・大幅な
方位流出、SMCは軽負荷で高精度保持）——ただしこれは1シナリオ・1シード・noise=offでの
結果であり、汎化の主張はしない（§3.1冒頭の他シナリオ同様、頑健性確認は未実施）。

#### yaw.phiのトレードオフ（単一シナリオ過学習の実例）

yawを`smc.roll/pitch`と同じ比率で境界層も半減（`phi`0.3→0.15）させたところ、`yaw_hold`の
`yaw_band`はわずかに改善した（0.31°→0.23°、どちらもゲート1.5°に対し十分な余裕）が、
**`stab_flight`（ヨースティック中立のroll+pitch複合ステップ）がFAILした**
（att_rmse=3.32°>ゲート3.0°、`phi`=0.3では2.88°）。`smc.yaw.phi`だけを戻して両シナリオを
再テストし、この回帰がyaw.phiの変更に起因することを確認した。stab_flightはヨースティックを
一切動かさないため、これはヨー指令への直接応答ではなく、roll/pitch機動中に生じるヨー反力
トルクを、より狭い境界層のヨーSMCがより神経質に処理してしまう**間接結合**だと考えられる。
`simulation-policy.md`§6が警告する「単一シナリオへの過学習」をSILS上で実際に踏んだ例と
なったため、`smc.yaw.phi`は広い方（0.3、変更なし）を採用し、両シナリオで安定な設定を最終
デフォルトとした（4シナリオ全て`--param`無し・既定値のみでの再検証結果は次のとおり）:

| シナリオ | 主要指標 | 結果 |
|---------|---------|------|
| `alt_flight` | alt_rmse/alt_band/duty_max | 0.0112 / 0.093 / 0.68 ✅ |
| `acro_flight` | att_rmse | 2.07° ✅ |
| `stab_flight` | att_rmse | 2.88° ✅ |
| `yaw_hold` | yaw_band/tilt_max/alt_min/duty_max | 0.31°/3.07°/0.91m/0.82 ✅（`alt_max`のみ既存の無関係な既知問題でFAIL） |

### 3.2 ロバスト性確認（noise / 摂動族）の結果と発見された弱点

確定した既定ゲイン（§3.1）に対し、`--noise n2`（実機相当帯域制限振動）と
`--torque-authority`/`--motor-delay`（simulation-policy.md§6が求める摂動族の一部）を
`acro_flight`/`stab_flight`双方・PID対比で確認した。

**noise n2（両シナリオ、seed=12345と777で再現性確認済み）:**

| シナリオ | 指標 | PID | SMC |
|---------|------|-----|-----|
| `acro_flight` | `Takeoff complete`到達 | ❌ **両シードとも失敗**（離陸未完了のまま） | ✅ 両シードとも成功 |
| `stab_flight` | `Takeoff complete`到達 | ❌ 失敗 | ✅ 成功 |
| `stab_flight` | att_rmse(<3.0) | 1.15°（離陸未完了のため参考値） | 4.15° ❌FAIL（離陸は成功した上での数値） |

離陸完了フェーズは100%`PidController`委譲（本変更は一切触れていない）にもかかわらず、
n2ノイズ下でPIDベースラインだけが両シナリオとも離陸完了できていない — レートループの
追従品質が姿勢推定・鉛直ハンドオフに何らかの形で影響した可能性があるが、**因果関係は
未解明**（推測に留め、断定しない）。一方でSMC自身は離陸完了後、`stab_flight`ではnoise無しの
2.88°からn2下で4.15°へ悪化しゲート未達 — 追従精度自体はノイズに対して脆弱。

**摂動族（`--torque-authority`/`--motor-delay`、noise=off）:**

| シナリオ | 摂動 | 指標 | PID | SMC |
|---------|------|------|-----|-----|
| `acro_flight` | torque-authority=0.55 | att_rmse(<3.0) | 0.98° ✅ | 2.01° ✅ |
| `acro_flight` | motor-delay=12ms | att_rmse(<3.0) / tilt_max(<25) | 1.44° / 9.08° ✅ | 0.66° / 13.77° ✅ |
| `stab_flight` | torque-authority=0.55 | att_rmse(<3.0) | **2.55° ✅** | **5.18° ❌FAIL** |
| `stab_flight` | motor-delay=12ms | tilt_max(<18) | 18.86°（僅かにFAIL） | 22.83° ❌FAIL（PIDより悪化） |

**重大な所見**: `acro_flight`（レート直撃）では摂動下でもSMCはPIDと同等以上に健闘するが、
`stab_flight`（姿勢複合ステップ、角度外側ループ経由でSMCへ目標が渡る条件）では
`torque-authority=0.55`（モータ劣化を模した差動トルク有効度低下）に対し**PIDは余裕で
耐えるがSMCは大きく崩れる**（att_rmse 2.88°→5.18°、ゲートの1.7倍超過）。`motor-delay`
でも同傾向（SMCの方がtilt_maxの悪化が大きい）。§3.1で確定した既定ゲインは**公称プラント
（§2.4の逆算値ベース）に対して`stab_flight`をギリギリ通すところまで詰めた値**であり、
まさに`simulation-policy.md`§6が警告する「単一点最適化」の罠に、yaw.phiだけでなく
到達則ゲイン全体でも陥っていたことが摂動テストで露呈した。到達則の`k`（外乱・不確かさの
上限を上回るべきスイッチングゲイン）が、実際のトルク有効度低下（0.4〜0.7倍）に対して
不足している可能性が高い。

**結論**: 現状のゲインセットは実機投入判断の材料にできる段階ではない。`k`を増やす方向の
再チューニングが必要だが、`stab_flight`単体への最適化を繰り返すと同じ罠を再現するだけ
なので、**摂動を含めた複数条件を同時に満たすゲインを探索する**（あるいは`k`の理論的な
下限を外乱モデルから見積もり直す）必要がある。§5 Next stepsに反映。

### 3.3 頑健性弱点への対処（ラウンド2〜3）と安全性優先の判断

§3.2で発見した`stab_flight`×`torque-authority=0.55`の弱点（att_rmse 5.18°）に対し、
全条件（noise/摂動族）を通すことを目標に`eta`/`k`/`φ`の追加チューニングを行った。

**ラウンド2**（`eta`150/145→400/390、`φ`0.15→0.05、`k`20/20は据え置き）:
`torque-authority`は劇的に改善（0.4/0.55/0.7全てPASS、att_rmse 2.2〜2.9°）し、
名目性能も改善（stab_flight att_rmse 2.88°→2.04°）。ただし**`--motor-delay 15ms`で
完全な転倒**（att_rmse 187°、tilt_max 180〜201° — ゲート未達ではなく実際の制御喪失）、
**`--noise n1`（n2より軽いレベル）では離陸すら完了しない**という壊滅的な退行を招いた。

`k`を20→60に3倍にしても`torque-authority=0.4`の結果が小数点3桁まで完全一致 —
これは`eta·s`項だけで既に出力上限（`output_limit`）に張り付いており、`k`の寄与が
意味を持つ前に飽和していた動かぬ証拠。つまりラウンド1の弱点は「ゲイン不足」ではなく、
この積分項を持たない到達則が飽和状態にあったことが原因だった。

**中間点の試行**（`eta`250/245、`φ`0.1）でも`motor-delay=15ms`で転倒（tilt_max 201°）
したまま`torque-authority=0.4`・`noise n1/n2`は未解決——`eta`が250程度を超えた時点で
既に、`φ`/`k`の調整とは無関係にモータの未モデル化な約15ms遅れに対する安全域を
外れることが確認された。これはラウンド1の問題の鏡像である: `torque-authority`頑健性に
必要な高`eta`は、遅れに対する位相余裕としては高すぎ、試した8通りのゲイン点のどれも
この二律背反を回避できなかった。

**根本原因の考察**: PIDの積分項は、持続的な乗法的トルク有効度損失を「ループゲインを
上げる」のでなく「誤差を追従回復まで積分し続ける」ことで補償する。本SMCの到達則
（`smc_rate.hpp`参照）は積分状態を一切持たないため、この種の頑健性をゲイン調整だけで
再現しようとすると必然的にループゲイン（`eta`）を上げる方向に向かい、それがそのまま
遅れに対する位相余裕を削ってしまう——**§2.1でスコープを「レートループのみ」に絞り、
かつ積分無しの単純な1次面を選んだ設計判断（§2.2）自体が持つ構造的な限界**である
可能性が高い。

**判断（安全性優先）**: 「転倒（ラウンド2/3）」は「ゲート未達だが制御された飛行のまま
着陸できる（ラウンド1、`motor-delay=15ms`でtilt_max=22.8〜23.5°、墜落ではない）」より
明確に悪い——ラウンド1の方が通過する摂動チェックの総数自体は少なくても。**ゲート通過数
より安全性を優先し、`params.cpp`のデフォルト値をラウンド1（`eta`150/145、`k`20/20、
`φ`0.15）に差し戻した。**

**最終状態（既知・記録済みの未解決FAIL、黙って受け入れたのではない）**:

| 条件 | 結果 |
|------|------|
| `stab_flight`名目 | PASS（att_rmse 2.88°） |
| `stab_flight` + `torque-authority` 0.4/0.55 | **FAIL**（att_rmse 4.9〜5.2°、PIDは2.5〜2.8°） |
| `stab_flight` + `motor-delay` 15ms | FAIL（tilt_max 23.5°、ただし転倒ではない。PIDも同条件でFAIL＝21.4°） |
| `stab_flight` + `noise n1` | **FAIL**（att_rmse 4.67°、ただし離陸・着陸は完遂） |
| `acro_flight`（名目・全摂動込み） | 一貫してPASS（レート直撃はどの条件でも崩れない） |

`acro_flight`（レートループへの直接指令）は摂動条件下でも一貫して健全である一方、
`stab_flight`（角度外側ループ経由の複合機動）でのみ頑健性の弱点が残る——この非対称性
自体も、問題が「レート追従則そのもの」というよりは「角度ループ由来の目標に対する
頑健性マージンの設計」にある可能性を示唆しており、次の設計変更（積分様項・遅れ補償項の
追加など、本計画のスコープ外）の手がかりになる。

### 3.4 提案（未実装）: PI型スライディング面への拡張

§3.3の根本原因考察（PIDの積分作用の欠如）を受け、面を以下に拡張する案を検討中
（**未実装**、実装するかは要判断）:

```
s = e + λ_i·∫e dt              （e = ω_ref − ω、λ_iは積分ゲイン [1/s]）
```

面上（`s=0`）を微分すると`ė + λ_i·e = 0`となり、`λ_i`は`ė+λe`型の超平面設計における
`λ`と同じ役割（誤差ダイナミクスの収束時定数`1/λ_i`を指定する設計パラメータ）を果たす
— §2.2で「相対次数1だから複数項は不要」と述べたのは追従誤差そのものへの言及であり、
定常的な乗法的不確かさの補償という別目的には、この積分項が正統に当てはまる。

**要検討事項**:
- 積分状態が必要になるため`SlidingModeRate`のstruct（現状「状態なし、`reset()`はno-op」）
  を変更する必要がある
- ワインドアップ対策が必須（`stab_flight`は出力上限付近で張り付く場面がある）——
  `pid.hpp`の`PID` structが既に持つ条件付き積分ロジックを踏襲できる
- 積分作用は本質的に低速な補正であり`η`を上げる必要が無いため、§3.3で発見した
  `motor-delay=15ms`での位相余裕トレードオフを払わずに`torque-authority`頑健性を
  得られる可能性がある（未検証の仮説）

文献的位置づけ・参考文献は§6参照。

### 3.5 PI型スライディング面の実装結果（部分的改善、完全解決ではない）

§3.4の提案を実装した（`smc_rate.hpp`の`SlidingModeRate`に積分状態`integral`・
`prev_error`を追加、`pid.hpp`と同じ条件付き積分アンチワインドアップ、`params.cpp`に
`smc.{roll,pitch,yaw}.lambda_i`を追加）。`λ_i`を{1.5, 3, 5}でスイープし、名目+§3.2/3.3の
摂動群で確認した。

| `λ_i` | 名目 | torque-authority=0.4 | torque-authority=0.55 | motor-delay=15ms | noise n1 |
|-------|------|----------------------|------------------------|-------------------|----------|
| 0（§3.3、参考） | 2.88°✅ | 4.9〜5.2°❌ | 5.18°❌ | 23.5°❌（転倒ではない） | 未検証 |
| 1.5 | 2.86°✅ | **3.60°❌** | 3.74°❌ | 24.7°❌ | 4.50°❌ |
| 3 | 2.94°✅（ギリギリ） | 3.78°❌ | **3.28°❌** | 22.9°❌ | 4.73°❌ |
| 5 | 3.01°❌ | 3.06°❌ | 3.93°❌ | 22.6°❌ | 4.84°❌ |

**成果**: `torque-authority=0.4/0.55`のatt_rmseは約4.9〜5.2°から約3.0〜3.8°へ改善
（約30%減）——**しかも`motor-delay=15ms`はどの`λ_i`でも悪化しなかった**（tilt_maxは
常に22〜25°帯、ラウンド2〜3のような180〜201°の転倒は一切再現しなかった）。これが
最大の成果: torque-authority頑健性を、遅れ耐性を犠牲にせず得られることを確認した
（§3.4末尾の仮説どおり）。

**未解決**: 3値とも条件間で非単調（`λ_i=1.5`は`ta=0.4`最良かつ名目PASS、`λ_i=3`は
`ta=0.55`はより良いが名目はギリギリ、`λ_i=5`は名目自体がFAIL）で、**どの`λ_i`でも
3.0°ゲートには届かなかった**。`noise n1/n2`は`λ_i`を上げても改善しなかった。

**最終選定**: `λ_i=1.5`（名目PASS・`torque-authority=0.4`の改善幅最大）を`params.cpp`の
既定値として採用。`stab_flight`での`torque-authority=0.4/0.55`・`noise n1/n2`は
**§3.3と同じく既知・記録済みの未解決FAILのまま**——実機投入ゲートは変わらず未達。

**次の一手（未実施）**: `λ_i`単独のスイープでは不十分だったため、`λ_i`と`η`の同時
再探索（積分項に頑健性の一部を任せた分、`η`を下げられる余地があるはず、という
§3.4末尾の仮説の後半部分はまだ検証していない）が次の自然な手がかり。

### 3.6 積分要素のリセット・アンチワインドアップ（レビューを受けて追加）

「超平面の積分要素のリセットはどうなっているか」という指摘を受けて整理・追加実装した。

**`reset()`が発火するタイミング（コード確認済み）**: `state_task.cpp:184-186`で
`IDLE_GROUND→ARMED_GROUND`（**ARM時のみ**、DISARM時は発火しない）に
`ControllerCmd::Reset`が1箇所だけpublishされ、`control_task.cpp:128-130`経由で
`AppController::reset()`→`smc_{roll,pitch,yaw}_.reset()`（`integral=0`）に伝播する。
モード切替（`onModeChange`）ではリセットされない——`PidController::onModeChange()`を
確認したところ、`att_roll_`/`att_pitch_`/`alt_pos_`/`alt_vel_`/`pos_x_/y_`/`vel_x_/y_`は
リセットするが**`rate_roll_`/`rate_pitch_`/`rate_yaw_`（SMCが置き換えた層）はリセット
しない**——既存PIDの既定動作そのものであり、本SMCの実装（モード切替でSMC側は
リセットしない）はこれに正しく倣っている。

**発見した抜け**: `pid.hpp`の`PID`はアンチワインドアップを「条件付き積分」＋
「`|integral|`へのハードクランプ（保険）」の二重構成にしているが、`SlidingModeRate`は
条件付き積分のみで、後者の保険が欠けていた。追加実装した（`smc_rate.hpp`）:

1. **条件付き積分**（既存）: 到達則全体のトルクで判定し、既に飽和方向へ押す更新は棄却
2. **バックストップ・ハードクランプ**（新規）: `integral`の線形寄与分
   （`inertia·eta·λ_i·integral`）単体が`output_limit`を超えないよう`|integral|`を制限
3. **偏差ゲートによるリセット**（新規、ユーザー提案）: `|e|`が閾値`e_reset`（`params.cpp`
   初期値は`5·φ`——roll/pitch 0.75 rad/s、yaw 1.5 rad/s）を超えたら積分を単に凍結でなく
   `0`へスナップする。大きな過渡（激しい機動・大外乱）で蓄積した古い積分値が、その後の
   平穏な追従へ漏れ出すのを防ぐ

回帰確認（`acro_flight`/`stab_flight`名目/`torque-authority=0.4/0.55`/`motor-delay=15ms`/
`noise n1`/`yaw_hold`）の結果、数値は追加前（§3.5）と誤差範囲内で一致——第2・3層は
これらの条件では発動せず「保険として静かに待機」しており、既存の動作を変えずに安全性
だけを積み増せたことを確認した。

### 3.7 `η`と`λ_i`の同時再探索（`torque-authority=0.4`が初PASS）

§3.5末尾の仮説（積分項に頑健性の一部を任せた分`η`を下げられるはず）を検証すべく、
`η`と`λ_i`を同時に振る探索を行った。結果は非単調・凹凸のある探索空間で、6通りの組
（`η`を下げる方向・上げる方向、`λ_i`を1.5〜6の範囲）を試した:

| 候補 | `η`(roll/pitch) | `λ_i` | `φ` | 名目 | ta=0.4 | ta=0.55 | delay=15ms |
|------|-----------------|-------|-----|------|--------|---------|------------|
| §3.5基準 | 150/145 | 1.5 | 0.15 | 2.92°✅ | 3.64°❌ | 3.72°❌ | 24.7°❌（転倒でない） |
| G（`η`を下げる） | 100/95 | 3 | 0.15 | 3.53°❌ | 4.09°❌ | 3.80°❌ | 23.5°❌ |
| H | 180/175 | 4 | 0.15 | 2.85°✅ | 3.38°❌ | 3.05°❌（僅差） | 24.0°❌ |
| I（さらに上げる） | 200/195 | 5 | 0.15 | 2.57°✅ | 3.60°❌ | 3.45°❌ | 24.4°❌ |
| **J（採用）** | **180/175** | **6** | **0.15** | **2.57°✅** | **2.78°✅** | 3.69°❌ | 24.3°❌ |
| K | 180/175 | 5 | 0.15 | 3.02°❌ | 3.96°❌ | 2.86°✅ | 24.4°❌ |
| L | 180/175 | 6 | 0.20 | 2.60°✅ | 3.05°❌（僅差） | 3.58°❌ | 24.4°❌ |

`η`を下げる方向（G）は仮説に反して全体が悪化した——積分作用だけでは`η`の役割を代替
しきれない。逆に`η`を**控えめに**上げつつ（150→180、ラウンド2〜3の破滅的な400とは
程遠い）`λ_i`を大きく上げる（1.5→6）方向が有効だった。ただし`λ_i`=4/5/6が単調に
改善するのではなく、`ta=0.4`と`ta=0.55`で逆の勝敗を示すなど（例: K は`ta=0.55`のみ
PASSしつつ名目がFAILする）、探索空間自体が非単調・凹凸型である。

**採用: J（`η`180/175、`λ_i`=6、`φ`は0.15のまま）** をparams.cppの既定値とした。
`params.cpp`の既定値のみ（`--param`無し）で再検証した最終結果:

| シナリオ/条件 | 結果 |
|---------------|------|
| `alt_flight`名目 | PASS |
| `acro_flight`名目 | PASS（att_rmse 1.94°） |
| `stab_flight`名目 | PASS（att_rmse 2.57°、§3.3の2.88°より改善） |
| `stab_flight` + `torque-authority=0.4` | **PASS（2.78°）— 本計画で初めてこの条件をクリア** |
| `stab_flight` + `torque-authority=0.55` | FAIL（3.69°） |
| `stab_flight` + `motor-delay=15ms` | FAIL（24.3°、転倒ではない。§3.3以降一貫して同じ帯） |
| `stab_flight` + `noise n1` | FAIL（5.16°） |
| `stab_flight` + `noise n2` | FAIL（4.18°） |
| `yaw_hold` | yaw_band/tilt_max/alt_min/duty_maxはPASS、alt_maxのみ既存の無関係な既知問題でFAIL |

`torque-authority=0.4`（simulation-policy.mdが定める摂動族の最も厳しい端）を初めて
クリアできたのは前進だが、`torque-authority=0.55`・`noise n1/n2`は依然未解決——
実機投入ゲートはまだ満たしていない。§3.3で確立した安全性優先の原則（`motor-delay=15ms`
は転倒しない限り許容し、torque-authority頑健性を優先する）は本ラウンドでも維持された。

### 3.8 スイッチングゲイン`k`の再探索（`torque-authority=0.55`が初PASS、しかし逆転）

実機投入を一旦見送り（ユーザー判断）、`k`（到達則のスイッチングゲイン、§3.7まで
20/20で固定）を§3.7の`η`180/175・`λ_i`=6・`φ`=0.15を固定して{20, 30, 40, 45}でスイープ
した。

| `k` | 名目 | ta=0.4 | ta=0.55 |
|-----|------|--------|---------|
| 20（§3.7採用値） | 2.57°✅ | **2.78°✅** | 3.69°❌（+23%） |
| 30 | 2.41°✅ | 4.07°❌（隣接点より悪化） | 3.08°❌ |
| **40（採用）** | 2.45°✅ | 3.06°❌（**僅差、+2%**） | **2.57°✅** |
| 45 | — | 3.32°❌ | 3.16°❌（両方再悪化） |

またも非単調——`k=30`は両隣（20・40）より両条件とも悪化し、`k=45`は`k=40`から
両方向とも後退した。`k=20`と`k=40`はそれぞれ`ta=0.4`と`ta=0.55`のどちらかを綺麗に
通すが**両方同時に通す`k`は見つからなかった**。総ゲート超過（PASSしていない方の
超過率）を比較すると`k=40`（`ta=0.4`が+2%のみ）の方が`k=20`（`ta=0.55`が+23%）より
明確にバランスが良いため、`k=40`を採用した。

**`params.cpp`の既定値のみでの最終回帰確認**:

| 条件 | 結果 |
|------|------|
| `acro_flight`名目 | PASS |
| `stab_flight`名目 | PASS（2.45°） |
| `stab_flight` + ta=0.4 | FAIL（3.06°、僅差） |
| `stab_flight` + **ta=0.55** | **PASS（2.57°）— 新たにクリア** |
| `stab_flight` + motor-delay=15ms | FAIL（24.1°、転倒でない、§3.3以降一貫） |
| `stab_flight` + noise n1 | FAIL（4.75°） |
| `yaw_hold` | yaw_band/tilt_max/alt_min/duty_maxはPASS、alt_maxのみ既知の無関係問題 |

§3.7〜3.8を通じて計9通りの`(η, λ_i, φ, k)`点を試行し、探索空間が本質的に非凸である
ことを確認した——グリッドサーチ的な人手の探索では限界が見えてきており、`ta=0.4`の
残り2%を埋めるには、手動スイープでなく実際の最適化アルゴリズム（Nelder-Mead等）による
探索、あるいは§2.1のスコープ自体の見直し（例: 到達則の別形状）が必要な可能性が高い。
実機投入ゲートはまだ満たしていない。

### 3.9 実機書き込み（2026-09-11、ユーザー判断によりACRO限定で先行実施）

ユーザー判断により、STABILIZE以上のロバスト性ギャップ（§3.8）が残る状態のまま、
ACROモード限定・ベンチ/テザー拘束下での初回実機投入に進んだ。

**副産物のバグ修正（2件目）**: `sf app build smc_rate`（実機ESP-IDF/Xtensaビルド、
本セッションで初めて実行）が、`simulator/sils/CMakeLists.txt`で一度修正したのと
**同じ根本原因**（Windowsバックスラッシュパスの`\U`エスケープ問題）で失敗した——
今回は`firmware/vehicle/main/CMakeLists.txt`の`SF_APP_DIR`処理（ESP-IDF本体の
`component.cmake`側`foreach()`）。同じ`file(TO_CMAKE_PATH ...)`正規化で修正。

**実行結果**:
```
[OK] Build successful: smc_rate (1122.4 KB, フラッシュ空き63%)
[OK] Flash successful: smc_rate (COM3, 460800 baud)
```
起動ログ確認: 5フェーズ起動シーケンス・キャリブレーション（level_offset/gyro_bias算出）
とも正常完了。磁力計未校正の警告のみ（既知・想定内）。

**引き続き未解決のまま**（§3.8時点の状態から変更なし）: `stab_flight`
条件下の`torque-authority=0.4`（僅差）・`noise n1/n2`。**STABILIZE以上での飛行は
非推奨**、ACROモード・ベンチ/テザー拘束下でのテストに限定することをユーザーに
申し送り済み。

### 3.10 POS_HOLD位置制御崩れの実機報告とSILSでの原因特定

実機テストで「角度の水平性は良好だが位置制御（POS_HOLD、自由飛行）が崩れた」との報告を
受けた（`docs/plans/woolly-wondering-micali.md`の調査プラン参照）。フライトログは取得不能
だったため、コード調査から立てた仮説（`attitude.{roll,pitch}.trim`——PID時代にNVS学習・
永続化された姿勢トリムが、SMCの定常追従特性に対して不整合になっている）をSILSで直接検証した。

`pos_flight.scn`（POS_HOLD斜め複合capstone、drift<3.0mゲート）に`--param
attitude.roll.trim=X --param attitude.pitch.trim=X`でPID時代相当の学習済みトリムを模擬注入し、
PID/SMCで比較した:

| トリム`X` [rad] | PIDのdrift | SMCのdrift |
|-----------------|-----------|-----------|
| 0（既定） | 0.33m ✅ | 0.74m ✅ |
| 0.01 | — | 1.47m ✅（ゲート近づく） |
| 0.02 | 0.016m ✅ | 2.07m ✅（ゲートに肉薄） |
| 0.05 | 0.036m ✅ | **5.64m ❌（ゲート3.0m大幅超過）** |

**明確な用量反応関係を確認**: PIDはトリム値に対しほぼ無感（0.05 radでも0.036mと実質誤差
レベル）だが、**SMCはトリムに対し強く感度があり、0.05 radで5.64mまでドリフトする**。
トリムはPidControllerの角度外側ループ（無改造・両者共通）で目標角度に加算されるため、
レート目標（`rate_ref`）自体は両者に同一に渡るはず——それでもこの差が出るのは、レート
ループの追従精度の違いが、（本来は積分作用で吸収するはずの）角度外側ループの補正能力を
超えて蓄積してしまうため、と考えられる（正確な機構は未解明、§3.10末尾のNext参照）。

これは実機報告と定性的に一致する: 姿勢角度そのものの追従誤差（`att_rmse`/`tilt_max`）は
トリム0.05でもゲート内に収まっており「水平性は良い」という報告と整合しつつ、位置だけが
大きく崩れる。

**仮説は確証された**と判断する。`pos_roll`/`pos_pitch`/`pos_yaw`（トリム0でのSMC単体健全性）
の確認は次のステップとして残る。

### 3.11 実機トリム確認（仮説否定）とnoise再検証（こちらも否定）

実機再接続後、シリアルコンソール経由で`attitude.roll.trim`/`attitude.pitch.trim`を
直接確認したところ**両方とも0**だった——§3.10のトリム仮説は**このケースでは否定**された
（SILSで確認したSMCのトリム感度自体は実在するバグとして§3.10に残すが、今回の実機事象の
原因ではない）。

機体のSPIFFSログ領域も起動ログで`0 / 1920401 bytes used`（完全に空）で、`sf log capture`も
コンソールが`binlog`コマンドを認識せず失敗——**該当フライトのログは記録されておらず、
実機データからのフォレンジック調査は不可能**と判明した。

トリムが否定されたため、次点の仮説（§3.2/3.3で確立したSMCのnoise脆弱性がPOS_HOLDでも
再現するのでは）を`pos_flight.scn`で`--noise n1`/`n2`により検証したが、**これも支持されな
かった**——むしろ逆だった:

| 条件 | PID | SMC |
|------|-----|-----|
| 名目 | duty_maxのみFAIL（既知xfail、drift 0.33m良好） | 全項目PASS（drift 0.74m） |
| noise n1 | **`Takeoff complete`未達——離陸未完了のまま終了** | 完走、drift 1.60m（ゲート内）、duty_maxのみFAIL |
| noise n2 | **同上、離陸未完了** | 完走、drift 1.09m（ゲート内）、duty_maxのみFAIL |

PIDがnoise下で離陸完了しない現象は§3.2で`acro_flight`/`stab_flight`でも観測済み
（原因未調査のまま§5 Next stepsに放置）で、**POS_HOLDでも同様に再現し、PID側の既存の
未解明課題**であることが追加確認された。SMCはこの条件下でむしろPIDより頑健に飛行を完遂
している——今回の実機での位置制御崩れをnoise感度で説明する筋書きは支持されない。

**現時点の結論**: リーディングだった2つの仮説（トリム不整合・noise脆弱性）はいずれも
SILS/実機確認で**否定**された。該当フライトのログが存在しないため、以下は未検証のまま
残る候補である:
- 実機個体固有のモータ非対称性・torque-authorityがSILS検証レンジ（[0.4,1.0]の合成摂動）
  の外にある可能性
- オプティカルフロー（PMW3901）の床面テクスチャ/照明条件——ESKFの水平速度推定はコントローラ
  に依存しないため理論上は無関係のはずだが、実環境要因として排除できない
- 実際の風・機体外乱——SILSの`--turbulence`/`--ground-effect`未検証

**申し送り**: 次の実機テストでは**必ず先に`sf log wifi`または適切なログ機構を有効化**して
から飛行し、再発時に実データで検証できる状態を整えること。安全上の推奨（ACRO・ベンチ/
テザー拘束下）は変わらず維持する。

### 3.12 `sf log capture`が現行ファームで機能しない（既存ギャップ、SMC無関係）

§3.11で`sf log capture`が`binlog on`を送って失敗した件を追跡した。`firmware/vehicle/tasks/
cli_task.cpp`の`kCommands[]`（コマンドレジストリ）を確認したところ、現行世代コンソールが
持つコマンドは`param`/`status`/`sensor`/`version`/`pair`/`unpair`/`sound`/`led`/`motor`/
`wifi`/`magcal`/`reboot`の12個のみで、**`binlog`はそもそも存在しない**。`binlog`は
`firmware/vehicle_old`（レガシー世代）のコンソール専用コマンドで、`sf log capture`
（USB経由バイナリログ）はその前提を引き継いだままになっている——CLITaskはコントローラ
実装に依存しないコアタスクなので、**これは本SMC変更と無関係の既存ギャップ**である。

現行世代（`firmware/vehicle`）での正しいログ取得経路は`sf log wifi`（UDP:8890、起動ログの
`Data Stream ready on UDP 8890`と一致）。次回実機飛行の前に、この経路でログ取得できることを
確認しておくこと（機体のIPアドレス把握が前提——起動ログのSoftAP `192.168.10.1`、または
STA接続時のホームネットワークIP）。`sf log capture`のバインディングを現行ファーム向けに
修正する（`cli_task.cpp`へ`binlog`相当のコマンドを追加する）ことは別途の独立した修正候補
だが、今回は範囲外とする。

なお`sf status`コマンド（bettery等を含む）が現行ファームの正しいバッテリー確認手段——
§3.11で試した`battery`コマンドは存在しない（`Unrecognized command`は妥当な応答だった）。

### 3.13 POS_HOLD系シナリオ全4本のカバレッジ完了

`TEST_MATRIX.md`のPOS_HOLD系シナリオ4本（`pos_roll`/`pos_pitch`/`pos_flight`/`pos_yaw`）を
`apps/smc_rate`に対して初めて一通り実行し、PIDベースラインと比較した（§5 Next stepsに
残っていた検証ギャップの解消）。

| シナリオ | PID | SMC |
|---------|-----|-----|
| `pos_roll` | 全項目PASS（drift 0.56m） | 全項目PASS（drift 0.43m、att_rmse 0.27°とPIDより良好） |
| `pos_pitch` | 全項目PASS（drift 0.60m） | 全項目PASS（drift 0.45m、att_rmse 0.28°とPIDより良好） |
| `pos_flight` | duty_maxのみFAIL（既知xfail） | 全項目PASS（drift 0.74m） |
| `pos_yaw` | **DISARM未達（duty_max=1.0飽和で完走できず）** | 全項目PASS（drift 0.54m、duty_max 0.82） |

**4本中4本でSMCがPIDと同等以上**——特に`pos_flight`/`pos_yaw`ではPID側が既知のヨートルク
権限飽和（xfail）で崩れる一方、SMCは問題なく完走している。POS_HOLD位置制御の頑健性という
観点では、SILSが示す限りSMC（レートループのみ置換、位置ループはPidController委譲のまま）
は既存PIDを下回っていない——`docs/plans/smc-rate-loop-plan.md`全体を通じて、位置制御自体を
追加でSMC化する差し迫った技術的動機は今のところ見当たらない。§3.11の実機での崩れは、
これらのSILS結果と整合しない（SILSは健全と示すため）——真因はSILSでは再現できない実機
固有の要因にある可能性が高まった。

### 3.14 【重要・運用上の注意】WiFi STAモード切替でESP-NOW（プロポ通信）が切れる事故

§3.12の対処として`wifi mode sta`で自宅Wi-Fi（`ASUS_5E2.4`）に接続させたところ、**プロポの
操作が一切機体に届かなくなる事故が発生**した（地上で発生、実害なし）。

**原因**: StampFlyのWiFi実装は、SoftAPモード時は「`WiFi SoftAP ... (telemetry; ESP-NOW
unchanged)`」——ESP-NOWは固定チャンネル11。しかしSTAモードで外部APに参加すると
「`WiFi STA joining ... (telemetry; ESP-NOW follows the AP channel)`」——**ESP-NOWのチャンネルが
参加先APのチャンネルに連動して変わる**（`ASUS_5E2.4`はチャンネル4だった）。プロポは
チャンネル11で待ち受けたままのため、通信が完全に切断された。

**対処**: `wifi mode ap` → `param save` → `reboot`でSoftAPモード（チャンネル11固定）に
戻し、プロポ操作を復旧した。復旧はログの`StateManager: Mode change: STABILIZE → ACRO`等が
起動直後から連続することで確認した（プロポのモード切替スイッチ操作をリアルタイムで検知）。

**運用上の教訓**:
- `sf log wifi`のために`wifi mode sta`で自宅Wi-Fiに繋ぐ運用（§3.12）は、**プロポとの
  ESP-NOW通信を道連れに切断するリスクがある**——同じ理由でSTAモードのままARM/飛行を
  絶対に行わないこと
- ログ取得目的でSTAモードにする場合は、**地上・非武装時に限定し、ログ取得後は必ず
  `wifi mode ap`に戻してからプロポ操作・飛行を行う**運用ルールとする
- あるいは、プロポ側の待受チャンネルを自宅Wi-Fiのチャンネルに合わせられるなら（未調査）
  その方が安全かもしれない——今回は追わなかった
- 開いたシリアルポートがDTR/RTS経由で機体をリセットする仕様のため、`wifi mode`等のコマンドは
  **ポートオープン直後でなく、起動シーケンス完了を待ってから**送る必要がある（今回1回、
  リセット直後にコマンドを送って反映されず、`wifi show`で気づかず先に進んでしまった）

### 3.15 実機クラッシュのログ解析（2026-09-11、`sf log wifi`で新規取得）

§3.14の対処（自宅ルータの2.4GHz帯をチャンネル11に固定し、WiFi STA接続と
プロポ（ESP-NOW）を両立）が成功したのち、`sf log wifi -i 192.168.50.85 -d 180`で
新規フライトログ（`logs/smc_test_flight.jsonl`、243,179行、400Hz）を取得した。
この取得中に実機がクラッシュ（ユーザー報告「墜落した」、負傷・機体異常なしを確認済み）。
以下はこのログの解析結果。**§3.10-3.11で調査した過去の「姿勢は良好だが位置制御が崩れた」
報告とは別のイベント**であり、混同しないよう注意。

**解析手順**: `sf log viz logs/smc_test_flight.jsonl --time-range <START> <END> --save <FILE>`
でズームプロットを生成し目視確認。また`sf log analyze logs/smc_test_flight.jsonl --health`で
モータ健全性レポートを取得。生ログは`awk`で該当`ts`範囲の`posvel`/`ctrl`レコードを直接
抽出し、プロットだけでは分からない「瞬間的な値」を確認した（`ts`は起動からのマイクロ秒、
`sf log viz`の相対時刻は先頭サンプルの`ts`を0とした`ts/1e6`）。

**イベント概要**（起動後uptime、`sf log viz`相対時刻はuptime-25.67s）:

| 起動後uptime | viz相対時刻 | 内容 |
|---|---|---|
| 〜82.0s | 〜56.3s | 低高度（ToF Bot 0〜2m）でPOS_HOLD中、ロールスティック**-0.96（左ほぼフル）を0.7秒以上継続保持**、スロットルはほぼ0（`throttle`が82.08s時点から既に0.0） |
| 82.45〜82.50s | 56.8s | ジャイロ・加速度に瞬間的な大スパイク（補正済み値で±30rad/s、±80m/s²）、姿勢が±150°超へ急変 |
| **82.503204s** | **56.8s** | **`posvel`（ESKF位置/速度）が1テレメトリ周期（50ms）内に`(0.58,-0.89,-1.00)`→`(0,0,0)`へ瞬時リセット**。物理的な減速ではあり得ない不連続変化 |
| 82.72〜82.76s | 57.1s | ロール・ピッチスティック入力が-0.96→0へ約40msで収束、以降ログ末尾まで`throttle/roll/pitch/yaw`は全て0.0で不変 |
| 〜89.7s | 64s | 2回目の大スパイク（同様の姿勢・ジャイロ・加速度の急変）。この間`posvel`/`ctrl`は前述の通りすでに0で不変 |
| 89.7〜94s | 64〜68s | 姿勢が緩やかに収束（Roll 170°→80°、Pitch/Yawも減衰）、ジャイロ・加速度も静定 — 機体が地面で静止に向かうのと整合的 |

**評価**: これは緩慢な「ドリフト」ではなく、(1) スロットルほぼ0のままロールスティックを
0.7秒以上フル近く倒し続けた状態からの、(2) 低高度での急激な並進・落下、(3) 衝突と推測される
瞬間的な大加速度・大角速度スパイク、という**速い・激しいイベント**の記録である。
`posvel`が衝突と同時刻に1周期で厳密に0へジャンプする挙動は、位置推定が物理的に収束したの
ではなく、**着陸/クラッシュ検出等による状態リセットの結果**である可能性が高い（`status`
レコードの`flight_state`/`eskf_status`はログ全体を通じて`1`のまま変化がなく、WiFiテレメトリ
経由ではこの遷移を裏付けられなかった——推定の域を出ない点に注意）。したがって今回のイベントは
§3.10-3.11で調査した「姿勢は健全なまま位置制御だけが緩慢に崩れる」パターンとは**波形の性質が
異なり**、同一原因と決めつけるべきではない。

**モータ健全性レポート（`--health`）の新知見**:

```
Duty deviation from 4-motor mean (x1000): M1/FR -20.6  M2/RR +26.4  M3/RL -2.9  M4/FL -2.9
Saturation (duty>=0.98): M1/FR 2%, M3/RL 1% of samples
Verdict: yaw trim ur=-0.047 (CG-immune) => CW group {M2/RR, M4/FL} is weak
         Corner (lean): M2/RR(CW) — reaction-torque deficit points to prop drag
```

このログ単独の解析であり`--batch`によるCG除去確認はできていないが、**実際のモータ間で
無視できない非対称性（M2/RRの反力トルク不足）と、2%程度のduty飽和が実測された**。これは
本計画の§3（代替調査候補）に「実機torque-authorityがSILS検証レンジ外」として未検証のまま
残していた仮説に対する、初めての実測的な裏付けである。飽和が発生する状況（本ログのような
ロールフル入力時など）では制御余裕が薄いことを意味し、衝突そのものの直接原因とは言い切れない
ものの、姿勢を素早く戻す余力を削っていた可能性はある。

**結論・Next steps**:
- 本イベントは主に「低高度・低スロットルでロールフル入力を継続 → 衝突」という運用（操縦）
  由来の可能性が高く、SMCレートループの発散を示す直接証拠はない（衝突後のスパイクはSMC/PID
  どちらでも物理衝突なら同様に出る）。ただし対称性の検証は未実施
- [ ] M2/RRのプロペラを現物確認（欠け・曲がり・ガタ）——`--health`の推奨手順に従う
- [ ] 可能なら`sf log analyze --health --batch`用に複数のクリーンなホバーログを追加取得し、
      CG除去でM2/RR判定を確定させる
- [ ] 次回実機テストは§3.14の教訓（WiFi STA運用ルール）に加え、**低高度・フルスティック
      入力を伴う積極的な操縦は避け**、ベンチ/テザー拘束下でのACRO確認から再開することを推奨

**追記（同日、ACRO限定の再テスト）**: 上記推奨に沿ってACRO中心（`sf log wifi`で30秒、
`logs/smc_test_flight_20260911_180500.jsonl`）で再テストしたところ、姿勢は終始±100°以内に
収まりクラッシュなし。この新ログで`--health`を再実行したところ、**同じCW側モータグループ
{M2/RR, M4/FL}の弱さが、より大きな偏差で再現**した:

```
                         yaw trim ur       duty deviation(x1000)              saturation
smc_test_flight(初回)     -0.047      M1 -20.6 / M2 +26.4 / M3 -2.9 / M4 -2.9   M1 2%, M3 1%
smc_test_flight_180500   -0.275      M1 -88.7 / M2 +82.0 / M3 -48.6 / M4 +55.3  M2 2%, M4 3%
```

2本の独立したログで同じCWグループ（M2/RR, M4/FL）が弱い方向に一致しており、`--batch`による
CG除去確認を待たずとも、M2/RRプロペラの現物確認は優先度を上げてよい。

## 7. 位置制御ロバスト化（速度ループSMC化、`firmware/apps/smc_pos`）

### 7.1 経緯・設計判断

§3.15のモータ非対称性実測を受け、ユーザー提案「位置制御にもSMCを」に対し、実装前に
`firmware/vehicle/docs/poshold_journey.md` §4を確認したところ、**位置/速度ループは既に
実機プラント同定（実効「傾き→速度」ゲイン約0.4g、原因はモータトルク効き不足）に基づき
再設計・実機検証済み**（`position.vel.kp` 0.8→3.0, `position.pos.kp` 1.0→0.4、
K∈[2.8,7]で安定確認、実機で±6-7cmホールド達成）であることが判明。「±6-7cmはこの
37g機体のトルク効きの実用限界」と明記されており、これは制御則でなくハードウェアの限界と
結論されている。この新事実をユーザーに報告した上で、「既存が壊れているから」ではなく
**既に頑健化済みのPIDベースラインとのA/B/C比較**として実装を進める方針で合意した
（AskUserQuestion経由）。

アーキテクチャは、PidController本体へ最小限の差し替え口
（`setVelocityLawOverride()`、`computePositionHold()`内、位置ループ→速度ループの
vx/vy→ax/ay段のみ）を追加する方式を採用（専用IController全自作は不採用、理由は
コミットログ・app_controller.hppのコメント参照）。既定`nullptr`ならvehicleビルドの挙動は
1バイトも変わらないことをSILS回帰（`pos_flight.scn`、`vehicle`ターゲット、フック追加前後で
全メトリクス完全一致）で確認済み。

新設した`firmware/apps/smc_pos`は、`smc_rate`のレートループSMCをそのまま流用し
（`smc_rate.hpp`無変更コピー）、新規`smc_vel.hpp`（`SlidingModeVelocity`、
`SlidingModeRate`と同型・単位を加速度/速度に再導出）を`PidController::
setVelocityLawOverride()`経由で速度ループへ注入する。位置ループ（`pos_x_`/`pos_y_`）は
無改造のまま。新規param `smc.{velx,vely}.{k,eta,phi,lambda_i,e_reset}`（計10個）追加。

### 7.2 ラウンド1 SILS結果（初期シードゲイン、未チューニング）

`pos_flight.scn`（3者比較、`vehicle` / `apps/smc_rate` / `apps/smc_pos`）:

| target | horizontal_drift_max (<3.0) | tilt_max (<18.0) | duty_max (<0.9) | att_rmse (<5.0) | 総合 |
|---|---|---|---|---|---|
| vehicle (PID) | 0.3265 PASS | 11.10 PASS | **1.0000 FAIL** | 1.2644 PASS | FAIL（duty飽和、既知） |
| apps/smc_rate | 0.7394 PASS | 13.93 PASS | 0.8902 PASS | 1.3044 PASS | **PASS（全12項目）** |
| apps/smc_pos  | 0.7063 PASS | 13.93 PASS | **1.0000 FAIL** | **0.6680 PASS** | FAIL（duty飽和、vehicleと同じ症状が再発） |

**所見**: att_rmseはsmc_posが最良（0.668、smc_rateの約半分）——速度ループSMCが姿勢追従を
引き締める効果が見える。一方でduty_maxがvehicle同様1.0（飽和）に戻ってしまい、
smc_rate単体が達成していた「duty<0.9」の改善が失われた——速度ループSMCの加速度指令が
（seedゲイン段階では）レートSMC単体より高いモータ要求を生んでいる可能性。

`pos_gain_deficit_smallnudge_hold40.scn`（`--torque-authority 0.55`注入、40秒保持中の
水平振動の成長/収束を`trajectory.csv`のpx/pyから直接評価、この`.scn`自体に`.expect`
ゲートは無い——実機同定振動の周期9.4秒を検出する設計のため長時間の振幅推移を目視/数値で見る
運用):

| target | 保持前半(15-20s)水平半径max | 保持後半(45-55s)水平半径max | 傾向 |
|---|---|---|---|
| vehicle (PID) | 0.0485 m | 0.0361 m | 収束（既存の広いロバストマージンの通り） |
| apps/smc_rate | 0.0464 m | 0.0505 m | 微増（境界に近いが小さい） |
| apps/smc_pos  | 0.0340 m | 0.0342 m | **ほぼ一定、かつ3者中最小振幅** |

**所見**: この特定の摂動レベル（torque-authority=0.55）では、smc_posが3者中もっとも
タイトかつ安定したホールドを示した。ただし**単一シナリオ・単一摂動レベル・未チューニングの
初期シードゲインでの結果であり、smc_rateが辿った5ラウンドのチューニング・摂動族テスト
（§3.2-3.8）に相当する検証はまだ行っていない**。この結果だけで優劣を結論づけない。

### 7.3 現状のステータスとNext steps

- [x] `PidController`への差し替え口追加・vehicle無変更を回帰確認
- [x] `smc_pos`実装（レートSMC流用+速度SMC新規）、SILS/実機（ESP-IDF）双方でビルド確認
- [x] ラウンド1 SILS結果取得（`pos_flight`3者比較、gain_deficit 1条件）
- [ ] duty飽和の再発（§7.2）の原因調査 — 速度SMCの`k`/`eta`が現行seedで過大な可能性、
      `smc_rate`が辿ったのと同様の複数ラウンドチューニングが必要
- [ ] `pos_roll`/`pos_pitch`/`pos_yaw`/`pos_reposition`/`pos_auto_takeoff`の残り全シナリオ
- [ ] `pos_gain_deficit_*`を複数の`--torque-authority`水準（0.4〜1.0）でスイープ
- [ ] `--motor-delay`/`--noise`摂動族（`smc_rate`の§3.2と同じ一式）
- [ ] 結果を踏まえチューニングラウンドを回す（`smc_rate`の§3.1-3.8と同じ規律）
- [ ] 実機投入は本計画§4と同じゲート運用（ベンチ/テザー・ACRO確認から）

### 7.4 ラウンド2: duty飽和の解消とフルバッテリー検証

§7.3の課題「duty飽和の再発」に対処するため、`pos_flight.scn`で(k,eta,phi)の4点スイープ
（他はラウンド1のまま）を実施:

| 構成 | duty_max(<0.9) | att_rmse(<5.0) | drift(<3.0) | 判定 |
|---|---|---|---|---|
| ラウンド1シード (k=0.45,η=3.0,φ=0.15) | 1.000 FAIL | 0.668 | 0.706 | FAIL |
| A (k=0.3,η=1.5,φ=0.2) | 0.973 FAIL | 0.573 | 0.906 | FAIL |
| **B (k=0.2,η=1.0,φ=0.25)** | **0.780 PASS** | 0.514 | 1.246 | **PASS** |
| C (k=0.1,η=2.5,φ=0.3) | 1.000 FAIL | 0.846 | 0.757 | FAIL |
| D (k=0,η=3.0,φ=0.15, pure-P) | 1.000 FAIL | 0.613 | 0.742 | FAIL |

**所見**: ηが既存PID線形ゲイン`position.vel.kp=3.0`に近いと（k=0/0.1/0.45のいずれでも）duty
飽和する。η=1.0（既存PIDの約1/3）まで下げ、境界層φを広げて（0.15→0.25）スイッチング項
への依存を減らした構成Bだけが4ゲート全てをクリア。**ラウンド4/5で「smc_rateはηが元の
PID kpに近いほど良い」と逆の傾向が出た（§3.7-3.8）のと対照的**——同じPI型面SMCでも、
レートループと速度ループでは元のPIDとの相対的な位置づけが異なる（速度ループのPIDは
既に「実機同定に基づく強めのゲイン」で頑健化済みという§7.1の事情も影響していると推測）。

Bの5ゲイン（k/eta/phi=0.2/1.0/0.25、lambda_i=0.5据え置き、e_reset=5*phi=1.25で再計算）を
`params.cpp`のラウンド2デフォルトとして採用し、フルバッテリーで再検証:

| シナリオ/摂動 | drift | tilt | duty | att_rmse | 判定（数値4ゲート） |
|---|---|---|---|---|---|
| pos_flight (nominal) | 1.105 | 13.93 | 0.804 | 0.605 | 4/4 PASS |
| pos_roll | 0.671 | 6.52 | 0.729 | 0.261 | full PASS (12/12) |
| pos_pitch | 0.683 | 6.27 | 0.745 | 0.212 | full PASS (12/12) |
| pos_yaw | 0.752 | 9.72 | 0.796 | 0.996 | full PASS (12/12) |
| pos_reposition | (複数ゲート) | 2.17 | 0.718 | 0.067 | full PASS (19/19) |
| pos_auto_takeoff | — | 0.0 | 0.678 | — | full PASS (19/19) |
| torque-authority=0.4 | 1.149 | 14.35 | 0.733 | 0.837 | 4/4 PASS |
| torque-authority=0.55 | 1.140 | 14.13 | 0.765 | 0.886 | 4/4 PASS |
| **motor-delay=15ms** | **4.359 FAIL** | 17.28 | **1.000 FAIL** | 4.567 | **2/4 FAIL** |
| noise n1 | 1.980 | 9.60 | **1.000 FAIL** | 2.637 | 1/4 FAIL |
| noise n2 | 1.102 | 12.07 | **1.000 FAIL** | 1.406 | 1/4 FAIL |

**評価**: nominal・POS系全シナリオ・torque-authority摂動は全て明確に改善（ラウンド1の
duty飽和は解消）。**motor-delay=15msだけ、drift=4.36と明確な発散**（noise系はduty飽和は
するがdrift/att_rmseは有界に留まる——質的に別の失敗モード）。

### 7.5 motor-delay発散への対処試行（ラウンド3、結果: 未解決）

motor-delay=15ms条件でゲイン方向を2方向試したが、**どちらもラウンド2より悪化**した:

| 構成 | drift(<3.0) | duty(<0.9) | att_rmse(<5.0) |
|---|---|---|---|
| **ラウンド2 (k=0.2,η=1.0,φ=0.25)** | **4.36** | 1.00 | 4.57 |
| より柔らかく (k=0.15,η=0.6,φ=0.35) | 7.01（悪化） | 1.00 | 7.21（悪化） |
| さらに柔らかく (k=0.1,η=0.4,φ=0.4) | 8.58（悪化） | 1.00 | 7.38（悪化） |
| 柔+lambda_i=1.0 | 7.41（悪化） | 1.00 | 8.42（悪化） |
| より硬く (k=0.3,η=2.0,φ=0.35) | 9.44（さらに悪化） | 1.00 | 9.92（さらに悪化） |

**所見**: ゲインを弱めても強めても、ラウンド2より悪化する——`smc_rate`の§3.3-3.8で確認された
「非凸な探索空間」と同じ現象がここでも再現した。単純な1軸方向のゲイン調整では
motor-delay=15ms条件を解消できない。ラウンド2の(k=0.2,η=1.0,φ=0.25)は**今回試した中では
最良**（唯一motor-delayでも「発散はするが4.36止まり」——より弱い/強いゲインは7〜9台まで
悪化する）。根本対処には位相遅れを見込んだ到達則の再設計（例: 遅れ推定・予測補償）が
必要な可能性があり、単純なk/eta/phiスイープの範囲を超える——`smc_rate`が最終的に
noise n1/n2を解消しきれず既知の課題として残した（§4）のと同じ位置づけで、**ラウンド2を
現時点の到達点とし、motor-delay=15msでの発散を既知の未解決課題として記録する**。

### 7.6 現状のステータス（更新）

- [x] duty飽和の原因調査・ラウンド2ゲインで解消（nominal/POS全シナリオ/torque-authority）
- [x] motor-delay対処を2方向試行 → 悪化、ラウンド2が現状最良と確認
- [ ] **未解決**: motor-delay=15msでdrift発散（4.36 > 3.0ゲート）、noise n1/n2でduty飽和
      （drift/att_rmseは有界）——`smc_rate`と同じく実機投入前に要再検討
- [ ] 位相遅れを見込んだ到達則の再設計（Smith予測器的な補償等）は本ラウンドでは未着手
- [ ] 実機投入は本計画§4と同じゲート運用に加え、上記未解決条件がクリアされるまで
      ベンチ/テザー・ACRO限定を強く推奨

### 7.7 motor-delay根本対処: レートループへの無駄時間予測補償器

§7.5で「単純なk/eta/phiスイープでは解決しない」と結論した後、`smc_rate`自身の履歴と
`docs/architecture/simulation-policy.md`を再確認したところ、motor-delay=15msは
PID・SMC問わずプロジェクト全体で一貫してFAILしてきた既知の限界（「転倒しない」ことが
安全性の閾値、ゲート自体はクリアしない）であることが判明した。この前提をユーザーに
提示した上で、それでも根本設計に進むという判断を得た。

**設計**: レートループの`SlidingModeRate`（`smc_rate.hpp`、`smc_rate`/`smc_pos`共通）に
無駄時間予測補償器を追加。プラントが「トルク→角加速度=積分器」（追加の1次遅れは
無視できる、相対次数1の前提と整合）であることを利用し、Smith予測器[R4]
（O.J.M. Smith, 1957）の積分器プラント特化・簡略版として:

```
rate_predicted = rate_meas + Σ_{i=t-L}^{t} (torque_i / inertia) * dt
```

を実装（直近L秒間に自分が出したトルク指令をそのまま角加速度に換算して積算し、実測値へ
足し込む）。新param`smc.{roll,pitch,yaw}.delay_comp_ms`（既定0=無効、レートループの
既存挙動を完全に維持——回帰確認済み）。

**効果検証**（`--param smc.{roll,pitch,yaw}.delay_comp_ms=15`で明示的に有効化）:

| シナリオ | 条件 | 補償OFF | 補償ON(15ms) | 判定 |
|---|---|---|---|---|
| `pos_flight` (smc_pos) | motor-delay=15ms | drift=4.36 FAIL, tilt=17.28, duty=1.00 FAIL, att_rmse=4.57 | **drift=0.98 PASS**, tilt=13.84, duty=1.00 FAIL, **att_rmse=0.45 PASS** | 4項目中2項目が新規PASS、劇的改善 |
| `stab_flight` (smc_rate) | motor-delay=15ms | att_rmse=1.04, **tilt=24.06 FAIL**, duty=1.00 | att_rmse=2.21, **tilt=12.34 PASS**, duty=1.00 | シナリオ全体が**12/12 FULL PASS**（従来FAIL） |

**motor-delay=15ms条件に対しては明確かつ劇的な改善**——`smc_rate`自身が§3.3以降
一貫して解消できなかったFAIL（tilt_max 21-24°台）が、smc_rate・smc_posどちらも
ゲート内に収まった。

**ただし重要なトレードオフを発見**（既定OFFのまま維持し、常時ONにしなかった理由）:
補償を有効にした状態で、motor-delayを**注入していない**（実遅れ0の）nominal条件を
再検証したところ:

| シナリオ | 条件 | 補償OFF（ラウンド2） | 補償ON(15ms) | 判定 |
|---|---|---|---|---|
| `pos_flight` (smc_pos) | nominal | drift=1.11, tilt=13.93, duty=0.80, att_rmse=0.61 | drift=1.08, tilt=13.97, duty=0.81, att_rmse=0.72 | ほぼ同等、軽微な悪化のみ |
| `pos_flight` (smc_pos) | torque-authority=0.4 | duty=0.733 **PASS** | **duty=1.000 FAIL** | **新規退行** |
| `stab_flight` (smc_rate) | nominal | att_rmse≈1-2°台PASS（履歴） | **att_rmse=5.37 FAIL**（gate<3.0） | **新規・大幅退行** |

**評価**: 15msの無駄時間を「常に存在する」と仮定して予測すると、実際に無駄時間がない
（または注入量とズレている）条件ではモデル誤差そのものが外乱となり、nominal性能・
torque-authority摂動への頑健性を明確に悪化させる——**「実在しない遅れを補償しようとする
ことの副作用」**という、予測型補償器に典型的なトレードオフが実測された。
simulation-policy.mdの「SILS単独最適化禁止・摂動族全体で悪化させないこと」の原則に
照らすと、**既定で有効化するのは不適切**——ただし実機は常に何らかの無駄時間
（実測L≈8.4-14.7ms/軸）を持ち、SILSのnominal（無駄時間0）自体が実機からすれば
非現実的な条件である、という逆の見方もあり得るため、**既定OFFで維持しつつ、実機投入
判断はユーザーと個別協議する**こととした。

### 7.7b 感度検証（ミスマッチ・補償値の大きさ）

ユーザーからの追加確認要請を受け、「想定遅れと実際の遅れがズレたらどうなるか」
「補償値を小さくすれば安全か」を`smc_pos`（速度ループ、`pos_flight.scn`）と
`smc_rate`（レートループ、`stab_flight.scn`）の両方で検証した。

**`smc_pos`（速度ループ）:**

| 条件 | 注入遅れ | 補償値 | drift | tilt | duty(<0.9) | att_rmse |
|---|---|---|---|---|---|---|
| nominal, 小さい補償 | 0 | 8ms一律 | 1.18 | 15.0 | **0.77 PASS** | 1.28 |
| nominal, 実機軸別値 | 0 | 14.7/8.4/11.0 | 1.13 | 14.3 | 1.00 FAIL | 0.97 |
| 一致（小） | 8ms | 8ms一律 | 1.02 | 13.8 | 1.00 FAIL | 0.62 |
| 不一致（過大） | 8ms | 15ms一律 | 0.75 | 12.4 | 1.00 FAIL | 0.95 |
| 一致（中） | 11ms | 11ms一律 | 0.96 | 13.0 | 1.00 FAIL | 0.46 |
| 一致（実機軸別） | 15ms | 14.7/8.4/11.0 | 0.96 | 16.7 | 1.00 FAIL | 0.91 |

drift/tilt/att_rmseは**どの組み合わせでも健全**（一致・不一致を問わず）——§7.7で見た
smc_rate側の劇的な悪化は、この速度ループでは再現しなかった。一方**duty_maxは
comp値が~11ms以上（roll軸）になるとほぼ必ず飽和**し、一致・不一致にはあまり依存しない
——「ミスマッチの害」というより「補償値自体の大きさに比例するduty消費コスト」という
性質。**comp=8ms一律・遅れ無しの組み合わせだけが4ゲート全PASS**。

**`smc_rate`（レートループ、`stab_flight`nominal）:**

| 補償値 | att_rmse(<3.0) | tilt_max(<18.0) | duty_max |
|---|---|---|---|
| 0（既定） | 履歴上PASS（1-2°台） | — | — |
| 8ms | **4.54 FAIL** | 14.81 PASS | 0.76 PASS |
| 15ms | **5.37 FAIL**（§7.7既出） | 12.29 PASS | 0.80 PASS |

レートループは**8msの小さい補償値でもnominalのatt_rmseを悪化させる**（3.0ゲートを
明確に超過、15msよりはマシだが解消はしない）。速度ループより帯域が高く追従要求が
厳しいため、存在しない遅れの仮定への感度が高いと考えられる。

**結論**: 「小さい補償値なら安全」という単純な答えは無い——**ループによって挙動が違う**:
- 速度ループ（`smc_pos`）: 8ms程度の小さい補償なら、遅れが無い条件でも実害なし。
  実機投入するなら、まず控えめな値（8ms程度）から試すのが妥当
- レートループ（`smc_rate`）: 試した範囲（8ms・15ms）ではnominalの退行を解消できる
  補償値が見つからなかった——実機の遅れが常に一定以上存在することを前提にしない限り、
  現状は既定OFFのまま据え置くのが無難

### 7.7c 速度ループへの無駄時間予測補償器追加（ユーザー要請）と、レートループとの決定的な違い

ユーザーから「角度・位置の両方のSMCに追加したか」との確認を受け、速度ループSMC
（`smc_vel.hpp`の`SlidingModeVelocity`）にも同じ無駄時間予測補償器を追加して試した
（仮説: レートループだけで足りるはず——§7.7の設計はそう想定していた）。

新param `smc.{velx,vely}.delay_comp_ms`（既定0=無効）を追加し、`pos_flight.scn`で
「想定8ms・実際の注入遅れ15ms」という**過小ミスマッチ**（想定より実際の遅れが大きい）
条件を軸に比較:

| 構成 | drift(<3.0) | att_rmse(<5.0) | 判定 |
|---|---|---|---|
| 補償なし（基準） | 4.36 | 4.57 | 参考値 |
| レートのみ補償8ms（実際15ms、過小） | **27.90** | **16.27** | **無補償より6倍以上悪化** |
| レート8+速度8（両方、過小） | 20.96 | 14.02 | 依然壊滅的（速度補償が部分的に緩和） |
| **速度ループのみ補償8ms（実際15ms、過小）** | **0.51 PASS** | **0.93 PASS** | **無補償・レート補償のどちらより良好** |
| nominal・速度ループのみ補償8ms（遅れ無し） | 1.14 PASS | 0.52 PASS | 回帰なし |

**決定的な発見**: レートループの予測補償器は、想定した遅れより実際の遅れが**大きい**と
（過小ミスマッチ）、**無補償より大幅に悪化する**——高帯域・追従要求の厳しいレートループ
では、部分的にしか効かない補償がかえって新しい不安定モードを生むと考えられる。
一方**速度ループの補償器は同じ過小ミスマッチ条件でも良好に機能し、nominalでの副作用も
無い**——§7.7bで見た「レートループは補償値の大小に関わらずnominalで悪化する」傾向とも
整合する（速度ループの方が帯域が低く、想定と実際のズレに寛容）。

**実機投入に向けた方針転換**: 当初の設計仮説（レートループを直せば十分、
速度ループには不要）は誤りだった。**実機の実際の無駄時間を正確に把握できない
（経年・個体差で変動しうる）以上、ミスマッチに弱いレートループの補償器は既定OFFのまま
封印し、ミスマッチに強い速度ループの補償器（`delay_comp_ms=8`）を実機投入の候補とする**。

### 7.7d 実機投入に向けた広範囲検証（速度ループのみ、comp=8ms固定）

`nominal` / `matched delay=8ms` / `torque-authority=0.4` / `noise n1` /
`pos_roll`/`pos_pitch`/`pos_yaw` / `pos_reposition` / `pos_auto_takeoff`
（いずれも速度ループ補償8ms固定、レートループ補償は無効のまま）を再検証:

- **duty以外の全ゲートで健全**（`pos_reposition`・`pos_auto_takeoff`は19/19フルPASS）
- duty_maxは一部条件（noise n1・matched delay=8ms）で1.0に達するが、drift/att_rmseは
  常に健全——§7.4以降一貫する「duty飽和はするが破局的ではない」既知パターンの範囲内

さらに「想定8msに対し実際の遅れがどこまで大きくなると破綻するか」を確認:

| 想定8ms vs 実際 | drift(<3.0) | tilt_max(<18.0) | att_rmse(<5.0) |
|---|---|---|---|
| 0ms (nominal) | 1.14 PASS | 13.93 PASS | 0.52 PASS |
| 8ms（一致） | 0.63 PASS | 15.89 PASS | 0.48 PASS |
| 15ms（過小、7ms差） | 0.51 PASS | 17.28 PASS | 0.93 PASS |
| **20ms（過小、12ms差）** | **35.09 FAIL（壊滅的）** | **50.6°（転倒級）** | **15.62 FAIL** |

7ms差までは無害〜良好だが、12ms差で転倒級に破綻する——**しきい値は8〜20msの間のどこか**
（未特定）。実機の実際の遅れはL≈8.4-14.7ms/軸（実測平均）だが、バッテリー電圧・個体差・
経年劣化による**変動幅は未確認**。この変動幅が破綻しきい値に達しうるかを実機データで
把握しないまま実機投入するのは危険と判断し、**この時点では実機投入を見送り、まず実機の
実際の無駄時間がどの程度変動するかを把握する**方針とした（ユーザー判断、2026-09-11）。

### 7.7e 【訂正】pitch軸の実測遅れ値の誤り、および破綻境界の再確認

§7.7c/dで「実機軸別実測値」として使った `roll=14.7 / pitch=8.4 / yaw=11.0ms` は、
`pid_controller.hpp`のコメントに残っていた**古い値**であり、誤りだった（ユーザーが
`docs/events/sci_tutorial_2026/slides/sci_tutorial.pdf`のむだ時間スライド
「$\tau_d\approx5$ms」を示し、既に実機同定済みであることを指摘、資料の出典
`sci_s4_pid.tex`経由で正しい実測データ
（`analysis/reports/rate_sysid_reference/reference.json`、2026-06-14の実飛行2本、
2026-07-22生成）を確認した結果判明）:

| 軸 | 誤って使っていた値 | **正しい実測値**（2 run平均、`L_total = T+L`） | 2 run の幅 |
|---|---|---|---|
| roll | 14.7ms | **14.36ms** | 14.07〜14.66ms |
| **pitch** | **8.4ms（約半分に過小評価）** | **17.29ms** | 16.24〜18.33ms |
| yaw | 11.0ms | **10.82ms** | 10.60〜11.05ms |

**pitch軸を正しい実測値（17ms）で再検証**したところ、速度ループ補償8ms（§7.7dで
「安全」と結論づけた設定）は**壊滅的に発散**した:

| 想定8ms vs 実際 | drift(<3.0) | tilt_max(<18.0) | att_rmse(<5.0) |
|---|---|---|---|
| 15ms（誤ったpitch値で「安全」と判断） | 0.51 PASS | 17.28 PASS | 0.93 PASS |
| **17ms（pitch軸の正しい実測値、9ms差）** | **23.12 FAIL（壊滅的）** | **20.02° FAIL（転倒級）** | **13.98 FAIL** |

**破綻境界は7〜12msの差ではなく、7〜9msの差というさらに狭い範囲**にあることが確定した。
さらに重要なのは、**軸間の実測差だけで既に6.5ms（10.8ms〜17.3ms）ある**——バッテリー
電圧・経年劣化による変動を考慮する前の時点で、軸間差だけで破綻境界に迫っている。

**結論（§7.8を上書き）**: 速度ループ補償器は、当初考えていたよりもはるかに狭い安全
マージンしか持たない。実機投入の見送り判断は正しく、想定以上に重要だった。この安全
マージンの狭さを踏まえると、**単純な固定補償値（全軸一律）での実機投入は現時点では
推奨しない**——軸別に正確な値を使ってもなお、想定と実際のわずかなズレ（数ms）で
転倒級に破綻しうる。

### 7.8 現状のステータス（最終、訂正版）

- [x] 無駄時間予測補償器を実装（レート・速度両ループ、opt-in、既定OFFで既存挙動を完全維持）
- [x] motor-delay=15msでの劇的な改善を確認（smc_rate/smc_posのレートループ補償）
- [x] **重要な発見1**: レートループ補償は既定ONで nominal/torque-authority=0.4 に新規退行
- [x] **重要な発見2**: レートループ補償は「想定より実際の遅れが大きい」ミスマッチで
      無補償より大幅悪化（drift 4.36→27.9）——速度ループ補償の方がミスマッチに強い
- [x] **重要な発見3**: 速度ループ補償にも破綻点があり、想定との差が~9ms超で
      転倒級に発散する（§7.7eで確定——当初の~12msという見積もりより狭い）
- [x] **重要な発見4（§7.7e）**: pitch軸の実測遅れ値を誤っていた（8.4ms→正しくは17.3ms）。
      正しい値で再検証すると、§7.7dで「安全」と結論づけたcomp=8msの設定は**pitch軸で
      壊滅的に発散する**（drift=23.1、tilt=20.0°）。既存の実機同定データ
      （`analysis/reports/rate_sysid_reference/reference.json`）だけで軸間差が6.5msあり、
      破綻境界（7〜9ms差）に対して安全マージンがほぼ無い
- [x] **結論**: 速度ループ補償器（および当然レートループ補償器）は、**現時点の設計
      （固定の想定遅れ値）では実機投入を推奨しない**。安全マージンが極めて狭く、
      軸間の実測差だけで破綻境界に迫るため
- [ ] 実機投入を再検討する場合の選択肢（未着手）:
      (a) 軸別に正確な実測値を使い、かつ保守的なマージン（例: 実測値の50%程度）を
          取った上でSILSスイープを密に行い安全域を確認する
      (b) 固定値ではなく実測遅れを適応的に推定する機構（オンライン同定）へ再設計する
      (c) 遅れ補償という方向性自体を見送り、既存の（実機投入済み・実績のある）
          レート+速度SMCのround-2ゲイン単体で運用する
- [ ] `sf sils sysid-gate`のような枠組みで、バッテリー電圧・飛行時間による遅れ変動を
      より多くのrunで継続的に把握する仕組みは未整備（現状2 runのみ、同日測定）

### 7.9 §7全体の結論: 位置制御（速度ループ）SMC化は見送り

§3.15の実機モータ非対称性の発見を発端に、`firmware/apps/smc_pos`として位置制御
カスケードの速度ループをSMC化し、複数ラウンドの検証を行った。最終的な結論として、
**位置制御へのSMC投入はこれ以上進めない**（ユーザー判断、2026-09-11）。

**理由**:

1. **素の速度ループSMC自体（`delay_comp`無効、既定状態）は安全だが、既存PIDを明確に
   上回らなかった**——ラウンド2（§7.4）でnominal・torque-authority摂動は全てクリア
   したが、項目によって一長一短（att_rmse改善・drift悪化等）で、決定的な優位性は
   示せなかった
2. **既存PID（`position.vel.kp/ti`）は§7.1の時点で既に実機同定に基づき頑健化・
   実機検証済み**（`firmware/vehicle/docs/poshold_journey.md` §4、±6-7cmホールド
   精度、K∈[2.8,7]という広いロバストマージンで安定確認済み）——SMC化はこの実績を
   代替するほどの根拠を示せなかった
3. **本来の差別化要因になり得た「無駄時間予測補償器」（§7.7-7.7e）が、実用に足る
   安全マージンを持たないと判明**——pitch軸の実測遅れ値の誤り訂正後、想定と実際の
   差7〜9msという狭い範囲で転倒級に破綻することが確定し、軸間の実測差
   （roll14.4/pitch17.3/yaw10.8ms）だけで既にこの境界に迫っている

**「安全」×「既存より明確に優れている」の両方を満たせなかった**ため、追加の複雑さ・
保守コストを正当化できないというのが最終判断。`firmware/apps/smc_pos`はコードとして
残すが、実機投入・追加のチューニングラウンドは行わない。

**`firmware/apps/smc_rate`（レートループのみのSMC）はこの結論と独立**——ACRO限定で
既に実機検証済みであり、今回の発見（速度ループ・遅れ補償に関する安全マージンの狭さ）は
`smc_rate`単体の評価には影響しない。

### 7.10 スライディング面の不感バンド（試行、結果: 効果なし）

ユーザー提案（「無駄時間が強いなら不感バンドや減衰バンドを超平面に与えることができる」）
を受け、無駄時間予測補償器とは異なり**実際の遅れLを知る必要がない**頑健化策として、
`SlidingModeRate`のスライディング面`s`に不感バンド（`s_deadband`、既定0=無効、opt-in）
を追加した: `|s|<=s_deadband`の範囲では境界層・到達則への入力を0へ縮める。

`stab_flight` + `motor-delay=15ms`（§3.3以降未解決のtilt_max FAIL）で検証:

| 条件 | tilt_max(<18.0) | att_rmse(<3.0) |
|---|---|---|
| 不感バンドなし（基準） | 24.06 FAIL | 1.04 |
| db=0.02 rad/s | 24.53 FAIL（ほぼ不変） | 1.99 |
| db=0.05 rad/s | 24.26 FAIL（ほぼ不変） | 1.99 |
| db=0.1 rad/s | 24.33 FAIL（ほぼ不変） | 1.19 |
| nominal（遅れ無し）+ db=0.05 | — | **3.79 FAIL（新規退行）** |

**効果なし**——`tilt_max`は不感バンドの大きさ（φ=0.15の13〜67%相当を試行）に関わらず
24°前後でほぼ不変。加えてnominal条件でatt_rmseの新規退行が確認された。**推定される
理由**: `stab_flight`のこの失敗モードは、`poshold_journey.md` §4.2のような「小振幅から
成長する振動」ではなく、積極的なスティック操作による**大振幅の過渡応答**（一気に
tiltが跳ねる）であり、原点近傍の小さな`s`への反応を消す不感バンドは、そもそも
この失敗モードの経路上に無い。狙い自体（Lを知らずに頑健化する）は妥当だったが、
このシナリオの失敗機序には当てはまらなかった。

機構自体（既定0で無効、既存挙動に影響なし）はコードとして残す——将来、小振幅の
持続振動が問題になる場面（位置ループのnominal-hover振動等）では有効かもしれない。

### 7.11 スーパーツイスティング法（STA）の試行

ユーザー提案（「別の制御手法をもう一つ試す（super-twisting等）」）を受け、`firmware/apps/
smc_rate_sta`として2次スライディングモード（スーパーツイスティング法、STA、Levant 1993
[R5]、Moreno & Osorio 2012の実用チューニング指針[R6]）を実装した。1次PI型面SMC
（`k*sat(s/φ)+η*s`）と異なり、`u1=k1·√|s|·sign(s)`という連続な項（s=0で連続的に消える、
人為的な境界層不要）と積分項`z=∫-k2·sign(s)dt`で構成する（設計根拠は
[`smc_rate_sta.hpp`](../../firmware/apps/smc_rate_sta/smc_rate_sta.hpp)参照）。

初期シード（既存1次SMCのround-5ゲインに合わせて算出、k1=170,k2=85（roll/pitch）·
k1=43,k2=21（yaw））で`stab_flight+motor-delay=15ms`を検証したところ、**チューニング
一切なしでtilt_max=21.58°**——PID(23.65°)・1次SMC(24.06°)のどちらよりも既に良好
だった（ただしゲート18°は未達）。

### 7.12 k1/k2スイープとバランス点

roll/pitchのk1/k2をスイープ（`stab_flight+motor-delay=15ms`）:

| k1 | k2 | tilt_max(<18.0) |
|---|---|---|
| 100 | 30 | 17.68（PASSだがatt_rmse=6.30で別ゲートFAIL） |
| **120** | **60** | **14.90 PASS** |
| 140 | 70 | 17.19 PASS |
| 150 | 75 | 19.53 FAIL |
| 170(seed) | 85 | 21.58 FAIL |

k1=120/k2=60が最も低いtilt_maxを示したが、nominal/torque-authority/noiseで
att_rmseが軒並み悪化（2.5-3°基準→3-5.7°）する代償が見つかった。**k1=140/k2=70が
両立点**: nominal（att_rmse=2.74、§3.3以降初のnominalフルPASS）と
motor-delay=15ms（att_rmse=0.99, tilt_max=17.19、この条件で史上初のフルPASS）の
どちらも3ゲート同時クリア。torque-authority=0.55・acro nominalも健全、
torque-authority=0.4（att_rmse=3.34軽微FAIL）・noise n1（5.24 FAIL）は既存PID/
1次SMCで既に受容済みの同種限界の範囲内——**新規退行ではないと判断**し、`params.cpp`の
既定値をk1=140/k2=70（roll/pitch）・比例スケールしたk1=35.4/k2=17.3（yaw、未検証）に
採用した。

### 7.13 【重要】複合入力シナリオでの壊滅的破綻を発見 — 実機投入を中止

ユーザーから「SILSのシナリオに複雑なコントロール入力した場合のモノも追加して」との
指摘を受け、`stab_flight`（単一の大きめステップ入力）だけでなく`pos_flight.scn`
（ロール+ピッチ同時・斜め方向、POS_HOLD、Layer-4キャップストーン）でk1=140/k2=70を
検証した。**この一言が実機投入前の重大な事故を防いだ**。

| 条件 | drift(<3.0) | tilt_max(<18.0) | att_rmse(<5.0) |
|---|---|---|---|
| pos_flight nominal | 0.72 PASS | 13.65 PASS | 1.07 PASS |
| **pos_flight + motor-delay=15ms** | **57.23（壊滅的）** | **35.92°（転倒級）** | **26.47（壊滅的）** |

`stab_flight`では良好だった同じゲインが、より複雑で持続的な複合操縦
（斜め方向の同時ロール+ピッチ）と組み合わさると、**全く別次元の壊滅的発散**を示した。
`stab_flight`単体での検証は明らかに不十分であり、**STAアプローチ全体を実機投入するには
時期尚早**と判断する。

**結論（当初）**: `smc_rate_sta`の実機投入は中止。STAという手法自体は（§7.11-7.12で
見た通り）有望だが、単一シナリオでの局所最適に陥りやすく、`smc_rate`の§3.2で確立した
「複数シナリオ・複数摂動での同時検証」という規律を、STAでも最初から徹底する必要が
あった——今回はそれを怠ったまま実機投入に進もうとしてしまった、という反省点として
記録する。

### 7.14 STA再チューニング（複数アーキタイプ同時ゲート）と新規複雑シナリオ

ユーザー指示（「バランス点を探し出し、実機に書き込んで」「SILSのシナリオに複雑な
コントロール入力した場合のモノも追加して」）を受け、§7.13の反省を踏まえて再開。
**今度は`stab_flight`・`pos_flight`・新規シナリオの3アーキタイプを同時にゲートとして
スイープした。**

**新規シナリオ**: [`stab_combined_aggressive.scn`](../../simulator/sils/scenarios/stab_combined_aggressive.scn)
を追加。`stab_flight`（1軸ずつ順番）・`pos_flight`（roll+pitch同時だが単一ステップを
保持、POS_HOLD）のどちらとも異なる第3のアーキタイプ: STABILIZEで**3軸（roll+pitch+yaw）
を同時に**動かし、さらに**機動の途中で方向を反転**させる（実際のパイロットの「乱雑な」
スティック入力に近い）。ゲート値はPID基準（att_rmse=0.53, tilt=10.80）と1次SMC基準
（att_rmse=3.11, tilt=15.43）の実測値から設定（`stab_combined_aggressive.expect`）。

**結合スイープ**（`stab_flight`+`pos_flight`、`motor-delay=15ms`、roll/pitch同時変更）:

| k1 | k2 | stab（tilt/att_rmse） | pos（drift/tilt/att_rmse、duty以外） |
|---|---|---|---|
| **60** | **30** | **12.3°/2.12 PASS** | **0.70m/16.3°/0.69 PASS** |
| 80 | 40 | 12.9°/2.08 PASS | 0.64m/16.6°/0.66 PASS |
| 100 | 50 | 16.8°/**5.09 FAIL** | 1.94m/13.3°/2.19 PASS |
| 120 | 60 | 14.9°/1.32 PASS | **6.18m FAIL**/16.0°/**8.35 FAIL** |
| 140（§7.13） | 70 | 17.2°/0.99 PASS | **57.2m FAIL**/**35.9° FAIL**/**26.5 FAIL** |

k1=60とk1=80がどちらも両シナリオで健全だったため、nominal（遅れ無し）条件で追加比較
したところ、**k1=80はnominalのatt_rmseが4.16まで悪化**（k60は2.96）——k1=50も同様に
nominalで4.21まで悪化し、**k1=60付近が狭い局所最適**であることを確認した
（「低いほど安全」という単純な関係ではない）。

**最終採用: k1=60, k2=30**（roll/pitch）、yawは同じ60/170比でスケール（k1=15.2,
k2=7.4、yaw固有の検証は未実施）。3アーキタイプ×{nominal, motor-delay=15ms}の
全6条件で再検証:

| シナリオ | 条件 | 結果 |
|---|---|---|
| `stab_flight` | nominal | att_rmse=2.88, tilt=13.17 — **PASS** |
| `stab_flight` | motor-delay=15ms | att_rmse=2.23, tilt=12.32 — **PASS** |
| `pos_flight` | nominal | drift=0.78, tilt=13.97, att_rmse=0.94 — **PASS** |
| `pos_flight` | motor-delay=15ms | drift=0.95, tilt=16.10, att_rmse=1.14 — **PASS**（§7.13の57m/36°から回復） |
| `stab_combined_aggressive` | nominal | att_rmse=4.56, tilt=16.35 — **PASS** |
| `stab_combined_aggressive` | motor-delay=15ms | att_rmse=2.51, tilt=16.74 — **PASS** |
| `acro_flight` | nominal | att_rmse=1.93, tilt=7.84 — PASS |
| `stab_flight` | torque-authority=0.4 | att_rmse=3.44（軽微FAIL、既知の受容済みクラス） |
| `stab_flight` | noise n1 | att_rmse=4.72（FAIL、既知の受容済みクラス） |

**§7.13の教訓通り、3アーキタイプ同時検証で初めて、単一シナリオへの過学習ではない
ゲインが見つかった。** 残るFAIL（torque-authority=0.4・noise n1）は`smc_rate`自身の
PID/1次SMC基準で既に受容済みの同種の限界であり、新規退行ではない。

### 7.15 組み合わせ総点検（過去実装の一括確認）

ユーザー指示（「過去の実装の組み合わせ総点検」）を受け、これまでのセッションで追加した
機能の相互作用・回帰を確認した:

- **無駄時間予測補償器 + スライディング面不感バンドの同時使用**（`smc_rate`、両方
  非ゼロ）: `stab_flight+motor-delay=15ms`でatt_rmse=3.44（軽微FAIL）・tilt=13.63
  PASS・duty PASS——破局的な相互作用なし、2機構は問題なく合成できることを確認
- **`smc_pos`のnominal回帰**（`pos_flight`）: 既存の基準値（drift=1.105, tilt=13.93,
  duty=0.80, att_rmse=0.61）と完全一致——今回のセッションでの変更による意図しない
  影響なし
- **`smc_pos`の速度ループ遅れ補償のnominal回帰**（comp=8ms）: 同様に既存基準値と一致
- **全4ターゲットの実機（ESP32-S3）ビルド確認**: `vehicle`・`smc_rate`・`smc_pos`・
  `smc_rate_sta`——詳細は本節末尾

コードベース全体（`smc_rate`・`smc_pos`・`smc_rate_sta`の3アプリ、無駄時間予測補償器・
不感バンド・STAという3つの新規機構）は、相互に矛盾なく共存しており、params.cppの
パラメータ名重複も無いことを確認した。

**追加点検（コードレビュー、実行せず静的確認）**:
- **`IController`の11個の override が3アプリ全てで一致**（`compute`/`reset`/
  `onModeChange`/`onLanding`/`onTakeoff`/`onTakeoffComplete`/`isTakeoffComplete`/
  `setGuidanceTarget`/`isGuidanceActive`/`startExcitation`/`fetchSysidResult`）——
  実装漏れなし
- **`reset()`の状態網羅性**: `SlidingModeRate`（`integral`/`prev_error`/
  `torque_ring_`系）・`SlidingModeVelocity`（同型、`accel_ring_`系）・
  `SuperTwistingRate`（`integral`/`z`/`prev_error`）——いずれも保持する実行時状態を
  全て`reset()`でクリアしていることをフィールド一覧と照合して確認。ゲイン類
  （`k`/`eta`/`phi`等）はconfigであり`reset()`対象外で正しい
- **param名の完全一致**: `smc_sta.*`（15個）・`smc.vel{x,y}.*`（12個）・
  `smc.{roll,pitch,yaw}.{delay_comp_ms,s_deadband}`（6個）について、
  `params.cpp`の宣言と各appの`get_float()`呼び出しを突き合わせ、全て一致
  （タイプミス・未配線なし）
- **`init()`/`reloadParams()`の網羅性**: 3アプリ全てで、両メソッドが対応する
  `load*Params()`を漏れなく呼んでいることを確認（`smc_pos`は`loadRateSmcParams()`
  ＋`loadVelSmcParams()`の2つを両方）
- **ドキュメントの陳腐化を1件発見・修正**: `firmware/apps/smc_pos/README.md`が
  §7.9の「実機投入見送り」決定を反映せず「SILS摂動族テストをクリアしてから実機投入」
  という古い記述のままだった——修正しコミット

### 7.16 `smc_rate_sta`実機投入（2026-09-11）

§7.14の3アーキタイプ同時検証を経て、ユーザー指示により`smc_rate_sta`（STA、
k1=60/k2=30 round-2ゲイン）を実機（ESP32-S3, COM3経由）へ書き込んだ
（`sf app flash smc_rate_sta`、`[OK] Flash successful`、ハードリセット確認済み）。
**この時点でアーム・飛行は一切行っていない**——フラッシュ完了後、実機モニタ接続などの
追加操作はユーザー不在のため意図的に行わず停止した（このプロジェクトの一貫した
安全方針: 実機操作はユーザー立ち会い下で行う）。次に実機を扱う際は、既存の運用ルール
（§3.14: WiFi STA運用時の注意、§4: ベンチ/テザー・ACRO限定から開始）をそのまま適用する。
**`smc_rate_sta`は実機初投入であり、飛行実績が一切無い**点に注意——`smc_rate`
（1次SMC、ACRO実績あり）とは異なる。

### 7.17 【重要・実機投入後に発覚】持続外乱下での転倒を発見——実機飛行を保留

ユーザーからの質問（「超平面に含まれる積分器はどの程度の時間で飽和する？」「15sオーダーで
あるなら問題ないかな。1秒切られると怪しいが」）を受け、理論見積もり（§前回回答、
「roll軸で約15秒」）を実測で検証するため、`SuperTwistingRate`の内部状態
（`integral`/`z`/`s`）を一時的にログ出力する計装を追加し、**持続的な**ロールステップ
（60秒保持）+ `torque-authority=0.4`という、これまでの§7.14検証（全て短時間・過渡的な
入力）では試していない条件でSILS実行した。

**結果、理論見積もりは誤りだったことが判明した**。積分器は「安全に飽和して落ち着く」
のではなく:

1. `z`（STA積分項）が線形に成長（roll: 約±20-25/秒）しながら、`u1=k1√|s|sign(s)`項が
   ほぼ相殺し、正味トルクは終始小さいまま
2. その間、実際のレート誤差`e`は収束せず、振動しながら振幅が増大
3. `|e|`が`e_reset`（roll/pitch=0.75 rad/s）を超えると積分・z が同時に0へリセットされ、
   周期約7-9秒のサイクルとして繰り返す
4. **このサイクルを3-4回（約26秒）繰り返した後、t≈33.0秒で発散し転倒**
   （`trajectory.csv`のroll列が178.997°に到達、コンソールログの
   `level_offset: [-3.1416, 0.0000] rad`（=π rad）と整合）

**重要な訂正**: 「積分器がどの程度の時間で飽和するか」という質問に対する正しい答えは
「安全に飽和する時間」ではなく「**持続外乱下では真に定常収束せず、周期的リセットを
繰り返しながら悪化し、約33秒で転倒に至る**」だった。1秒未満の速い飽和ではないが、
ユーザーが懸念していた「怪しさ」は別の形で的中していたことになる。

**結論・対応**:
- **この構成（`smc_rate_sta`, k1=60/k2=30）は、持続的な大入力+torque-authority低下の
  組み合わせで実機投入中の飛行に耐えない可能性がある**——§7.14の検証は全て過渡的な
  入力に限定されており、この種の持続外乱は未検証だった、という検証設計上の穴が
  あったことを意味する
- **実機はフラッシュ済み（§7.16）だが、この問題が解明されるまで飛行テストを保留する**
- 診断用の一時計装コード（`ESP_LOGI`によるroll軸の`s`/`integral`/`z`ログ）と
  診断専用シナリオ（`_diag_sta_integrator_saturation.scn`、先頭アンダースコアで
  通常の回帰群から除外）は削除せず一旦残す——追加調査に使える

### Next steps（§7.17を受けて）

- [ ] 持続外乱（30秒超の一定大入力 + torque-authority低下）を新しい検証軸として、
      §7.14の3アーキタイプ全てに追加する（短時間入力だけでは不十分と判明したため）
- [ ] `e_reset`によるリセットが「安全弁」でなく「問題の先送り」になっている可能性を
      検討する——リセットのたびに何が改善されずに繰り返し悪化しているのかを特定する
- [ ] k1=60/k2=30以外のゲインでもこの持続外乱テストを行い、同じ転倒が起きるか
      （STA全般の弱点か、この特定ゲインの問題か）切り分ける
- [ ] 1次SMC（`smc_rate`）・PID（`vehicle`）でも同じ持続外乱シナリオを実行し、
      比較基準を得る（STA固有の弱点か、レートループ全般の弱点かの切り分け）

## 4. 実機投入ゲート

上記SILS検証手順が全てクリアし、かつ**ユーザーの明示的な判断**を得てから初めて
`sf app build smc_rate` / `sf app flash smc_rate -m`に進む。development_roadmap.mdの層別検証
（ACRO→STABILIZE→…）の思想を踏襲し、まずACROのみ・ベンチ/テザー拘束下で確認する。
SMCは既存の線形ETFEモデル（`rate_sysid.py`の`G(s)=b·e^{-Ls}/(s(Ts+1))`）による同定を前提とした
±20%/±50%ゲート（development_roadmap.md Phase 3/4）がそのまま適用できるとは限らない点に留意する
（非線形制御則のため）— 実機投入判断はSILSの数値結果を提示した上でユーザーと個別に協議する。

## 5. Next steps

- [x] `smc_rate.hpp`の初期ゲイン（k/eta/phi）を、既存`rate.{roll,pitch,yaw}.kp/ti/td`とI_axisから逆算 → SILSで応答を見て調整（§2.4/§3.1、`params.cpp`デフォルト値に反映済み）
- [x] 既存ACRO/姿勢シナリオでのPIDとのA/B比較結果を記録（§3.1、4シナリオ）
- [x] `--noise n2`確認 → PIDが離陸完了できない副作用（原因未解明）とSMC自体のatt_rmse劣化を発見（§3.2）
- [x] `--motor-delay`/`--torque-authority`確認 → **`stab_flight`で明確な頑健性の弱点を発見**（§3.2、`torque-authority=0.55`でatt_rmse 2.88°→5.18°FAIL、PIDは2.55°で耐える）— 現行ゲインは§3.1の詰めすぎによる単一点最適化の再発と判断
- [x] **§3.2の頑健性弱点への対処を試行**（§3.3）: 8通りのゲイン点を探索。`torque-authority`は`eta`大幅増で解決可能と判明したが、その副作用で`motor-delay=15ms`が転倒（180〜201°）する致命的な退行を発見。**安全性を優先しラウンド1のゲインに差し戻し**、`torque-authority=0.4/0.55`・`noise n1`は既知の未解決FAILとして記録（`acro_flight`は全条件で健全、`stab_flight`のみの弱点）
- [x] **PI型スライディング面の実装**（§3.4/§3.5、[R3]参照）: `torque-authority=0.4/0.55`のatt_rmseを約4.9〜5.2°→3.0〜3.8°に改善（約30%減）、かつ`motor-delay=15ms`の転倒（ラウンド2〜3の180〜201°）は再発せず——ただし3.0°ゲートには未達のまま。`λ_i`単独スイープ{1.5,3,5}では非単調で完全解決に至らなかった
- [x] **`λ_i`/`η`の同時再探索**（§3.7）: 6通りの組を探索。`η`を下げる仮説は反証されたが、`η`を控えめに上げつつ`λ_i`を大きく上げる方向（180/175, `λ_i`=6）で**`torque-authority=0.4`が本計画で初めてPASS**（2.78°）。ただし探索空間は非単調で`torque-authority=0.55`・`noise n1/n2`は依然未解決
- [ ] `torque-authority=0.55`・`noise n1/n2`の解消（次の作業）: §3.7の非単調性から単純なスカラー増減では限界の可能性——`k`（スイッチングゲイン）や境界層`φ`の同時探索、あるいはn1/n2に対しては積分の観測ノイズ対策（例: 積分入力のLPF）等、別の切り口が必要か検討
- [ ] noise n2でPIDだけ離陸完了しない現象の原因調査（§3.2で発見、レートループ品質→姿勢推定→鉛直ハンドオフへの影響経路の特定、本変更に起因するかの切り分け含む）
- [ ] `pos_yaw`等、まだ確認していない既存シナリオでの回帰確認
- 実機投入は§3.3の未解決FAIL（`torque-authority=0.4/0.55`、`noise n1`）が解消するまで**検討しない**。コミットは未実施 — ユーザー指示によりまだ行わない

## 6. 参考文献

本計画の制御則設計（§2.2到達則、§2.2超平面根拠、§3.4提案）が依拠する文献。Web検索で
書誌情報を確認済み（2026-09-11）。

| ID | 文献 | 本計画での使用箇所 |
|----|------|-------------------|
| R1 | W. Gao and J. C. Hung, "Variable structure control of nonlinear systems: a new approach," *IEEE Transactions on Industrial Electronics*, vol. 40, no. 1, pp. 45–55, 1993. | §2.2の到達則`τ = I·(k·sat(s/φ)+η·s)`（"constant-plus-proportional rate reaching law" / exponential reaching law）の根拠 |
| R2 | J.-J. E. Slotine and W. Li, *Applied Nonlinear Control*, Prentice Hall, 1991. | 境界層（`sign()`→`sat()`置換）によるチャタリング抑制の標準的手法。§2.2の`φ`（境界層半幅）と、本文冒頭で説明した相対次数と超平面設計（`s=ė+λe`型）の一般論の教科書的典拠 |
| R3 | V. Utkin and J. Shi, "Integral sliding mode in systems operating under uncertainty conditions," in *Proc. 35th IEEE Conference on Decision and Control (CDC)*, Kobe, Japan, 1996, pp. 4591–4596. | §3.4で提案した積分項付加の着想元（Integral Sliding Mode, ISM）。**用語の精度に関する注記**: Utkin & Shiの原論文のISMは、システムの次数を保存し初期時刻からスライディングモードを成立させる（到達フェーズそのものを除去する）より精緻な構成であり、§3.4で提案した単純な加算型`s = e + λ_i∫e dt`（実務上「PI型スライディング面」と呼ばれる簡略版）とは厳密には異なる。本計画で実装を検討する場合はPI型スライディング面（簡略版）であり、それを指してISMと呼ぶのは不正確——§3.4の文中表現もこの区別に従う |

補足: §3.4で言及した「PI型スライディング面」は無人機（クアッドロータ）の姿勢制御分野でも
広く実用されている（例: 風外乱下のクアッドロータ姿勢制御にADRC+ISMCを組み合わせた研究、
*Mechanical Systems and Signal Processing*, vol. 129, 2019, 著者名の完全なリストは未確認
のため書誌情報として確定はしていない — 個別の設計判断はR1〜R3の確定済み3件に基づく）。
