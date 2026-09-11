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
