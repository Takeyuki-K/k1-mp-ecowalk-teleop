# k1_ecowalk_ros

[k1-mp-ecowalk-public](https://github.com/Takeyuki-K/k1-mp-ecowalk-public) の**学習済み歩行ポリシー**を MuJoCo 上で ROS 2 から動かし、rosbridge 経由のブラウザ UI から速度指令を出すパッケージ。

- 既定ポリシー `policy_set:=v56`（**v5.6.3** = k1-mp-ecowalk-public の main / ブランチ `v5.6.3`、`k1_mp_gait56/`：左右対称の歩行・走行、その場旋回、停止1秒後の足揃え、厳密な静止摩擦、走行旋回の自動減速 v·|ω| ≤ 2.5 m/s²）。`policy_set:=v55` で v5.5（`k1_mp_gait55/`）: 歩行 ⇄ 走行の自動切替 + 歩行/走行中の旋回 + その場旋回 + 旋回時の自動減速 + 急停止
- `policy_set:=v3`: 旧 v3 歩行旋回ポリシー（`k1_mp_turn/`、1.35 m/s まで）
- **再実装なし**: 学習時の環境クラス（v5.5 は `K1Walk3Batch` / `K1Run4Batch` とゲートマネージャ `gait55.py`）を n=1 でそのまま使用
- 学習範囲外の指令は黙って通さず、`/k1/status` と UI に明示

## 構成

```
browser (index.html + roslib) ──ws:9090── rosbridge ── /cmd_vel ──► k1_sim (MuJoCo + policy, 50 Hz)
                                                      ◄── /k1/status, /joint_states, /odom, /imu/data, /clock, TF
                                                                          └─► robot_state_publisher ─► RViz
```

| I/F | 型 | 内容 |
|---|---|---|
| `/cmd_vel` (sub) | Twist | `linear.x` 前進 [m/s], `angular.z` 旋回 [rad/s]。`linear.y` は未学習のため無視 |
| `/k1/start` `/k1/stop` `/k1/reset` | std_srvs/Trigger | stop は学習済みの安全停止（減速 → 足踏み → 両足揃え） |
| `/k1/brake` | std_srvs/Trigger | v5.5 の急停止（走行: 4 m/s² で 1.8 m/s まで制動 → 歩行へ → 停止） |
| `/joint_states` | JointState | 23 関節 + 受動 MP 関節 2 |
| `/odom`, TF `odom→pelvis` | Odometry | **真値**（VIO/LIO の評価用。入力に使わないこと） |
| `/imu/data` | Imu | MuJoCo の imu サイト（quat / gyro / accel） |
| `/clock` | Clock | シミュレーション時刻（reset でも巻き戻らない） |
| `/k1/status` | String(JSON) | v/ω の指令・参照・実測、脚電力推定、Kp スケール、足裏力、学習範囲フラグ |

### v5.5 の指令の扱い

| 指令 | 動作 |
|---|---|
| v = 0, \|ω\| ≤ 1.0 | 足踏み / その場旋回 |
| 0.3 ≤ v ≤ 1.65 | 歩行（旋回時は v·\|ω\| ≤ 1.0 m/s² に自動減速） |
| v ≥ 1.8（旋回考慮後） | 走行に自動切替、5.0 m/s まで（旋回時は v·\|ω\| ≤ 3.0 m/s² に自動減速、体を内側に傾ける） |
| v < 0, 横移動 | 未学習（0 にクランプ / 無視） |

### v3 の指令の扱い（学習分布 = `k1env_turn.py _new_motion`）

| 指令 | 扱い |
|---|---|
| v = 0, \|ω\| ≤ 1.0 | 足踏み / その場旋回（学習済み） |
| 0.30 ≤ v ≤ 1.0, \|ω\| ≤ 1.0 | 歩行旋回（学習済み） |
| 1.0 < v ≤ 1.35, ω = 0 | 直進（学習済み） |
| 0 < v < 0.30 / v > 1.0 で旋回 | **送信するがフラグ表示**（学習分布外） |
| v < 0, v > 1.35, \|ω\| > 1.0 | クランプ（後退は未学習） |

デッドマン: `/cmd_vel` が 0.5 s 途絶 → v=ω=0（足踏み継続）。指令 0 が 5 s 続く → 安全停止。

## Docker（x86、ホストが Ubuntu 22.04 / Humble のままで使う）

```bash
git clone https://github.com/Takeyuki-K/k1-mp-ecowalk-teleop.git ~/k1-mp-ecowalk-teleop   # private: GitHub の認証が必要
cd ~/k1-mp-ecowalk-teleop/k1_ecowalk_ros
docker build -f docker/Dockerfile -t k1-ecowalk:jazzy .     # 初回 10〜20 分（PyTorch CPU 版のダウンロード）
./docker/run.sh                                              # → ブラウザで http://localhost:8080
```

### ポリシーのバージョン指定（`ECOWALK_REF`）

イメージには k1-mp-ecowalk-public が `/opt/k1-mp-ecowalk-public` に clone されて焼き込まれる。どのブランチ・タグを使うかは `--build-arg ECOWALK_REF=<ブランチ or タグ>` で指定する（省略時は `main`）。

```bash
docker build --build-arg ECOWALK_REF=v5.6.3 -f docker/Dockerfile -t k1-ecowalk:jazzy .
```

| `ECOWALK_REF` | 含まれるポリシー | 対応する `policy_set` |
|---|---|---|
| `main`（既定）/ `v5.6.3` | `k1_mp_gait56/`（2026-10-08 以降の main = v5.6.3） | `v56`（既定） |
| `v5.6.2` / `v5.6.1` | `k1_mp_gait56/`（旧版） | `v56` |
| `v5.5-turn-run` | `k1_mp_gait55/` | `v55` |

- **注意: Docker のキャッシュ**。`ECOWALK_REF=main` のまま再ビルドしても、clone の行は前回のキャッシュが使われ、public 側の main の更新は入らない。最新の main を取り込むときは `--no-cache`、またはブランチ・タグを明示する（`ECOWALK_REF` の値が変われば clone 以降の層だけ作り直される）
- 古い main が入ったイメージで既定の `policy_set:=v56` を起動すると `FileNotFoundError: k1_mp_gait56 not found under /opt/k1-mp-ecowalk-public` で `k1_sim` が落ちる。上記のどちらかで作り直す
- イメージ名は `run.sh` の既定 `k1-ecowalk:jazzy` に合わせる（別名にしたら `IMAGE=<名前> ./docker/run.sh`）
- 再ビルドせずに切り替える: ホストのクローンで `git checkout <ブランチ>` してから `ECOWALK_DIR=~/k1-mp-ecowalk-public ./docker/run.sh`（ホストのクローンがイメージ内のものより優先される）
- ベース: 公式 `osrf/ros:jazzy-desktop`（Ubuntu 24.04、Python 3.12）。numpy は apt の 1.26 のまま、MuJoCo と CPU 版 PyTorch は venv（`/opt/k1venv`、system site-packages 参照）に入れる
- `docker build` の最後で REPORT_TURN.md の数値と照合するテストが走る。ここで失敗したらイメージは作られない
- `./docker/run.sh k1-launch viewer:=false` ビューアなし / `./docker/run.sh k1-launch rviz:=true` RViz あり / `./docker/run.sh bash` シェル
- 自分のクローンを使う（学習し直したポリシーなど）: `ECOWALK_DIR=~/k1-mp-ecowalk-public ./docker/run.sh`
- **DDS の分離**: コンテナは `ROS_DOMAIN_ID=77`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`。ホストの Humble や同じ LAN の実機 AMR に `/cmd_vel` が流れないようにするため。ホストから見たい場合は同じ値を設定する（Humble ⇄ Jazzy 間の通信は保証されないので、基本はコンテナ内で `docker exec` する）
- GPU: NVIDIA Container Toolkit があれば `--gpus all`、なければ `/dev/dri`（Intel/AMD）。どちらもなければソフトウェア描画（重い場合は `viewer:=false`）

## セットアップ（Ubuntu 24.04 / ROS 2 Jazzy）

```bash
sudo apt install ros-jazzy-rosbridge-server ros-jazzy-robot-state-publisher ros-jazzy-rviz2
git clone https://github.com/Takeyuki-K/k1-mp-ecowalk-public.git ~/k1-mp-ecowalk-public
mkdir -p ~/k1_ecowalk_ws/src
git clone https://github.com/Takeyuki-K/k1-mp-ecowalk-teleop.git ~/k1_ecowalk_ws/src/k1-mp-ecowalk-teleop

# ROS の Python に mujoco / torch を入れる。apt の numpy を壊さないよう venv を推奨
python3 -m venv --system-site-packages ~/venv/k1 && source ~/venv/k1/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cpu   # CPU 版（CUDA 版は数 GB）
pip install mujoco==3.14.0 scipy

cd ~/k1_ecowalk_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
```

## 実行

```bash
source ~/venv/k1/bin/activate && source /opt/ros/jazzy/setup.bash && source ~/k1_ecowalk_ws/install/setup.bash
ros2 launch k1_ecowalk_ros teleop_sim.launch.py            # viewer:=false rviz:=true など
```

ブラウザで `http://localhost:8080`（別 PC からは `http://<simPCのIP>:8080`）。パッドをドラッグ、または W/A/S/D。
`teleop_twist_keyboard` もそのまま使える（後退キーは 0 にクランプされる）。

## テスト（ROS 不要）

```bash
python3 -m pytest test/test_sim_core.py -q
```

REPORT_TURN.md の数値（旋回 ω 誤差 ≤ 3 %、0.9 m/s 速度誤差、1.2 m/s からの安全停止）と照合し、ラッパーが学習時と違う入力を与えていないかを確認する。

## 注意（現場で詰まりやすい点）

- **numpy 衝突**: ecowalk は numpy 2.x 前提、Jazzy の apt パッケージ（cv_bridge 等）は numpy 1.26 でビルド。上記 venv（`--system-site-packages`）で隔離し、`colcon build` も venv を有効化したシェルで行う。
- **CPU / スレッド**: PyTorch の推論スレッド数はスレッドごとの設定。シミュレーションループ（別スレッド）で `torch.set_num_threads(1)` しないと v5.5 は 1 周期 54 ms（実時間の 1/3）まで遅くなった（修正済み、6–9 ms/周期）。`sim slower than real time` が出る場合は `viewer:=false`。
- **rosbridge のポート**: 別 PC から使う場合は 9090 / 8080 をファイアウォールで開ける。roslib は同梱済み（CDN 不要 = オフライン網で動く）。
- **IMU は 50 Hz**（制御周期の最終サブステップ値）。VIO には 200 Hz 以上が必要 → 次段階で物理サブステップ毎に出力する。
- MP 関節は実機 K1 に存在しない。sim2real 時は実機側の機構追加が前提。

## 次の段階（VIO / LIO マッピング）

1. 頭部カメラ（ステレオ）と LiDAR を MJCF に追加、オフスクリーンレンダリングで `/camera/*`, `/points` を配信
2. IMU を 200–400 Hz に（物理サブステップ単位）、カメラと同一クロックでタイムスタンプ
3. 障害物のある屋内シーン、`/odom` 真値で ATE/RPE 評価

**重要**: センサーを頭部に載せると質量・慣性が変わる。ポリシーはセンサー無しの質量で学習されているため、搭載後は「無修正で歩けるか」をこのテストで確認し、落ちるなら追加学習する。
