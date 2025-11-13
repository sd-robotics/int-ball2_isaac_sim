# Int-Ball2シミュレータ (Isaac Sim)

[![README in English](https://img.shields.io/badge/English-d9d9d9)](./README.md)
[![日本語版 README](https://img.shields.io/badge/日本語-d9d9d9)](./README_JA.md)

![GitHub contributors](https://img.shields.io/github/contributors/sd-robotics/int-ball2_isaac_sim)
![GitHub issues](https://img.shields.io/github/issues/sd-robotics/int-ball2_isaac_sim)
![GitHub fork](https://img.shields.io/github/forks/sd-robotics/int-ball2_isaac_sim)
![GitHub stars](https://img.shields.io/github/stars/sd-robotics/int-ball2_isaac_sim)

<!--  [![Ubuntu22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview) -->
<!-- [![IsaacSim](https://img.shields.io/badge/IsaacSim-4.2.0-green.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html) -->
<!-- [![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html) -->
<!-- [![ros2-humble installation](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) -->

<p style="display: inline">
  <img src="https://img.shields.io/badge/-Ubuntu_22.04_LTS-555555.svg?style=flat&logo=ubuntu">  
  <img src="https://img.shields.io/badge/-Isaac_Sim 4.2.0-76B900.svg?style=flat&logo=nvidia&logoColor=white">
  <img src="https://img.shields.io/badge/-ROS2 Humble-%2322314E?style=flat&logo=ROS&logoColor=white">
  <img src="https://img.shields.io/badge/-Python 3.10-3776AB.svg?logo=python&style=flat&logoColor=white">
  <img src="https://img.shields.io/badge/License-Apache--2.0-60C060.svg?style=flat">
</p>

![Int-Ball2 Isaac Sim 01](img/int-ball2_isaac_sim_01.png)

## 目次
1. [**Int-Ball2シミュレータ (Isaac Sim) とは？**](#int-ball2シミュレータ-isaac-sim-とは)

2. [**前提条件**](#前提条件)

3. [**インストール**](#インストール)
    1. [リポジトリのクローン](#リポジトリのクローン)
    2. [依存関係のインストール](#依存関係のインストール)
    3. [アセットのダウンロード](#アセットのダウンロード)

4. [**使い方**](#使い方)
    1. [ビルド & ソース](#ビルド--ソース)
    2. [シミュレータの起動方法](#シミュレータの起動方法)
    3. [ROS Bridgeによるデータ取得](#ros-bridgeによるデータ取得)
    4. [遠隔操作 (ジョイコントローラ)](#遠隔操作-ジョイコントローラ)
    5. [ISS上の実機データの再生](#ISS上の実機データの再生)
    6. [追従制御の実行方法](#追従制御の実行方法)

5. [**データの可視化**](#データの可視化)

6. [**謝辞**](#謝辞)

---

## Int-Ball2シミュレータ (Isaac Sim) とは？
Int-Ball2は、ISSの日本実験棟 (JEM) に配備されたフリーフライング型のカメラロボットです。
地上からの遠隔操作で動画像の撮影を行い、宇宙飛行士をサポートします。
さらに、Int-Ball2は、拡張機能としてユーザーが開発したソフトウェアを実行することができ、
宇宙でのロボット技術を実証するためのプラットフォームとして使用することができます。

このリポジトリでは、Int-Ball2用のROS + Isaac Sim (NVIDIA) ベースのシミュレータを提供します。
ISS/JEM環境におけるInt-Ball2の挙動を、ユーザー開発プログラムでシミュレーションすることができます。

![Int-Ball2 Hardware](img/int-ball2_hardware.png)

## 前提条件
このリポジトリを使用するには、以下の環境を用意する必要があります。

|  Package  |         Version         |
| --------- | ----------------------- |
|   Ubuntu  | 22.04 (Jammy Jellyfish) |
| Isaac Sim | 4.2.0 (September 2024)  |
|    ROS    |     Humble Hawksbill    |
|   Python  |          3.10 <=        |

## インストール
### リポジトリのクローン
まだ、ROS 2のワークスペースがない場合は作成します。
```bash
mkdir -p ~/int-ball2_ws/src
cd ~/int-ball2_ws/src
```

本パッケージをワークスペースにクローンします。
```bash
git clone https://github.com/sd-robotics/int-ball2_isaac_sim.git
```

### 依存関係のインストール
ワークスペースまで移動します。
```bash
cd ~/int-ball2_ws/
```

依存関係をインストールします。
``` bash
rosdep install --from-paths src --ignore-src -r -y
```

### アセットのダウンロード
プロジェクトまで移動します。
```bash
cd ~/int-ball2_ws/src/int-ball2_isaac_sim
```

アセットをダウンロードします（Int-Ball2、JEM等）。
```bash
bash install_local.sh
```

## 使い方
### ビルド & ソース
パッケージのビルドとワークスペースのソースを行います。
```bash
cd ~/int-ball2_ws
colcon build --symlink-install
source install/setup.bash
```

### シミュレータの起動方法
ros2 launchでシミュレータを起動します。
```bash
ros2 launch int-ball2_isaac_sim int-ball2_isaac_sim.launch.py gui:="~/int-ball2_ws/src/int-ball2_isaac_sim/assets/KIBOU.usd"
```

> [!NOTE]
> `ROS_DOMAIN_ID`が事前にセットアップされていない場合、デフォルトIDが`0`となります。

画面左側にある「▶」ボタンを押すことでIsaac Simのシミュレーションが実行されます。
その後、Isaac Sim内のセンサや推進装置といったROSプログラムが動きます。

![Int-Ball2 Isaac Sim 02](img/int-ball2_isaac_sim_02.png)

視点を変えることで、ISS「きぼう」日本実験棟を見渡すことができます。

![Int-Ball2 Isaac Sim 02](img/int-ball2_isaac_sim_03.png)

> [!TIP]
> Isaac Simを実行するためにノートPCを使用しており、Isaac Simが起動したときにモニターがフリーズする問題に悩まされている場合は、次のコマンドでNVIDIA GPUを使用するようにシステムを切り替えてみてください。
> ```bash
> sudo prime-select nvidia
> ```
>
> これにより、グラフィックを多用するタスクのパフォーマンスが向上します。ラップトップが正常にNVIDIA GPUに切り替わったかどうかを確認するには、次のコマンドを使用できます。
> ```bash
> prime-select query
> ```

### ROS Bridgeによるデータ取得
ユーザープログラムによって取得可能なデータは下記の通りです。

| 種類  |         ROS定義名          |                                 概要                               |
| ----- | ------------------------- | ------------------------------------------------------------------ | 
| Topic | /camera_main/image_raw    | Int-Ball2正面のメインカメラの画像。                                  |
| Topic | /camera_main/camera_info  | Int-Ball2正面のメインカメラに関する情報。                             |
| Topic | /camera_left/image_raw    | Int-Ball2の左側にあるサブカメラ (左) の画像。                         |
| Topic | /camera_left/camera_info  | Int-Ball2の左側にあるサブカメラ (左) に関する情報。                   |
| Topic | /camera_right/image_raw   | Int-Ball2の左側にあるサブカメラ (右) の画像。                         |
| Topic | /camera_right/camera_info | Int-Ball2の左側にあるサブカメラ (右) に関する情報。                   |
| Topic | /imu/imu                  | IMU (慣性計測ユニット) のセンサ値。                                  |
| Topic | /ground_truth             | ロボットの位置と姿勢 (ドッキングステーションからInt-Ball2本体) の真値。 |

ユーザープログラムによって制御可能なデータは下記の通りです。

| 種類  |         ROS定義名          |                                    概要                                    |
| ----- | ------------------------- | ------------------------------------------------------------------------- | 
| Topic | /ctl/wrench               | Int-Ball2 に力とトルクを入力する。Int-Ball2はForceとTorqueによって制御される。 |

### 遠隔操作 (ジョイコントローラ)
ワークスペースをソースします。
```bash
cd ~/int-ball2_ws
source install/setup.bash
```

コマンドを実行する前に、ジョイスティック・コントローラがPCに接続されていることを確認します。
次にテレオペ (ジョイスティック・コントローラでの操作) のプログラムを起動します。
```bash
ros2 launch int-ball2_control int-ball2_teleop.launch.py
```

ジョイスティック・コントローラでの操作は下記のとおりです。

並進移動の場合：
- X軸とY軸：左スティックで、
- Z軸：Bボタン＋RTまたはLT。

回転移動：
- X軸とY軸：右スティックで、
- Z軸：Aボタン＋RTまたはLT。

![Int-Ball2 Teleop](img/int-ball2_teleop.png)


### ISS上の実機データの再生
SpaceDataは、ISS上の実機Int-Ball2のデータを収集する機会を得ました。
これらのROSBagsは[HuggingFace](https://huggingface.co/datasets/SpaceData/int-ball2_data_on_iss)で公開されており、本シミュレータ上でも再生することができます。

1. まず、[HuggingFace CLI](https://huggingface.co/docs/huggingface_hub/main/ja/guides/cli)を使ってROSBagsデータをダウンロードします。
```bash
cd ~/int-ball2_ws/src/int-ball2_isaac_sim/int-ball2_data_replay/
# ダウンロードしたいROSBagsを選択してください
huggingface-cli download --repo-type dataset --local-dir ./ SpaceData/int-ball2_data_on_iss bags/rosbag_20250421111514.bag
```
2. 実機Int-Ball2はROS Melodicで動作しているため、[Rosbags](https://gitlab.com/ternaris/rosbags)を使ってROS2で再生できる形式に変換します。
```bash
cd bags/
rosbags-convert --src rosbag_20250421111514.bag --dst-storage sqlite3 --dst ./rosbag2_20250421111514
```
3. 変換したROSBagsを使ってIsaac Simを起動します。
```bash
ros2 launch ib2_isaac_sim int-ball2_issbag_demo.launch.py bag_file:="<ABSOLUTE/PATH/TO/ROSBAG>"
```

### 追従制御の実行方法

ISS上で収集したデータを再生するだけでなく、Int-Ball2が実際のISS内でどのように移動するかをシミュレーションすることもできます。
[Int-Ball2 Simulator](https://github.com/jaxa/int-ball2_simulator) のオリジナルコードをROS 2およびIsaac Simシミュレータ内で動作するように更新しました。

> [!NOTE]
> 現在、制御パッケージでは、元のリポジトリのようにロボットの位置や姿勢に誤差（V-SLAMの結果が実環境の位置誤差を模擬して不安定になる挙動）は適用されていません。

それでは、このシステムを試して、ロボットを自律移動させてみましょう！

1. 提供されているlaunchファイルでシミュレーションを起動します。
```bash
ros2 launch ib2_isaac_sim int-ball2_isaac_sim.launch.py usd_file:="KIBOU.usd"
```
2. 画面左側の「▶」ボタンを押してシミュレータを開始します。
3. ロボットの位置推定システムを起動します。
```bash
ros2 launch ib2_nav nav.launch.py
```
4. ロボットの誘導制御システムを起動します。
```bash
ros2 launch ib2_ctl bringup.launch.py
```
5. これで、ROS 2 Actionを使ってロボットを制御する準備ができました。独自のアクションクライアントを開発する前に、以下の例を使ってロボット誘導システムを試すことができます。
```bash
ros2 action send_goal /ctl/command ib2_msgs/action/CtlCommand "target:
  header:
    stamp:
     sec: 0
     nanosec: 0
    frame_id: ''
  pose:
    position:
     x: 0.0
     y: 0.0
     z: 0.0
    orientation:
     x: 0.0
     y: 0.0
     z: 0.0
     w: 1.0
 type:
  type: 40"
```

> [!TIP]
> ゴール送信時に`type: 40`を指定するとISS原点（ドッキングステーション）を基準に、`type: 30`を指定するとロボット自身を基準に移動させることができます。

## データの可視化
ワークスペースをソースします。
```bash
cd ~/int-ball2_ws
source install/setup.bash
```

Rvizを起動します。
```bash
ros2 launch int-ball2_control rviz_visualize.launch.py 
```

![Int-Ball2 Rviz](img/int-ball2_rviz.png)

## 引用
本プラットフォームを研究に使用する場合は、以下のように引用してください。
```
[1] SpaceData Inc., 2025, int-ball2_isaac_sim (Version v1.0.0) [Source code], GitHub, Available at: https://github.com/sd-robotics/int-ball2_isaac_sim.
```

## 謝辞
このシミュレータは、宇宙イノベーションパートナーシップ（J-SPARC：JAXA Space Innovation through Partnership and Co-creation）の枠組みの中で、株式会社スペースデータがJAXAと協力して開発したものです。

> [!TIP]
> この文書は、Apache License 2.0の下でライセンスされているJAXAのInt-Ball2シミュレータの内容を含んでいます。
> - [Int-Ball2 Simulator (Gazebo)](https://github.com/jaxa/int-ball2_simulator)

---

[トップに戻る](#int-ball2シミュレータ-isaac-sim)
