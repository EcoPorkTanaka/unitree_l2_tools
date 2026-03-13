# Unitree L2 Tools

Unitree L2 Bionic 4D LiDAR 用の Python ツール集です。公式 SDK (unilidar_sdk2) のプロトコル仕様に基づき、LiDAR の状態確認・設定変更・点群キャプチャをコマンドラインから行えます。

公式 SDK は C++ / ROS 向けに提供されていますが、本ツールは Python のみで動作し、SDK のインストールや ROS 環境を必要としません。

## 機能一覧

| ツール | 説明 |
|--------|------|
| `lidar_config.py` | 対話式設定ツール。状態確認・動作モード変更・ネットワーク設定・IMU 情報表示など、LiDAR の全設定項目を対話メニューから操作 |
| `lidar_command.py` | コマンドラインツール。回転開始/停止・接続テスト (ping + UDP 通信確認) をワンライナーで実行 |
| `capture_raw.py` | 点群キャプチャツール。指定秒数のスキャンデータを座標変換なしで LAZ 形式に保存 |

## 必要環境

- Python 3.9 以上
- Unitree L2 Bionic 4D LiDAR (FW 2.8 以上で動作確認済み)
- イーサネット接続 (LiDAR と PC を直接接続、またはスイッチ経由)

## インストール

```bash
git clone https://github.com/EcoPorkTanaka/unitree_l2_tools.git
cd unitree_l2_tools
pip install -r requirements.txt
```

依存パッケージ:

| パッケージ | 用途 |
|-----------|------|
| `numpy` | 点群の座標変換・配列処理 |
| `laspy` | LAS/LAZ ファイルの読み書き |
| `lazrs` | LAZ 圧縮のバックエンド (laspy が内部で使用) |

## ネットワーク設定

LiDAR と PC をイーサネットケーブルで直接接続し、PC 側のネットワークインターフェースに固定 IP を設定します。

| 項目 | デフォルト値 | 説明 |
|------|------------|------|
| PC (ホスト) IP | `192.168.1.2` | PC のイーサネットアダプタに設定する IP アドレス |
| PC 受信ポート | `6201` | LiDAR からの点群データを受信するポート |
| LiDAR IP | `192.168.1.62` | LiDAR 本体の IP アドレス (工場出荷時) |
| LiDAR 送信ポート | `6101` | LiDAR のコマンド受付ポート |

デフォルト値は `config.py` で定義されています。各ツールの `--ip`, `--lidar-ip` 等のオプションで実行時に上書きできます。

### macOS でのネットワーク設定例

```bash
# イーサネットアダプタに固定IPを設定
sudo ifconfig en0 192.168.1.2 netmask 255.255.255.0

# LiDARへの疎通確認
ping 192.168.1.62
```

### Linux でのネットワーク設定例

```bash
sudo ip addr add 192.168.1.2/24 dev eth0
sudo ip link set eth0 up
ping 192.168.1.62
```

## 使い方

### 対話式設定ツール (`lidar_config.py`)

LiDAR の全設定項目を対話メニューから操作できます。

```bash
python lidar_config.py
python lidar_config.py --ip 192.168.1.100          # ホストIPを指定
python lidar_config.py --lidar-ip 192.168.1.100     # LiDAR IPを指定
```

#### メニュー構成

**情報取得:**

| 番号 | 機能 | 説明 |
|------|------|------|
| 1 | 状態表示 | 接続状態、回転状態、点群パケットレート (pkt/s, pts/s)、IMU 姿勢 (RPY)、角速度、加速度、重力加速度を一覧表示 |
| 2 | バージョン | デバイス名 (YS-L2)、ハードウェアバージョン、ファームウェアバージョン、コンパイル日付 |
| 6 | 設定同期 | バージョン情報 + WorkMode 設定 + LiDAR 内部状態 (水平/垂直モーター速度、汚れ指数、パケットロス率、APD 温度/電圧、レーザー電圧、IMU 温度) を一括取得 |
| a | レイテンシ | LiDAR との UDP 往復時間 (RTT) を計測。片道推定値も表示 |

**制御:**

| 番号 | 機能 | 説明 |
|------|------|------|
| 3 | 回転開始 | モーターを起動し点群出力を開始。コマンドを複数回送信し、モーター起動完了 (点群パケット受信) まで最大 25 秒待機して確認 |
| 4 | 回転停止 | モーターを停止しスタンバイ状態に移行。停止前後のパケット数を比較して停止を確認 |
| 5 | 再起動 | LiDAR 本体をリブート (確認プロンプトあり)。再起動には約 10 秒かかる |

**設定変更 (再起動後に反映):**

| 番号 | 機能 | 説明 |
|------|------|------|
| 7 | 起動モード | 電源投入時の動作を設定。`SELF START` (電源 ON で自動起動、デフォルト) / `CMD START` (コマンド待ち) |
| 8 | 動作モード | WorkMode ビットフラグを個別トグルまたは直接値入力で変更。通信モード (ENET/UART)、ワークモード (NEGA/Normal)、3D/2D モード、IMU 表示、グレースケール出力 |
| 9 | ネットワーク | LiDAR IP、送信先 PC IP、ゲートウェイ、サブネットマスク、ポート番号を変更。IP アドレスのバリデーションと確認プロンプトあり |
| 0 | 設定リセット | 全設定を工場出荷時の状態に復元 (確認プロンプトあり) |

### コマンドツール (`lidar_command.py`)

スクリプトやパイプラインから使いやすいワンライナー形式のツールです。

```bash
python lidar_command.py start   # 回転開始 + データ受信確認
python lidar_command.py stop    # 回転停止
python lidar_command.py test    # ping + 回転開始 + データ受信確認
```

| アクション | 動作 |
|-----------|------|
| `start` | バージョン取得 (通信テスト) → 回転開始コマンド送信 → ポート 6201 で点群パケット受信を確認 |
| `stop` | 回転停止コマンドを送信 |
| `test` | ICMP ping で疎通確認した後、`start` と同じ処理を実行 |

オプション:

```
--ip IP          ホストPCのIPアドレス (デフォルト: 192.168.1.2)
--lidar-ip IP    LiDARのIPアドレス (デフォルト: 192.168.1.62)
```

### 点群キャプチャ (`capture_raw.py`)

LiDAR から受信した点群を座標変換なしの生データとして LAZ ファイルに保存します。キャプチャ中は 1.5 秒間隔でキープアライブコマンドを送信し、接続を維持します。

```bash
python capture_raw.py                         # 1秒間キャプチャ (デフォルト)
python capture_raw.py 10                      # 10秒間キャプチャ
python capture_raw.py 60 -o room_scan.laz     # 60秒間、ファイル名を指定
python capture_raw.py 5 --ip 192.168.1.100    # ホストIPを指定
```

オプション:

```
duration              キャプチャ秒数 (デフォルト: 1)
-o, --output FILE     出力ファイル名 (デフォルト: capture_{秒数}s_raw.laz)
--ip IP               ホストPCのIPアドレス (デフォルト: 192.168.1.2)
--port PORT           受信ポート (デフォルト: 6201)
--lidar-ip IP         LiDARのIPアドレス (デフォルト: 192.168.1.62)
--lidar-port PORT     LiDARの送信ポート (デフォルト: 6101)
```

出力ファイルの仕様:

- 形式: LAZ (LAS 1.2, Point Format 0, lazrs 圧縮)
- 座標: LiDAR 座標系の直交座標 (x, y, z) メートル単位、精度 0.001m
- 強度: `intensity` フィールドに uint8 の生値 (0-255) を格納
- 座標変換: なし (天井設置補正等は適用されない)

## ファイル構成

```
unitree_l2_tools/
├── lidar_config.py     対話式設定ツール (メインツール)
├── lidar_command.py    コマンドライン制御ツール + プロトコル実装
├── capture_raw.py      点群キャプチャ → LAZ 保存
├── config.py           ネットワーク設定 (デフォルト値)
├── requirements.txt    依存パッケージ
└── README.md
```

### モジュール間の依存関係

```
lidar_config.py ──→ lidar_command.py ──→ config.py
capture_raw.py  ──→ lidar_command.py ──→ config.py
                ──→ config.py
```

- `lidar_command.py`: プロトコルの中核。フレーム構築 (`build_frame`)、CRC32 計算、全パケットタイプのコマンド生成を担当
- `lidar_config.py`: `lidar_command.py` のコマンド生成関数を使って対話的に LiDAR を操作
- `capture_raw.py`: `lidar_command.py` の初期化コマンドを使って LiDAR を起動し、自前のデコーダで点群を取得

## プロトコル仕様

公式 SDK (`unilidar_sdk2`) の `unitree_lidar_protocol.h` に基づく実装です。

### フレーム構造

全てのパケットは以下の共通フレーム構造を持ちます:

```
+-------------------+------------------+-------------------+
| FrameHeader (12B) | Payload (可変)   | FrameTail (12B)   |
+-------------------+------------------+-------------------+
```

**FrameHeader (12 バイト):**

| オフセット | サイズ | 型 | 説明 |
|-----------|--------|-----|------|
| 0 | 4 | bytes | マジックバイト `0x55 0xAA 0x05 0x0A` |
| 4 | 4 | uint32 LE | パケットタイプ |
| 8 | 4 | uint32 LE | パケット全体サイズ (Header + Payload + Tail) |

**FrameTail (12 バイト):**

| オフセット | サイズ | 型 | 説明 |
|-----------|--------|-----|------|
| 0 | 4 | uint32 LE | CRC32 (Payload のみに対して計算、IEEE 802.3) |
| 4 | 4 | uint32 LE | msg_type_check (送信時は 0) |
| 8 | 2 | bytes | 予約 `0x00 0x00` |
| 10 | 2 | bytes | テールマーカー `0x00 0xFF` |

### パケットタイプ一覧

| タイプ | 定数名 | 方向 | 説明 |
|--------|--------|------|------|
| 100 | `PACKET_TYPE_USER_CMD` | PC → LiDAR | ユーザーコマンド (回転制御、バージョン取得等) |
| 101 | `PACKET_TYPE_ACK` | LiDAR → PC | コマンド応答 (ACK) |
| 102 | `PACKET_TYPE_POINT_DATA` | LiDAR → PC | 3D 点群データ (1044 バイト固定、300 点/パケット) |
| 103 | `PACKET_TYPE_2D_POINT_DATA` | LiDAR → PC | 2D 点群データ |
| 104 | `PACKET_TYPE_IMU_DATA` | LiDAR → PC | IMU データ (80 バイト固定、クォータニオン + 角速度 + 加速度) |
| 105 | `PACKET_TYPE_VERSION` | LiDAR → PC | バージョン情報 |
| 107 | `PACKET_TYPE_WORK_MODE` | 双方向 | 動作モード設定/応答 |
| 108 | `PACKET_TYPE_IP_CONFIG` | PC → LiDAR | IP アドレス設定 |
| 109 | `PACKET_TYPE_MAC_CONFIG` | PC → LiDAR | MAC アドレス設定 |

### ユーザーコマンド (タイプ 100)

Payload は `cmd_type` (uint32) + `cmd_value` (uint32) の 8 バイトです。

| cmd_type | 定数名 | cmd_value | 説明 |
|----------|--------|-----------|------|
| 1 | `USER_CMD_RESET` | 1 | リブート |
| 2 | `USER_CMD_STANDBY` | 0=開始, 1=停止 | モーター回転制御 |
| 3 | `USER_CMD_VERSION_GET` | 0 | バージョン情報取得 |
| 4 | `USER_CMD_LATENCY` | 0 | レイテンシ計測 |
| 5 | `USER_CMD_CONFIG_RESET` | 0 | 設定を工場出荷時に復元 |
| 6 | `USER_CMD_CONFIG_GET` | 0 | 現在の設定取得 |
| 7 | `USER_CMD_AUTO_STANDBY` | 0=自動起動, 1=コマンド起動 | 電源ON時の動作モード |

### 3D 点群パケット (タイプ 102, 1044 バイト)

1 パケットあたり 300 点を格納します。約 91 パケット/秒 (約 27,000 点/秒) で送信されます。

| オフセット | サイズ | 内容 |
|-----------|--------|------|
| 0-11 | 12B | FrameHeader: magic(4) + type(4) + size(4) |
| 12-15 | 4B | sequence: パケット連番 |
| 16-19 | 4B | payload_size |
| 20-23 | 4B | timestamp_sec: UNIX タイムスタンプ (秒) |
| 24-27 | 4B | timestamp_nsec: ナノ秒部分 |
| 28-47 | 20B | InsideState: 水平/垂直モーター速度、汚れ指数、パケットロス率 |
| 48-63 | 16B | DeviceParams: APD 温度/電圧、レーザー電圧、IMU 温度 |
| 64-75 | 12B | CalibParam: axis_dist(float), bias1(float), bias2(float) |
| 76-95 | 20B | AngleParams: 垂直角度関連パラメータ |
| 96-127 | 32B | ScanGeometry: h_angle_start(float), h_angle_step(float), ..., v_angle_start(float), v_angle_step(float) |
| 128-131 | 4B | point_num: 点数 (uint32, 通常 300) |
| 132-731 | 600B | Ranges: uint16 LE × 300 (距離、mm 単位) |
| 732-1031 | 300B | Intensities: uint8 × 300 (反射強度) |
| 1032-1043 | 12B | FrameTail: CRC32 等 |

**座標変換:**

各点の 3D 座標は以下の手順で計算します:

```
dist_m = range_mm / 1000.0 + axis_dist / 1000.0
h_angle = h_angle_start + index * h_angle_step
v_angle = v_angle_start + index * v_angle_step + v_bias

x = dist_m * cos(v_angle) * cos(h_angle)
y = dist_m * cos(v_angle) * sin(h_angle)
z = dist_m * sin(v_angle)
```

### IMU パケット (タイプ 104, 80 バイト)

| オフセット | サイズ | 型 | 内容 |
|-----------|--------|-----|------|
| 28-43 | 16B | float × 4 | クォータニオン (x, y, z, w) |
| 44-55 | 12B | float × 3 | 角速度 (rad/s) |
| 56-67 | 12B | float × 3 | 加速度 (m/s²) |

### WorkMode ビットフラグ (タイプ 107)

Payload は `mode` (uint32) の 4 バイトです。各ビットが設定項目に対応します。

| ビット | 0 (デフォルト) | 1 | 備考 |
|--------|---------------|---|------|
| bit 0 | NEGA Mode (360°×96° FOV) | Normal Mode | |
| bit 1 | 3D | 2D | |
| bit 2 | IMU 表示無効 | IMU 表示有効 | |
| bit 3 | ENET (UDP) | UART | SDK で確認済み |
| bit 4 | Gray ON | Gray OFF | |

設定変更は SetMode → Restart の手順で反映されます。bit 3 以外のビット位置は推定値です。

### ACK パケット (タイプ 101)

コマンド応答の Payload 構造:

| オフセット | サイズ | 型 | 内容 |
|-----------|--------|-----|------|
| 0 | 4 | uint32 LE | packet_type (元コマンドのタイプ) |
| 4 | 4 | uint32 LE | cmd_type |
| 8 | 4 | uint32 LE | cmd_value |
| 12 | 4 | uint32 LE | status: 1=成功, 2=CRC エラー, 3=ヘッダーエラー, 4=ブロックエラー, 5=データ未準備 |

## トラブルシューティング

### ソケットバインド失敗

```
[エラー] ソケットバインド失敗: [Errno 49] Can't assign requested address
```

PC のイーサネットアダプタに `192.168.1.2` が設定されていません。ネットワーク設定を確認してください。

### データ未受信 (タイムアウト)

```
タイムアウト - データ未受信
```

- LiDAR の電源が入っているか確認
- イーサネットケーブルが正しく接続されているか確認
- `ping 192.168.1.62` で疎通確認
- LiDAR のネットワーク設定が変更されている場合、`lidar_config.py` のネットワーク設定メニューで確認・修正

### キャプチャで点群が少ない

LiDAR は電源投入後、モーターが安定するまで数秒かかります。`capture_raw.py` は実行時に自動で回転開始コマンドを送信しますが、モーター安定前のデータはデコードに失敗する場合があります。事前に `python lidar_command.py start` で回転を開始し、数秒待ってからキャプチャすると安定したデータが得られます。
