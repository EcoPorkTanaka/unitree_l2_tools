# Unitree L2 Tools

Unitree L2 Bionic 4D LiDAR の設定・キャプチャツール集。

## 機能

- **対話式設定ツール** — LiDARの状態確認・動作モード変更・ネットワーク設定・IMU情報表示
- **コマンドツール** — LiDARの回転開始/停止・接続テスト
- **生データキャプチャ** — 点群データをLAZ形式で保存

## 必要環境

- Python 3.9以上
- Unitree L2 Bionic 4D LiDAR
- イーサネット接続（LiDARとPCを直接接続）

## インストール

```bash
pip install -r requirements.txt
```

## ネットワーク設定

LiDARとPCをイーサネットケーブルで直接接続し、PCのIPアドレスを設定します。

| 項目 | デフォルト値 |
|------|------------|
| PC（ホスト）IP | 192.168.1.2 |
| PC 受信ポート | 6201 |
| LiDAR IP | 192.168.1.62 |
| LiDAR 送信ポート | 6101 |

## 使い方

### 対話式設定ツール

LiDARの状態確認、動作モード変更、ネットワーク設定などを対話的に行えます。

```bash
python lidar_config.py
```

メニュー:

| 番号 | 機能 |
|------|------|
| 1 | 状態表示（接続・回転・IMU・パケットレート） |
| 2 | バージョン情報 |
| 3 | 回転開始 |
| 4 | 回転停止 |
| 5 | 再起動 |
| 6 | 設定同期（内部状態・温度・電圧等の詳細取得） |
| 7 | 起動モード設定（自動起動/コマンド起動） |
| 8 | 動作モード設定（3D/2D、NEGA/Normal、UART/ENET等） |
| 9 | ネットワーク設定 |
| 0 | 設定リセット（工場出荷時に戻す） |
| a | レイテンシ計測 |

### コマンドツール

```bash
python lidar_command.py start   # LiDAR回転開始
python lidar_command.py stop    # LiDAR回転停止
python lidar_command.py test    # 接続テスト（ping + UDP通信確認）
```

### 生データキャプチャ（LAZ保存）

```bash
python capture_raw.py 10              # 10秒間キャプチャ
python capture_raw.py 5 -o scan.laz   # ファイル名を指定
python capture_raw.py 3 --ip 192.168.1.100  # ホストIPを指定
```

## ファイル構成

| ファイル | 説明 |
|---------|------|
| `lidar_config.py` | 対話式設定ツール |
| `lidar_command.py` | LiDARプロトコルのコマンド構築・送信 |
| `capture_raw.py` | 生データキャプチャ・LAZ保存 |
| `config.py` | ネットワーク設定 |

## パケット構造

Unitree L2 のUDPパケット（タイプ102: 3D点群）:

```
FrameHeader (12B): magic(4) + type(4) + size(4)
DataInfo (16B): sequence(4) + payload_size(4) + ts_sec(4) + ts_nsec(4)
InsideState (20B): 内部状態
DeviceParams (16B): 温度・電圧等
CalibParam (12B): axis_dist(4) + bias1(4) + bias2(4)
AngleParams (20B): 角度関連パラメータ
ScanGeometry (32B): 角度開始・ステップ等
PointNum (4B): 点数 (通常300)
Ranges: uint16 × point_num (距離mm)
Intensities: uint8 × point_num (強度)
FrameTail (12B): CRC等
```

## ライセンス

MIT
