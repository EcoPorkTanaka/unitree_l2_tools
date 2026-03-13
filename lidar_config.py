#!/usr/bin/env python3
"""
Unitree L2 Bionic 4D LiDAR 対話式設定ツール

LiDARの状態確認・設定変更を対話的に行う。

使用方法:
    python lidar_config.py [オプション]

オプション:
    --ip IP        ホストPCのIPアドレス (デフォルト: 192.168.1.2)
    --lidar-ip IP  LiDARのIPアドレス (デフォルト: 192.168.1.62)
"""

import argparse
import socket
import struct
import time
import sys
import math

from config import LIDAR_IP, LIDAR_PORT, LOCAL_IP, LOCAL_PORT
from lidar_command import (
    build_command, build_user_cmd, build_work_mode, build_ip_config,
    CMD_START_ROTATION, CMD_STOP_ROTATION,
    CMD_GET_VERSION, CMD_REBOOT,
    USER_CMD_STANDBY, USER_CMD_RESET, USER_CMD_VERSION_GET,
    USER_CMD_CONFIG_GET, USER_CMD_CONFIG_RESET, USER_CMD_AUTO_STANDBY,
    USER_CMD_LATENCY,
    PACKET_TYPE_IP_CONFIG,
    WORKMODE_BIT_NORMAL, WORKMODE_BIT_2D, WORKMODE_BIT_IMU_DISPLAY,
    WORKMODE_BIT_UART, WORKMODE_BIT_GRAY_OFF,
)


class LidarConnection:
    """LiDARとのUDP通信を管理する"""

    def __init__(self, local_ip, lidar_ip, lidar_port):
        self.local_ip = local_ip
        self.lidar_ip = lidar_ip
        self.lidar_port = lidar_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
        self.sock.settimeout(0.3)
        self.sock.bind((local_ip, 0))
        self.bound_port = self.sock.getsockname()[1]

    def send_cmd(self, cmd_id, param=0):
        """コマンドを送信する"""
        cmd = build_command(cmd_id, param)
        self.sock.sendto(cmd, (self.lidar_ip, self.lidar_port))

    def send_raw(self, packet: bytes):
        """生パケットを送信する"""
        self.sock.sendto(packet, (self.lidar_ip, self.lidar_port))

    def flush(self, max_duration=0.3):
        """受信バッファをクリアする"""
        self.sock.settimeout(0.005)
        t_end = time.time() + max_duration
        try:
            while time.time() < t_end:
                self.sock.recvfrom(65535)
        except socket.timeout:
            pass
        self.sock.settimeout(0.3)

    def recv_packets(self, duration=1.0, max_packets=500):
        """指定時間パケットを受信して種類別に分類する"""
        packets = {101: [], 102: [], 104: [], 105: [], 107: []}
        t_start = time.time()
        count = 0
        while time.time() - t_start < duration and count < max_packets:
            try:
                data, _ = self.sock.recvfrom(65535)
                if len(data) >= 8:
                    ptype = struct.unpack_from('<I', data, 4)[0]
                    if ptype in packets:
                        packets[ptype].append(data)
                count += 1
            except socket.timeout:
                continue
        return packets

    def close(self):
        self.sock.close()


def parse_imu_packet(data):
    """type 104 (80B) IMU/状態パケットを解析する"""
    if len(data) != 80:
        return None

    seq = struct.unpack_from('<I', data, 12)[0]
    ts_sec = struct.unpack_from('<I', data, 20)[0]
    ts_nsec = struct.unpack_from('<I', data, 24)[0]

    # クォータニオン (w, x, y, z)
    qx = struct.unpack_from('<f', data, 28)[0]
    qy = struct.unpack_from('<f', data, 32)[0]
    qz = struct.unpack_from('<f', data, 36)[0]
    qw = struct.unpack_from('<f', data, 40)[0]

    # 角速度 (rad/s)
    wx = struct.unpack_from('<f', data, 44)[0]
    wy = struct.unpack_from('<f', data, 48)[0]
    wz = struct.unpack_from('<f', data, 52)[0]

    # 加速度 (m/s^2)
    ax = struct.unpack_from('<f', data, 56)[0]
    ay = struct.unpack_from('<f', data, 60)[0]
    az = struct.unpack_from('<f', data, 64)[0]

    # クォータニオン→オイラー角
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return {
        "seq": seq,
        "timestamp": ts_sec + ts_nsec * 1e-9,
        "quaternion": (qw, qx, qy, qz),
        "euler_deg": (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)),
        "angular_vel": (wx, wy, wz),
        "accel": (ax, ay, az),
    }


def parse_pointcloud_header(data):
    """type 102 (1044B) 点群パケットのヘッダー情報を解析する"""
    if len(data) < 132:
        return None

    seq = struct.unpack_from('<I', data, 12)[0]
    ts_sec = struct.unpack_from('<I', data, 20)[0]
    ts_nsec = struct.unpack_from('<I', data, 24)[0]

    axis_dist = struct.unpack_from('<f', data, 64)[0]
    v_bias = struct.unpack_from('<f', data, 80)[0]
    h_start = struct.unpack_from('<f', data, 96)[0]
    h_step = struct.unpack_from('<f', data, 100)[0]
    v_start = struct.unpack_from('<f', data, 116)[0]
    v_step = struct.unpack_from('<f', data, 120)[0]
    point_num = struct.unpack_from('<I', data, 128)[0]

    return {
        "seq": seq,
        "timestamp": ts_sec + ts_nsec * 1e-9,
        "axis_dist_mm": axis_dist,
        "v_bias_rad": v_bias,
        "h_start_deg": math.degrees(h_start),
        "h_step_deg": math.degrees(h_step),
        "v_start_deg": math.degrees(v_start),
        "v_step_deg": math.degrees(v_step),
        "point_num": point_num,
    }


def show_status(conn):
    """LiDARの現在の状態を全て表示する"""
    print()
    print("  状態を取得中...")
    conn.flush()

    conn.send_cmd(CMD_GET_VERSION, 0)

    packets = conn.recv_packets(duration=1.5, max_packets=300)

    imu_pkts = [p for p in packets[104] if len(p) == 80]
    pc_pkts = packets[102]

    print()
    print("=" * 56)
    print("  LiDAR状態情報")
    print("=" * 56)

    total = len(imu_pkts) + len(pc_pkts)
    print()
    print("  ■ 接続")
    print(f"    LiDARアドレス     : {conn.lidar_ip}:{conn.lidar_port}")
    print(f"    ローカルアドレス  : {conn.local_ip}:{conn.bound_port}")
    print(f"    受信パケット数    : {total} (1.5秒間)")

    if total == 0:
        print()
        print("    ⚠ パケット未受信 - LiDARが接続されていない可能性")
        return

    print()
    print("  ■ 動作状態")
    if pc_pkts:
        print(f"    回転状態          : 動作中")
        print(f"    点群パケット      : {len(pc_pkts)}パケット/1.5秒")
        print(f"    IMUパケット       : {len(imu_pkts)}パケット/1.5秒")

        info = parse_pointcloud_header(pc_pkts[-1])
        if info:
            print(f"    点数/パケット     : {info['point_num']}")
            print(f"    軸間距離          : {info['axis_dist_mm']:.2f} mm")
            print(f"    水平角度開始      : {info['h_start_deg']:.2f}°")
            print(f"    水平角度ステップ  : {info['h_step_deg']:.4f}°")
            print(f"    垂直角度開始      : {info['v_start_deg']:.2f}°")
            print(f"    垂直角度ステップ  : {info['v_step_deg']:.4f}°")
            print(f"    垂直バイアス      : {math.degrees(info['v_bias_rad']):.4f}°")
            pps = len(pc_pkts) / 1.5
            pts_per_sec = pps * info['point_num']
            print(f"    推定レート        : {pps:.0f} pkt/s ({pts_per_sec:,.0f} pts/s)")
    else:
        print(f"    回転状態          : 停止中")
        print(f"    IMUパケット       : {len(imu_pkts)}パケット/1.5秒")

    if imu_pkts:
        imu = parse_imu_packet(imu_pkts[-1])
        if imu:
            r, p, y = imu["euler_deg"]
            wx, wy, wz = imu["angular_vel"]
            ax, ay, az = imu["accel"]
            qw, qx, qy, qz = imu["quaternion"]
            print()
            print("  ■ IMU")
            print(f"    姿勢 (RPY)        : Roll {r:+.2f}°  Pitch {p:+.2f}°  Yaw {y:+.2f}°")
            print(f"    クォータニオン    : w={qw:.4f} x={qx:.4f} y={qy:.4f} z={qz:.4f}")
            print(f"    角速度 (rad/s)    : x={wx:+.4f}  y={wy:+.4f}  z={wz:+.4f}")
            print(f"    加速度 (m/s²)     : x={ax:+.4f}  y={ay:+.4f}  z={az:+.4f}")
            g = math.sqrt(ax*ax + ay*ay + az*az)
            print(f"    重力加速度        : {g:.3f} m/s² (理論値: 9.807)")

    print()
    print("=" * 56)


def get_version(conn):
    """バージョン情報を取得する"""
    print()
    print("  バージョン情報取得中...")
    conn.flush()
    conn.send_cmd(CMD_GET_VERSION, 0)
    time.sleep(0.3)

    packets = conn.recv_packets(duration=1.0, max_packets=100)
    ver_pkts = packets[105]

    if ver_pkts:
        data = ver_pkts[-1]
        if len(data) >= 12 + 80:
            hw = data[12:16]
            sw = data[16:20]
            name_bytes = data[20:44]
            date_bytes = data[44:52]

            hw_str = f"{hw[0]}.{hw[1]}.{hw[2]}.{hw[3]}"
            sw_str = f"{sw[0]}.{sw[1]}.{sw[2]}.{sw[3]}"
            name_str = name_bytes.rstrip(b'\x00').decode('ascii', errors='replace')
            date_str = date_bytes.rstrip(b'\x00').decode('ascii', errors='replace')

            print()
            print(f"  デバイス名          : {name_str}")
            print(f"  ハードウェアバージョン: {hw_str}")
            print(f"  ファームウェアバージョン: {sw_str}")
            print(f"  コンパイル日付      : {date_str}")
    else:
        print("  バージョンパケット未受信")


def start_rotation(conn):
    """回転を開始する (USER_CMD_STANDBY, value=0)"""
    print("  → 回転開始コマンド送信中...")
    conn.flush()
    # 複数回送信（確実に受理させる）
    for _ in range(3):
        conn.send_cmd(CMD_START_ROTATION, 0)
        time.sleep(0.3)

    # モーター起動を段階的に確認（最大25秒）
    print("  モーター起動待機中...", end="", flush=True)
    for wait in [3, 5, 5, 5, 5]:
        time.sleep(wait)
        # 開始コマンドを再送（確実に受理させる）
        conn.send_cmd(CMD_START_ROTATION, 0)
        time.sleep(0.2)
        # バッファに溜まった古いパケットを捨ててから確認
        conn.flush()
        conn.send_cmd(CMD_GET_VERSION, 0)
        packets = conn.recv_packets(duration=2.0, max_packets=2000)
        pc_count = len(packets[102])
        if pc_count > 0:
            info = parse_pointcloud_header(packets[102][-1])
            pts = info['point_num'] if info else '?'
            print()
            print(f"  [OK] 回転開始確認 ({pc_count}パケット受信, {pts}点/パケット)")
            return
        print(".", end="", flush=True)

    print()
    print(f"  [確認] コマンド送信済み (点群パケット未検出 - 起動に時間がかかる場合があります)")


def stop_rotation(conn):
    """回転を停止する (USER_CMD_STANDBY, value=1)"""
    print("  → 回転停止コマンド送信中...")

    conn.flush()
    pre_packets = conn.recv_packets(duration=0.5, max_packets=100)
    pre_pc = len(pre_packets[102])

    conn.send_cmd(CMD_STOP_ROTATION, 0)
    time.sleep(2.0)

    conn.flush()
    post_packets = conn.recv_packets(duration=1.5, max_packets=200)
    post_pc = len(post_packets[102])

    print()
    if pre_pc > 0 and post_pc == 0:
        print(f"  [OK] 回転停止確認 (点群: {pre_pc}→{post_pc} パケット)")
    elif post_pc == 0:
        print(f"  [OK] 回転停止状態 (点群パケットなし)")
    else:
        print(f"  [確認] コマンド送信済み (点群: {pre_pc}→{post_pc} パケット)")
        print(f"         停止までに時間がかかることがあります")


def parse_ack(packets):
    """ACKパケットを解析して結果を返す"""
    ack_pkts = packets[101]
    if not ack_pkts:
        return None
    ack = ack_pkts[-1]
    if len(ack) < 12 + 16:
        return None
    return {
        "packet_type": struct.unpack_from('<I', ack, 12)[0],
        "cmd_type": struct.unpack_from('<I', ack, 16)[0],
        "cmd_value": struct.unpack_from('<I', ack, 20)[0],
        "status": struct.unpack_from('<I', ack, 24)[0],
    }


def print_ack_result(ack):
    """ACK結果を表示する"""
    if ack is None:
        print("  ACK未受信")
        return False
    status = ack["status"]
    status_names = {1: "成功", 2: "CRCエラー", 3: "ヘッダーエラー",
                    4: "ブロックエラー", 5: "データ未準備"}
    name = status_names.get(status, f"不明({status})")
    print(f"  ACKステータス: {name}")
    return status == 1


def config_get(conn):
    """設定情報を取得する (USER_CMD_CONFIG_GET)"""
    print()
    print("  設定情報取得中...")
    conn.flush()
    conn.send_cmd(USER_CMD_CONFIG_GET, 0)
    time.sleep(0.5)

    packets = conn.recv_packets(duration=1.0, max_packets=100)
    ack = parse_ack(packets)
    if ack:
        print(f"  ACK: packet_type={ack['packet_type']}, cmd_type={ack['cmd_type']}, "
              f"cmd_value={ack['cmd_value']}, status={ack['status']}")
    else:
        print("  ACKパケット未受信")


def set_network(conn):
    """ネットワーク設定を変更する"""
    print()
    print("  ■ ネットワーク設定変更")
    print()
    print(f"    現在の接続先: {conn.lidar_ip}:{conn.lidar_port}")
    print()
    print("    変更する項目の値を入力してください。")
    print("    空欄のままEnterで現在値を維持します。")
    print()

    # 各項目の入力
    lidar_ip = input(f"    LiDAR IP [{conn.lidar_ip}]: ").strip()
    lidar_ip = lidar_ip or conn.lidar_ip

    user_ip = input(f"    送信先PC IP [{conn.local_ip}]: ").strip()
    user_ip = user_ip or conn.local_ip

    gateway = input("    ゲートウェイ [192.168.1.1]: ").strip()
    gateway = gateway or "192.168.1.1"

    subnet = input("    サブネットマスク [255.255.255.0]: ").strip()
    subnet = subnet or "255.255.255.0"

    lidar_port_str = input(f"    LiDARポート [{conn.lidar_port}]: ").strip()
    lidar_port = int(lidar_port_str) if lidar_port_str else conn.lidar_port

    user_port_str = input("    送信先ポート [6201]: ").strip()
    user_port = int(user_port_str) if user_port_str else 6201

    # IPアドレスの検証
    for name, ip in [("LiDAR IP", lidar_ip), ("送信先PC IP", user_ip),
                     ("ゲートウェイ", gateway), ("サブネット", subnet)]:
        parts = ip.split('.')
        if len(parts) != 4 or not all(p.isdigit() and 0 <= int(p) <= 255 for p in parts):
            print(f"  [エラー] 無効なIPアドレス: {name} = {ip}")
            return

    # 確認表示
    print()
    print("  送信する設定:")
    print(f"    LiDAR IP       : {lidar_ip}")
    print(f"    送信先PC IP    : {user_ip}")
    print(f"    ゲートウェイ   : {gateway}")
    print(f"    サブネットマスク: {subnet}")
    print(f"    LiDARポート    : {lidar_port}")
    print(f"    送信先ポート   : {user_port}")
    print()

    confirm = input("  この設定を適用しますか？ (yes/no): ").strip().lower()
    if confirm != "yes":
        print("  キャンセルしました")
        return

    # パケット送信
    print("  → ネットワーク設定送信中...")
    pkt = build_ip_config(lidar_ip, user_ip, gateway, subnet,
                          lidar_port, user_port)
    conn.flush()
    conn.send_raw(pkt)
    time.sleep(0.5)

    packets = conn.recv_packets(duration=1.0, max_packets=100)
    ack = parse_ack(packets)
    ok = print_ack_result(ack)

    if ok:
        print()
        print("  [OK] ネットワーク設定を送信しました")
        print("  ※ 設定を保存するには再起動してください")
        if lidar_ip != conn.lidar_ip or lidar_port != conn.lidar_port:
            print(f"  ※ LiDARアドレスが変更されます: "
                  f"{conn.lidar_ip}:{conn.lidar_port} → {lidar_ip}:{lidar_port}")
            print("  ※ 再起動後は新しいアドレスで接続してください")


def config_reset(conn):
    """設定を工場出荷時に戻す (USER_CMD_CONFIG_RESET)"""
    print()
    print("  ■ 設定リセット")
    print("    全設定を工場出荷時の状態に戻します。")
    print()

    confirm = input("  本当にリセットしますか？ (yes/no): ").strip().lower()
    if confirm != "yes":
        print("  キャンセルしました")
        return

    print("  → 設定リセットコマンド送信中...")
    conn.flush()
    conn.send_cmd(USER_CMD_CONFIG_RESET, 0)
    time.sleep(0.5)

    packets = conn.recv_packets(duration=1.0, max_packets=100)
    ack = parse_ack(packets)
    ok = print_ack_result(ack)

    if ok:
        print("  [OK] 設定リセット完了")
        print("  ※ 再起動後に反映されます")


def set_auto_standby(conn):
    """自動スタンバイ設定 (USER_CMD_AUTO_STANDBY)"""
    print()
    print("  ■ 電源ON時の起動モード (Power On mode)")
    print("    電源投入時にLiDARを自動起動するか設定します。")
    print("    ※ 設定後、再起動が必要です。")
    print()
    print("    0) 自動起動 (SELF START) — 電源ONで即座に動作開始 [デフォルト]")
    print("    1) コマンド起動 (CMD START) — 電源ONでスタンバイ(データ出力なし)")
    print("       → 回転開始(メニュー3)またはSDKで手動起動が必要")
    print()

    choice = input("  番号を入力 (0-1): ").strip()
    if choice not in ("0", "1"):
        print("  無効な選択です")
        return

    val = int(choice)
    label = "自動起動 (SELF START)" if val == 0 else "コマンド起動 (CMD START)"

    print(f"  → 設定中: {label}...")
    conn.flush()
    conn.send_cmd(USER_CMD_AUTO_STANDBY, val)
    time.sleep(0.3)
    # このコマンドはACKを返さない
    print(f"  [OK] {label} (コマンド送信済み)")
    print("  ※ 再起動後に反映されます")


def measure_latency(conn):
    """レイテンシ計測 (USER_CMD_LATENCY)"""
    print()
    print("  レイテンシ計測中...")
    conn.flush()

    t_send = time.time()
    conn.send_cmd(USER_CMD_LATENCY, 0)

    # ACK受信までの時間を計測
    conn.sock.settimeout(2.0)
    try:
        while True:
            data, _ = conn.sock.recvfrom(65535)
            if len(data) >= 8:
                ptype = struct.unpack_from('<I', data, 4)[0]
                if ptype == 101:  # ACK
                    t_recv = time.time()
                    rtt = (t_recv - t_send) * 1000
                    print(f"  往復時間 (RTT): {rtt:.1f} ms")
                    print(f"  片道推定:       {rtt/2:.1f} ms")
                    if len(data) >= 28:
                        status = struct.unpack_from('<I', data, 24)[0]
                        print(f"  ACKステータス:  {status} ({'成功' if status == 1 else 'エラー'})")
                    break
    except socket.timeout:
        print("  タイムアウト (ACK未受信)")
    finally:
        conn.sock.settimeout(0.3)


def get_current_mode(conn):
    """現在のWorkMode値を取得する"""
    conn.flush()
    conn.send_cmd(USER_CMD_CONFIG_GET, 0)
    time.sleep(0.3)
    packets = conn.recv_packets(duration=1.0, max_packets=200)
    wm_pkts = packets[107]
    if wm_pkts and len(wm_pkts[-1]) >= 16:
        return struct.unpack_from('<I', wm_pkts[-1], 12)[0]
    return None


def decode_work_mode(mode):
    """WorkModeビットフラグをデコードして設定辞書を返す"""
    return {
        "work_mode": "Normal" if (mode >> WORKMODE_BIT_NORMAL) & 1 else "NEGA",
        "dimension": "2D" if (mode >> WORKMODE_BIT_2D) & 1 else "3D",
        "imu_display": "有効" if (mode >> WORKMODE_BIT_IMU_DISPLAY) & 1 else "無効",
        "comm": "UART" if (mode >> WORKMODE_BIT_UART) & 1 else "ENET(UDP)",
        "gray": "OFF" if (mode >> WORKMODE_BIT_GRAY_OFF) & 1 else "ON",
    }


def print_work_mode(mode):
    """WorkMode設定を表示する"""
    d = decode_work_mode(mode)
    print(f"    モード値 (raw)    : {mode} (0x{mode:04X}, bin={mode:08b})")
    print(f"    a) 通信モード     : {d['comm']}"
          f"  {'[SDK確認済み]' if True else ''}")
    print(f"    b) ワークモード   : {d['work_mode']} Mode"
          f"  (NEGA=360°x96° FOV / Normal=標準)")
    print(f"    c) 3D/2Dモード    : {d['dimension']}")
    print(f"    d) IMU表示        : {d['imu_display']}")
    print(f"    e) グレースケール : Gray {d['gray']}")


def set_work_mode(conn):
    """動作モード設定 (WorkModeConfig, type 107)

    マニュアル記載の以下の設定を管理する:
    - ENET/UART Select: 通信モード選択
    - Work Mode: Normal Mode / NEGA Mode (ネガティブ角度モード)
    - 3D/2D Mode: 3Dモード / 2Dモード
    - IMU Display Enable: IMU表示の有効/無効
    - Gray Enable: グレースケールデータ出力の有効/無効

    ※ 全設定は再起動後に反映されます。
    ※ ビット位置は bit 3 (UART) 以外は推定値です。
       実機で動作しない場合は f) で直接モード値を指定してください。
    """
    print()
    print("  ■ 動作モード設定 (WorkMode)")
    print()

    # 現在のモードを取得
    mode = get_current_mode(conn)
    if mode is None:
        print("    現在のモード値を取得できません")
        print("    デフォルト値 0 を使用します")
        mode = 0

    while True:
        print()
        print("  現在の設定:")
        print_work_mode(mode)
        print()
        print("    f) 直接モード値入力")
        print("    s) 設定を送信 (SetMode)")
        print("    q) 戻る")
        print()

        choice = input("    変更する項目 (a-f/s/q): ").strip().lower()

        if choice == "a":
            bit = WORKMODE_BIT_UART
            mode ^= (1 << bit)
        elif choice == "b":
            bit = WORKMODE_BIT_NORMAL
            mode ^= (1 << bit)
        elif choice == "c":
            bit = WORKMODE_BIT_2D
            mode ^= (1 << bit)
        elif choice == "d":
            bit = WORKMODE_BIT_IMU_DISPLAY
            mode ^= (1 << bit)
        elif choice == "e":
            bit = WORKMODE_BIT_GRAY_OFF
            mode ^= (1 << bit)
        elif choice == "f":
            val_str = input("    モード値 (10進数): ").strip()
            if val_str.isdigit():
                mode = int(val_str)
            else:
                print("    無効な入力です")
        elif choice == "s":
            print(f"  → モード値 {mode} (0x{mode:04X}) を送信中...")
            conn.flush()
            conn.send_raw(build_work_mode(mode))
            time.sleep(0.5)
            packets = conn.recv_packets(duration=1.0, max_packets=200)
            ack = parse_ack(packets)
            if ack:
                print_ack_result(ack)
            # 再度取得して確認
            new_mode = get_current_mode(conn)
            if new_mode is not None:
                print(f"  設定後のモード値: {new_mode} (0x{new_mode:04X})")
            print("  ※ 再起動後に反映されます")
            return
        elif choice == "q":
            return
        else:
            print("    無効な入力です")


def sync_params(conn):
    """設定同期 — LiDARから現在のパラメータ情報を取得する (Synchronous)

    マニュアル記載: Obtains parameter information from L2
    """
    print()
    print("  ■ 設定同期 (Synchronous)")
    print("    LiDARから現在のパラメータ情報を取得します。")
    print()

    # バージョン情報
    conn.flush()
    conn.send_cmd(CMD_GET_VERSION, 0)
    time.sleep(0.3)
    packets = conn.recv_packets(duration=1.0, max_packets=200)

    ver_pkts = packets[105]
    if ver_pkts:
        data = ver_pkts[-1]
        if len(data) >= 12 + 80:
            hw = data[12:16]
            sw = data[16:20]
            name = data[20:44].rstrip(b'\x00').decode('ascii', errors='replace')
            print(f"  デバイス            : {name}")
            print(f"  HW                  : {hw[0]}.{hw[1]}.{hw[2]}.{hw[3]}")
            print(f"  FW                  : {sw[0]}.{sw[1]}.{sw[2]}.{sw[3]}")
            print()

    # WorkMode取得
    conn.flush()
    conn.send_cmd(USER_CMD_CONFIG_GET, 0)
    time.sleep(0.3)
    packets = conn.recv_packets(duration=1.0, max_packets=200)

    wm_pkts = packets[107]
    if wm_pkts and len(wm_pkts[-1]) >= 16:
        mode = struct.unpack_from('<I', wm_pkts[-1], 12)[0]
        print("  WorkMode設定:")
        print_work_mode(mode)
    else:
        print("  WorkMode: 取得失敗")

    # LiDAR内部状態（点群パケットから取得）
    print()
    conn.flush()
    conn.send_cmd(CMD_GET_VERSION, 0)
    packets = conn.recv_packets(duration=1.5, max_packets=500)
    pc_pkts = packets[102]
    imu_pkts = [p for p in packets[104] if len(p) == 80]

    if pc_pkts:
        p = pc_pkts[-1]
        if len(p) >= 128:
            # LidarInsideState (offset 28-64 in LidarPointData, after DataInfo(16B))
            # DataInfo starts at offset 12 (after FrameHeader)
            # LidarInsideState starts at 12+16=28
            sys_rot = struct.unpack_from('<I', p, 28)[0]
            com_rot = struct.unpack_from('<I', p, 32)[0]
            dirty = struct.unpack_from('<f', p, 36)[0]
            lost_up = struct.unpack_from('<f', p, 40)[0]
            lost_down = struct.unpack_from('<f', p, 44)[0]
            apd_temp = struct.unpack_from('<f', p, 48)[0]
            apd_volt = struct.unpack_from('<f', p, 52)[0]
            laser_volt = struct.unpack_from('<f', p, 56)[0]
            imu_temp = struct.unpack_from('<f', p, 60)[0]
            print("  LiDAR内部状態 (LidarInsideState):")
            print(f"    水平モーター速度  : {sys_rot} r/min")
            print(f"    垂直モーター速度  : {com_rot} r/min")
            print(f"    汚れ指数          : {dirty:.4f}")
            print(f"    パケットロス(上)  : {lost_up:.4f}")
            print(f"    パケットロス(下)  : {lost_down:.4f}")
            print(f"    APD温度           : {apd_temp:.1f} ℃")
            print(f"    APD電圧           : {apd_volt:.2f} V")
            print(f"    レーザー電圧      : {laser_volt:.2f} V")
            print(f"    IMU温度           : {imu_temp:.1f} ℃")
    else:
        print("  LiDAR内部状態: 点群パケットなし（回転停止中？）")

    print()
    print(f"  受信パケット: 点群={len(pc_pkts)} IMU={len(imu_pkts)}")


def reboot_lidar(conn):
    """LiDARを再起動する (USER_CMD_RESET)"""
    print()
    confirm = input("  本当に再起動しますか？ (yes/no): ").strip().lower()
    if confirm != "yes":
        print("  キャンセルしました")
        return

    print("  → 再起動コマンド送信中...")
    conn.send_cmd(CMD_REBOOT, 1)
    time.sleep(0.5)

    conn.flush()
    packets = conn.recv_packets(duration=2.0, max_packets=100)
    total = len(packets[102]) + len(packets[104])

    print()
    if total == 0:
        print("  [OK] 再起動中 (パケット停止を確認)")
        print("  ※ 再起動には約10秒かかります")
    else:
        print(f"  [確認] コマンド送信済み (まだ{total}パケット受信中)")


def main():
    parser = argparse.ArgumentParser(description="Unitree L2 LiDAR 対話式設定ツール")
    parser.add_argument("--ip", default=LOCAL_IP,
                        help=f"ホストPCのIP (デフォルト: {LOCAL_IP})")
    parser.add_argument("--lidar-ip", default=LIDAR_IP,
                        help=f"LiDAR IP (デフォルト: {LIDAR_IP})")
    parser.add_argument("--lidar-port", type=int, default=LIDAR_PORT,
                        help=f"LiDARポート (デフォルト: {LIDAR_PORT})")
    args = parser.parse_args()

    print("=" * 56)
    print("  Unitree L2 Bionic 4D - 対話式設定ツール")
    print("=" * 56)
    print(f"  LiDAR : {args.lidar_ip}:{args.lidar_port}")
    print(f"  ホスト: {args.ip}")
    print()

    try:
        conn = LidarConnection(args.ip, args.lidar_ip, args.lidar_port)
    except OSError as e:
        print(f"[エラー] ソケット作成失敗: {e}")
        print(f"  → ホストPCのIPが {args.ip} に設定されているか確認してください")
        sys.exit(1)

    print(f"  接続準備完了 (ローカルポート: {conn.bound_port})")

    try:
        while True:
            print()
            print("  ──────────────────────────────────────────")
            print("  1) 状態表示          7) 起動モード設定")
            print("  2) バージョン情報    8) 動作モード設定")
            print("  3) 回転開始          9) ネットワーク設定")
            print("  4) 回転停止          0) 設定リセット")
            print("  5) 再起動            a) レイテンシ計測")
            print("  6) 設定同期          q) 終了")
            print("  ──────────────────────────────────────────")
            print()

            choice = input("  番号を入力: ").strip().lower()

            if choice == "1":
                show_status(conn)
            elif choice == "2":
                get_version(conn)
            elif choice == "3":
                start_rotation(conn)
            elif choice == "4":
                stop_rotation(conn)
            elif choice == "5":
                reboot_lidar(conn)
            elif choice == "6":
                sync_params(conn)
            elif choice == "7":
                set_auto_standby(conn)
            elif choice == "8":
                set_work_mode(conn)
            elif choice == "9":
                set_network(conn)
            elif choice == "0":
                config_reset(conn)
            elif choice == "a":
                measure_latency(conn)
            elif choice in ("q", "quit", "exit"):
                break
            else:
                print("  無効な入力です")

    except (KeyboardInterrupt, EOFError):
        print()

    conn.close()
    print("  終了しました")


if __name__ == "__main__":
    main()
