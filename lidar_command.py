#!/usr/bin/env python3
"""
Unitree L2 Bionic 4D LiDAR コマンド送信ツール

公式SDK (unilidar_sdk2) の unitree_lidar_protocol.h に基づくフレーム構築。

フレーム構造:
  FrameHeader (12B): magic[4] + packet_type(uint32) + packet_size(uint32)
  Payload (可変)
  FrameTail (12B): crc32(uint32) + msg_type_check(uint32) + reserve[2] + tail[2]

CRC32は Payload のみに対して計算する (FrameHeader, FrameTailは含まない)。
"""

import socket
import struct
import time
import sys
import zlib

from config import LIDAR_IP, LIDAR_PORT, LOCAL_IP, LOCAL_PORT

# マジックバイト
MAGIC = b'\x55\xAA\x05\x0A'

# テールマーカー
TAIL_MARKER = b'\x00\xFF'

# FrameHeader / FrameTail サイズ
HEADER_SIZE = 12
TAIL_SIZE = 12

# パケットタイプ (公式SDK: unitree_lidar_protocol.h)
PACKET_TYPE_USER_CMD = 100          # LIDAR_USER_CMD_PACKET_TYPE
PACKET_TYPE_ACK = 101               # LIDAR_ACK_DATA_PACKET_TYPE
PACKET_TYPE_POINT_DATA = 102        # LIDAR_POINT_DATA_PACKET_TYPE
PACKET_TYPE_2D_POINT_DATA = 103     # LIDAR_2D_POINT_DATA_PACKET_TYPE
PACKET_TYPE_IMU_DATA = 104          # LIDAR_IMU_DATA_PACKET_TYPE
PACKET_TYPE_VERSION = 105           # LIDAR_VERSION_PACKET_TYPE
PACKET_TYPE_WORK_MODE = 107         # LIDAR_WORK_MODE_CONFIG_PACKET_TYPE
PACKET_TYPE_IP_CONFIG = 108         # LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE
PACKET_TYPE_MAC_CONFIG = 109        # LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE

# ユーザーコマンドタイプ (LidarUserCtrlCmd.cmd_type)
USER_CMD_RESET = 1                  # USER_CMD_RESET_TYPE: リセット
USER_CMD_STANDBY = 2                # USER_CMD_STANDBY_TYPE: value 0=開始, 1=停止
USER_CMD_VERSION_GET = 3            # USER_CMD_VERSION_GET: バージョン取得
USER_CMD_LATENCY = 4                # USER_CMD_LATENCY_TYPE: レイテンシ計測
USER_CMD_CONFIG_RESET = 5           # USER_CMD_CONFIG_RESET: 設定リセット
USER_CMD_CONFIG_GET = 6             # USER_CMD_CONFIG_GET: 設定取得
USER_CMD_AUTO_STANDBY = 7           # USER_CMD_CONFIG_AUTO_STANDBY

# 後方互換用定数
CMD_GET_VERSION = USER_CMD_VERSION_GET
CMD_START_ROTATION = "start"        # センチネル: build_command内で処理
CMD_STOP_ROTATION = "stop"          # センチネル: build_command内で処理
CMD_REBOOT = USER_CMD_RESET
# WorkMode ビットフラグ定義（推定、要実機検証）
# マニュアル記載の全設定項目が SetMode → Restart で反映される
WORKMODE_BIT_NORMAL = 0             # bit 0: 0=NEGA Mode(default), 1=Normal Mode
WORKMODE_BIT_2D = 1                 # bit 1: 0=3D(default), 1=2D
WORKMODE_BIT_IMU_DISPLAY = 2        # bit 2: 0=Disable(default), 1=Enable
WORKMODE_BIT_UART = 3               # bit 3: 0=ENET/UDP(default), 1=UART [SDK確認済み]
WORKMODE_BIT_GRAY_OFF = 4           # bit 4: 0=Gray ON(default), 1=Gray OFF


def _crc32(data: bytes) -> int:
    """CRC-32 (IEEE 802.3) を計算する。SDKと同じアルゴリズム。"""
    return zlib.crc32(data) & 0xFFFFFFFF


def build_frame(packet_type: int, payload: bytes) -> bytes:
    """
    LiDARプロトコルフレームを構築する (公式SDK準拠)
    """
    total_size = HEADER_SIZE + len(payload) + TAIL_SIZE

    # FrameHeader
    header = MAGIC
    header += struct.pack('<I', packet_type)
    header += struct.pack('<I', total_size)

    # CRC32: ペイロードのみに対して計算
    crc = _crc32(payload)

    # FrameTail
    tail = struct.pack('<I', crc)           # crc32
    tail += struct.pack('<I', 0)            # msg_type_check (送信時は0)
    tail += b'\x00\x00'                     # reserve
    tail += TAIL_MARKER                     # tail: 0x00 0xFF

    return header + payload + tail


def build_user_cmd(cmd_type: int, cmd_value: int = 0) -> bytes:
    """ユーザーコマンドパケット (LidarUserCtrlCmdPacket, 32B) を構築する"""
    payload = struct.pack('<II', cmd_type, cmd_value)
    return build_frame(PACKET_TYPE_USER_CMD, payload)


def build_work_mode(mode: int) -> bytes:
    """動作モード設定パケット (LidarWorkModeConfigPacket, 28B) を構築する"""
    payload = struct.pack('<I', mode)
    return build_frame(PACKET_TYPE_WORK_MODE, payload)


def build_ip_config(lidar_ip: str, user_ip: str, gateway: str,
                    subnet_mask: str, lidar_port: int, user_port: int) -> bytes:
    """
    IP設定パケット (LidarIpAddressConfigPacket) を構築する

    ペイロード (LidarIpAddressConfig, 20B):
      - lidar_ip[4] + user_ip[4] + gateway[4] + subnet_mask[4]
      - lidar_port (uint16 LE) + user_port (uint16 LE)
    """
    def ip_to_bytes(ip_str):
        return bytes(int(x) for x in ip_str.split('.'))

    payload = ip_to_bytes(lidar_ip)
    payload += ip_to_bytes(user_ip)
    payload += ip_to_bytes(gateway)
    payload += ip_to_bytes(subnet_mask)
    payload += struct.pack('<HH', lidar_port, user_port)
    return build_frame(PACKET_TYPE_IP_CONFIG, payload)


def build_command(cmd_id, param: int = 0) -> bytes:
    """後方互換用ラッパー"""
    if cmd_id == CMD_START_ROTATION:
        return build_user_cmd(USER_CMD_STANDBY, 0)
    elif cmd_id == CMD_STOP_ROTATION:
        return build_user_cmd(USER_CMD_STANDBY, 1)
    elif cmd_id == CMD_REBOOT:
        return build_user_cmd(USER_CMD_RESET, 1)
    else:
        return build_user_cmd(cmd_id, param)


def send_command(sock: socket.socket, cmd_data: bytes,
                 lidar_ip: str, lidar_port: int) -> None:
    """コマンドをLiDARに送信する"""
    sock.sendto(cmd_data, (lidar_ip, lidar_port))


def wait_response(sock: socket.socket, timeout: float = 2.0) -> tuple:
    """LiDARからの応答を待つ"""
    sock.settimeout(timeout)
    try:
        data, addr = sock.recvfrom(65535)
        return data, addr
    except socket.timeout:
        return None, None


def start_lidar(lidar_ip: str = LIDAR_IP, lidar_port: int = LIDAR_PORT,
                local_ip: str = LOCAL_IP, local_port: int = LOCAL_PORT):
    """LiDARを初期化して回転を開始する"""

    print("=" * 50)
    print("  Unitree L2 Bionic 4D - LiDAR初期化")
    print("=" * 50)
    print()
    print(f"[設定] LiDAR   : {lidar_ip}:{lidar_port}")
    print(f"[設定] ローカル : {local_ip}:{local_port}")
    print()

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    cmd_sock.bind((local_ip, local_port))
    print(f"[情報] ソケット: {local_ip}:{local_port}")

    # 1. バージョン取得（通信テスト）
    print()
    print("[1/2] バージョン情報を取得中...")
    cmd = build_user_cmd(USER_CMD_VERSION_GET, 0)
    send_command(cmd_sock, cmd, lidar_ip, lidar_port)
    resp, addr = wait_response(cmd_sock, timeout=2.0)
    if resp:
        print(f"  応答あり: {addr} → {len(resp)}バイト")
    else:
        print("  応答なし（タイムアウト）")
        print("  → LiDARへの接続を確認してください")

    # 2. 回転開始 (STANDBY value=0)
    print()
    print("[2/2] 回転を開始中...")
    cmd = build_user_cmd(USER_CMD_STANDBY, 0)
    send_command(cmd_sock, cmd, lidar_ip, lidar_port)
    resp, addr = wait_response(cmd_sock, timeout=2.0)
    if resp:
        print(f"  応答あり: {len(resp)}バイト")
    else:
        print("  応答なし")

    # データ受信確認（コマンドソケットをそのまま使用）
    print()
    print("[確認] データ受信を待機中...")
    cmd_sock.settimeout(5.0)

    received = 0
    try:
        for _ in range(10):
            data, addr = cmd_sock.recvfrom(65535)
            received += 1
            if received == 1:
                print(f"  データ受信開始! 送信元: {addr}")
                print(f"  パケットサイズ: {len(data)}バイト")
                if data[:4] == MAGIC:
                    ptype = struct.unpack_from('<I', data, 4)[0]
                    print(f"  パケットタイプ: {ptype}")
            if received >= 5:
                break
    except socket.timeout:
        if received == 0:
            print("  タイムアウト - データ未受信")
            print()
            print("[ヒント] LiDARがデータを送信していません。")
            print("  以下を確認してください:")
            print("  1. LiDARの電源が入っているか")
            print("  2. イーサネットケーブルが接続されているか")
            print(f"  3. LiDARの送信先設定 ({local_ip}:{local_port})")

    if received > 0:
        print(f"  {received}パケット正常受信")
        print()
        print("[成功] LiDARが動作中です。")

    cmd_sock.close()
    print()
    return received > 0


def stop_lidar(lidar_ip: str = LIDAR_IP, lidar_port: int = LIDAR_PORT,
               local_ip: str = LOCAL_IP):
    """LiDARの回転を停止する (STANDBY value=1)"""
    print("[情報] LiDAR回転停止コマンド送信中...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((local_ip, 0))

    cmd = build_user_cmd(USER_CMD_STANDBY, 1)
    send_command(sock, cmd, lidar_ip, lidar_port)
    resp, _ = wait_response(sock, timeout=2.0)
    if resp:
        print("[情報] 停止応答受信")
    else:
        print("[情報] 応答なし")
    sock.close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Unitree L2 LiDARコマンドツール")
    parser.add_argument("action", choices=["start", "stop", "test"],
                        help="実行するアクション")
    parser.add_argument("--ip", default=LOCAL_IP,
                        help=f"ホストPCのIP (デフォルト: {LOCAL_IP})")
    parser.add_argument("--lidar-ip", default=LIDAR_IP,
                        help=f"LiDAR IP (デフォルト: {LIDAR_IP})")
    args = parser.parse_args()

    if args.action == "start":
        start_lidar(lidar_ip=args.lidar_ip, local_ip=args.ip)
    elif args.action == "stop":
        stop_lidar(lidar_ip=args.lidar_ip, local_ip=args.ip)
    elif args.action == "test":
        print("[テスト] LiDARへのpingとUDP通信テスト")
        import subprocess
        result = subprocess.run(['ping', '-c', '2', '-t', '2', args.lidar_ip],
                                capture_output=True, text=True)
        print(result.stdout)
        start_lidar(lidar_ip=args.lidar_ip, local_ip=args.ip)
