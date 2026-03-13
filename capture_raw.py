#!/usr/bin/env python3
"""
Unitree L2 Bionic 4D LiDAR 生データキャプチャ

LiDARから受信した点群を座標変換なしでLAZファイルに保存する。

使用方法:
    python capture_raw.py [秒数] [オプション]

例:
    python capture_raw.py 5                    # 5秒間キャプチャ
    python capture_raw.py 10 -o scan.laz       # 10秒間、ファイル名指定
    python capture_raw.py 3 --ip 192.168.1.100 # ホストIPを指定
"""

import argparse
import os
import socket
import struct
import time
import sys
import numpy as np
import laspy

from config import LIDAR_IP, LIDAR_PORT, LOCAL_IP, LOCAL_PORT


def decode_raw(data: bytes):
    """
    パケットから生の球面座標→直交座標に変換（天井補正なし）。

    Returns:
        (points, intensities) or None
    """
    MAGIC = b'\x55\xAA\x05\x0A'
    HEADER_SIZE = 128
    TAIL_SIZE = 12

    if len(data) < HEADER_SIZE + 4 + TAIL_SIZE:
        return None
    if data[:4] != MAGIC:
        return None

    ptype = struct.unpack_from('<I', data, 4)[0]
    if ptype != 102:
        return None

    # キャリブレーション: 軸間距離
    axis_dist = struct.unpack_from('<f', data, 64)[0]

    # 角度パラメータ
    v_bias = struct.unpack_from('<f', data, 80)[0]
    h_angle_start = struct.unpack_from('<f', data, 96)[0]
    h_angle_step = struct.unpack_from('<f', data, 100)[0]
    v_angle_start = struct.unpack_from('<f', data, 116)[0]
    v_angle_step = struct.unpack_from('<f', data, 120)[0]

    # 点数
    point_num = struct.unpack_from('<I', data, HEADER_SIZE)[0]
    if point_num == 0 or point_num > 500:
        return None

    data_start = HEADER_SIZE + 4
    ranges_size = point_num * 2
    intens_size = point_num
    if len(data) < data_start + ranges_size + intens_size + TAIL_SIZE:
        return None

    # 距離 (uint16, mm)
    ranges = np.frombuffer(
        data[data_start:data_start + ranges_size], dtype=np.uint16
    ).astype(np.float64)

    # 強度 (uint8)
    intensities = np.frombuffer(
        data[data_start + ranges_size:data_start + ranges_size + intens_size],
        dtype=np.uint8
    ).astype(np.uint16)

    # 有効点フィルタ
    valid = (ranges > 10) & (ranges < 60000)
    if not np.any(valid):
        return None

    # 距離をメートルに変換
    dist_m = ranges / 1000.0 + axis_dist / 1000.0

    # 角度計算
    indices = np.arange(point_num, dtype=np.float64)
    h_angles = h_angle_start + indices * h_angle_step
    v_angles = v_angle_start + indices * v_angle_step + v_bias

    # 球面→直交座標（無変換）
    cos_v = np.cos(v_angles)
    x = dist_m * cos_v * np.cos(h_angles)
    y = dist_m * cos_v * np.sin(h_angles)
    z = dist_m * np.sin(v_angles)

    points = np.column_stack([x[valid], y[valid], z[valid]])
    return points, intensities[valid]


def send_init(sock, lidar_ip, lidar_port):
    """LiDARに初期化コマンドを送信"""
    from lidar_command import build_command, CMD_START_ROTATION, CMD_GET_VERSION

    for cmd_id, param, name in [
        (CMD_GET_VERSION, 0, "バージョン取得"),
        (CMD_START_ROTATION, 0, "回転開始"),
    ]:
        cmd = build_command(cmd_id, param)
        sock.sendto(cmd, (lidar_ip, lidar_port))
        print(f"  → {name}")
        time.sleep(0.3)


def save_laz(points: np.ndarray, intensities: np.ndarray, filepath: str):
    """点群をLAZファイルに保存する"""
    header = laspy.LasHeader(point_format=0, version="1.2")
    header.scales = [0.001, 0.001, 0.001]
    header.offsets = [points[:, 0].min(), points[:, 1].min(), points[:, 2].min()]

    las = laspy.LasData(header)
    las.x = points[:, 0]
    las.y = points[:, 1]
    las.z = points[:, 2]
    las.intensity = intensities

    las.write(filepath)


def main():
    parser = argparse.ArgumentParser(
        description="Unitree L2 LiDAR 生データキャプチャ → LAZ保存"
    )
    parser.add_argument("duration", type=float, nargs="?", default=1.0,
                        help="キャプチャ秒数 (デフォルト: 1)")
    parser.add_argument("-o", "--output", default=None,
                        help="出力ファイル名 (デフォルト: capture_{秒数}s_raw.laz)")
    parser.add_argument("--ip", default=LOCAL_IP,
                        help=f"ホストPCのIP (デフォルト: {LOCAL_IP})")
    parser.add_argument("--port", type=int, default=LOCAL_PORT,
                        help=f"受信ポート (デフォルト: {LOCAL_PORT})")
    parser.add_argument("--lidar-ip", default=LIDAR_IP,
                        help=f"LiDAR IP (デフォルト: {LIDAR_IP})")
    parser.add_argument("--lidar-port", type=int, default=LIDAR_PORT,
                        help=f"LiDARポート (デフォルト: {LIDAR_PORT})")
    args = parser.parse_args()

    output = args.output or f"capture_{args.duration:.0f}s_raw.laz"

    print("=" * 50)
    print("  Unitree L2 - 生データキャプチャ")
    print("=" * 50)
    print(f"  キャプチャ時間 : {args.duration}秒")
    print(f"  出力ファイル   : {output}")
    print(f"  ホストIP       : {args.ip}:{args.port}")
    print(f"  LiDAR          : {args.lidar_ip}:{args.lidar_port}")
    print(f"  座標変換       : なし（生データ）")
    print()

    # ソケット作成・バインド
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    sock.settimeout(1.0)

    try:
        sock.bind((args.ip, args.port))
    except OSError as e:
        print(f"[エラー] ソケットバインド失敗: {e}")
        print(f"  → ホストPCのIPが {args.ip} に設定されているか確認してください")
        sys.exit(1)

    # LiDAR初期化
    print("[情報] LiDAR初期化中...")
    try:
        send_init(sock, args.lidar_ip, args.lidar_port)
    except Exception as e:
        print(f"[警告] 初期化コマンド送信失敗: {e}")

    time.sleep(0.5)

    # キャプチャ開始
    from lidar_command import build_command, CMD_GET_VERSION
    keepalive = build_command(CMD_GET_VERSION, 0)
    all_points = []
    all_intensities = []
    pkt_recv = 0
    pkt_decoded = 0
    last_keepalive = time.time()

    print(f"[情報] キャプチャ開始 ({args.duration}秒間)...")
    t_start = time.time()

    while True:
        elapsed = time.time() - t_start
        if elapsed >= args.duration:
            break

        # キープアライブ送信
        now = time.time()
        if now - last_keepalive >= 1.5:
            try:
                sock.sendto(keepalive, (args.lidar_ip, args.lidar_port))
            except OSError:
                pass
            last_keepalive = now

        # パケット受信
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError:
            break

        pkt_recv += 1
        result = decode_raw(data)
        if result is not None:
            pts, ints = result
            all_points.append(pts)
            all_intensities.append(ints)
            pkt_decoded += 1

        # 進捗表示（1秒ごと）
        if pkt_recv % 500 == 0:
            total_pts = sum(len(p) for p in all_points)
            print(f"  {elapsed:.1f}s | {pkt_recv}パケット | "
                  f"{pkt_decoded}デコード | {total_pts:,}点", flush=True)

    sock.close()

    # 結果確認
    if not all_points:
        print("[エラー] 有効な点群データを受信できませんでした")
        sys.exit(1)

    points = np.vstack(all_points)
    intensities = np.concatenate(all_intensities)

    print()
    print(f"[結果] 受信パケット : {pkt_recv}")
    print(f"[結果] デコード成功 : {pkt_decoded}")
    print(f"[結果] 総点数       : {len(points):,}")
    print(f"[結果] X範囲        : {points[:,0].min():.3f} ~ {points[:,0].max():.3f} m")
    print(f"[結果] Y範囲        : {points[:,1].min():.3f} ~ {points[:,1].max():.3f} m")
    print(f"[結果] Z範囲        : {points[:,2].min():.3f} ~ {points[:,2].max():.3f} m")

    # LAZ保存
    print(f"[情報] LAZ保存中: {output}")
    save_laz(points, intensities, output)

    size = os.path.getsize(output)
    print(f"[完了] {output} ({size:,}バイト, {len(points):,}点)")


if __name__ == "__main__":
    main()
