#!/usr/bin/env python3
import os
import sys
import glob
import sqlite3
import math
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def analyze_bag(bag_dir):
    if not os.path.exists(bag_dir):
        print(f"❌ Error: Bag directory not found: {bag_dir}")
        return

    db3_files = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not db3_files:
        print(f"❌ Error: No .db3 files found in {bag_dir}")
        return

    db_path = db3_files[0]
    conn = sqlite3.connect(db_path)
    c = conn.cursor()

    # Get topic id for /odom
    c.execute("SELECT id FROM topics WHERE name = '/odom'")
    row = c.fetchone()
    if not row:
        print("❌ Error: /odom topic not found in rosbag.")
        return
    topic_id = row[0]

    OdomMsg = get_message('nav_msgs/msg/Odometry')

    c.execute("SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp ASC", (topic_id,))
    rows = c.fetchall()

    if not rows:
        print("❌ Error: No messages on /odom topic.")
        return

    first_move_t = None
    last_move_t = None
    max_v = 0.0
    max_w = 0.0
    max_x = 0.0
    max_y = 0.0
    x0, y0 = None, None

    for ts, data in rows:
        msg = deserialize_message(data, OdomMsg)
        t_sec = ts / 1e9
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.sqrt(vx * vx + vy * vy)
        wz = abs(msg.twist.twist.angular.z)

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        if x0 is None:
            x0, y0 = px, py

        dx = px - x0
        dy = abs(py - y0)

        # 바퀴가 실제로 굴러간 순간 (v > 0.02 m/s)
        if v > 0.02:
            if first_move_t is None:
                first_move_t = t_sec
            last_move_t = t_sec
            max_v = max(max_v, v)
            max_w = max(max_w, wz)
            max_x = max(max_x, dx)
            max_y = max(max_y, dy)

    total_bag_duration = (rows[-1][0] - rows[0][0]) / 1e9

    if first_move_t and last_move_t and last_move_t >= first_move_t:
        pure_motion_time = last_move_t - first_move_t
    else:
        pure_motion_time = 0.0

    print("=" * 65)
    print(f"📦 [Rosbag 실측 분석 결과: {os.path.basename(bag_dir)}]")
    print("=" * 65)
    print(f"⏱️  전체 녹화 시간 (Total Bag Time)   : {total_bag_duration:.2f} 초")
    print(f"🚀  순수 주행 시간 (Pure Motion Time) : {pure_motion_time:.2f} 초 (대기 {total_bag_duration - pure_motion_time:.2f}초 제외)")
    print(f"📏  전진 주행 거리 (Travel X)         : {max_x:.2f} m")
    print(f"↔️  최대 회피 이탈폭 (Max Lateral Y)  : {max_y:.3f} m")
    print(f"🏎️  최고 선속도 (Max Linear Speed)   : {max_v:.2f} m/s")
    print(f"🔄  최고 각속도 (Max Angular Speed)  : {max_w:.3f} rad/s")
    print("=" * 65)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("사용법: python3 scripts/calc_bag_time.py <rosbag_폴더_경로>")
        sys.exit(1)
    analyze_bag(sys.argv[1])
