#!/usr/bin/env python3
"""
==============================================================================
Autonomous AMR Driving & Navigation Experiment Logger
Target: OMORobot R1 Mini / V-LiDAR Autonomous System (ROS 2 Humble)
==============================================================================
This tool records real-time driving logs, trajectory, V-LiDAR scans, and VSLAM
tracking health during autonomous navigation trials.

Outputs saved in experiments/logs/<YYYYMMDD_HHMMSS_scenario>/:
1. metadata.json       : Test scenario config, parameters, start/end time
2. driving_log.csv     : 5Hz time-series of pose, speed, tracking, and obstacle distances
3. summary_report.md   : Automated experiment summary with pass/fail evaluation
"""

import os
import sys
import time
import math
import argparse
import json
import csv
from datetime import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

class ExperimentLogger(Node):
    def __init__(self, scenario_name, map_name, output_dir):
        super().__init__('experiment_logger')
        self.scenario_name = scenario_name
        self.map_name = map_name
        self.output_dir = output_dir

        # 최신 센서 데이터 상태
        self.latest_scan = None
        self.latest_odom = None
        self.latest_cmd_vel = None
        self.latest_vslam = None
        self.vslam_last_time = 0.0

        # 메트릭 버퍼
        self.trajectory = []  # (x, y)
        self.speeds = []
        self.min_scan_dists = []
        self.tracking_states = []
        self.start_time = time.time()
        self.total_distance = 0.0
        self.prev_x = None
        self.prev_y = None

        # 구독자 설정
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(PoseStamped, '/run_slam/camera_pose', self.vslam_cb, 10)

        # CSV 파일 준비
        self.csv_path = os.path.join(output_dir, "driving_log.csv")
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "timestamp_s", "odom_x_m", "odom_y_m", "odom_yaw_deg",
            "linear_v_mps", "angular_w_rads", "cmd_v_mps", "cmd_w_rads",
            "vslam_tracking", "min_obs_dist_m", "center_obs_dist_m"
        ])

        # 5Hz (0.2s) 로깅 타이머
        self.timer = self.create_timer(0.2, self.logging_step)
        self.get_logger().info(f"🚀 Experiment Logger started: [{self.scenario_name}] on map [{self.map_name}]")
        self.get_logger().info(f"📁 Saving logs to: {self.output_dir}")

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def cmd_cb(self, msg: Twist):
        self.latest_cmd_vel = msg

    def vslam_cb(self, msg: PoseStamped):
        self.latest_vslam = msg
        self.vslam_last_time = time.time()

    def logging_step(self):
        now = time.time()
        elapsed = now - self.start_time

        # 1. Odometry 파싱
        ox, oy, oyaw, lin_v, ang_w = 0.0, 0.0, 0.0, 0.0, 0.0
        if self.latest_odom:
            pos = self.latest_odom.pose.pose.position
            ox, oy = pos.x, pos.y
            q = self.latest_odom.pose.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            oyaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
            lin_v = self.latest_odom.twist.twist.linear.x
            ang_w = self.latest_odom.twist.twist.angular.z

            if self.prev_x is not None:
                dx = ox - self.prev_x
                dy = oy - self.prev_y
                dist = math.hypot(dx, dy)
                if dist < 1.0:  # jump 방지
                    self.total_distance += dist
            self.prev_x, self.prev_y = ox, oy
            self.trajectory.append((ox, oy))
            self.speeds.append(abs(lin_v))

        # 2. Command velocity 파싱
        cmd_v, cmd_w = 0.0, 0.0
        if self.latest_cmd_vel:
            cmd_v = self.latest_cmd_vel.linear.x
            cmd_w = self.latest_cmd_vel.angular.z

        # 3. VSLAM 추적 상태 (최근 1초 이내 pose 수신 여부)
        is_tracking = (now - self.vslam_last_time) < 1.0 if self.vslam_last_time > 0 else False
        self.tracking_states.append(is_tracking)

        # 4. Scan 파싱
        min_d = 999.0
        center_d = 999.0
        if self.latest_scan and len(self.latest_scan.ranges) > 0:
            ranges = np.array(self.latest_scan.ranges)
            valid = ranges[np.isfinite(ranges)]
            if valid.size > 0:
                min_d = float(np.min(valid))
                self.min_scan_dists.append(min_d)
            # Center sector (채널 47~93)
            c_slice = ranges[47:94] if len(ranges) >= 94 else ranges
            valid_c = c_slice[np.isfinite(c_slice)]
            if valid_c.size > 0:
                center_d = float(np.min(valid_c))

        # 5. CSV 기록
        self.csv_writer.writerow([
            f"{elapsed:.2f}", f"{ox:.3f}", f"{oy:.3f}", f"{oyaw:.1f}",
            f"{lin_v:.3f}", f"{ang_w:.3f}", f"{cmd_v:.3f}", f"{cmd_w:.3f}",
            1 if is_tracking else 0,
            f"{min_d:.2f}" if min_d < 999 else "inf",
            f"{center_d:.2f}" if center_d < 999 else "inf"
        ])

    def close(self):
        self.csv_file.close()
        total_time = time.time() - self.start_time

        # 메트릭 계산
        avg_speed = np.mean(self.speeds) if self.speeds else 0.0
        max_speed = np.max(self.speeds) if self.speeds else 0.0
        min_obs = np.min(self.min_scan_dists) if self.min_scan_dists else 999.0
        track_ratio = (sum(self.tracking_states) / len(self.tracking_states) * 100.0) if self.tracking_states else 0.0
        close_encounters = sum(1 for d in self.min_scan_dists if d < 0.5)

        # 안전 판정
        if min_obs < 0.25:
            safety_verdict = "⚠️ 충돌 위험 (접근거리 < 25cm)"
        elif min_obs < 0.5:
            safety_verdict = "⚡ 근접 회피 (접근거리 25~50cm)"
        else:
            safety_verdict = "✅ 안전 주행 (최소거리 >= 50cm)"

        # metadata.json 생성
        meta = {
            "scenario": self.scenario_name,
            "map": self.map_name,
            "start_time": datetime.fromtimestamp(self.start_time).strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": round(total_time, 2),
            "total_distance_m": round(self.total_distance, 3),
            "avg_speed_mps": round(float(avg_speed), 3),
            "max_speed_mps": round(float(max_speed), 3),
            "min_obstacle_dist_m": round(float(min_obs), 3) if min_obs < 999 else None,
            "vslam_tracking_rate_pct": round(track_ratio, 1)
        }
        with open(os.path.join(self.output_dir, "metadata.json"), 'w') as f:
            json.dump(meta, f, indent=2)

        # summary_report.md 생성
        report_md = f"""# 📑 자율주행 실험 요약 리포트 (Trial Summary)

- **시나리오명**: `{self.scenario_name}`
- **적용 맵**: `{self.map_name}`
- **실험 일시**: {meta['start_time']}
- **총 주행 시간**: **{total_time:.1f} 초**
- **총 이동 거리**: **{self.total_distance:.2f} m**
- **데이터 폴더**: `{self.output_dir}`

---

## 🚗 1. 주행 및 기동 성능
| 지표 | 측정값 | 설명 |
| :--- | :--- | :--- |
| **평균 주행 속도** | **{avg_speed:.2f} m/s** | 주행 중 평균 선속도 |
| **최대 주행 속도** | **{max_speed:.2f} m/s** | 순간 최대 속도 |
| **VSLAM 추적 성공률** | **{track_ratio:.1f} %** | 카메라 특징점 위치추정 유지율 |

---

## 🛡️ 2. V-LiDAR 장애물 감지 및 안전성 평가
| 지표 | 측정값 | 판정 |
| :--- | :--- | :--- |
| **장애물 최근접 거리** | **{min_obs:.2f} m** | {safety_verdict} |
| **근접 경보 횟수 (<0.5m)** | **{close_encounters} 회** | 장애물 밀접 통과 횟수 |
| **주행 결과 평가** | **{'통과 (PASS)' if min_obs >= 0.25 and track_ratio >= 70 else '검토 필요 (RECHECK)'}** | 종합 안전 판정 |

---

## 📁 저장된 로그 파일 목록
1. `metadata.json`: 실험 세부 설정 및 집계 수치
2. `driving_log.csv`: 5Hz 주기 (위치, 속도, VSLAM, 라이다 거리) 시계열 데이터
"""
        with open(os.path.join(self.output_dir, "summary_report.md"), 'w') as f:
            f.write(report_md)

        self.get_logger().info("\n" + "=" * 65)
        self.get_logger().info("🏁 실험 로깅이 완료되었습니다!")
        self.get_logger().info(f"📊 이동 거리: {self.total_distance:.2f}m | 시간: {total_time:.1f}s | 최소 장애물 거리: {min_obs:.2f}m")
        self.get_logger().info(f"📄 요약 리포트: {os.path.join(self.output_dir, 'summary_report.md')}")
        self.get_logger().info("=" * 65)

def main():
    parser = argparse.ArgumentParser(description="Autonomous AMR Navigation Experiment Logger")
    parser.add_argument("--scenario", type=str, default="trial", help="Scenario description (e.g. corridor_box_avoidance)")
    parser.add_argument("--map", type=str, default="NTH4F", help="Map name (default: NTH4F)")
    parser.add_argument("--output-dir", type=str, default=None, help="Custom log directory")
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.output_dir or os.path.join(base_dir, "experiments", "logs", f"{now_str}_{args.scenario}")
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    logger_node = ExperimentLogger(args.scenario, args.map, out_dir)

    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        pass
    finally:
        logger_node.close()
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
