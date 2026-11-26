#!/bin/bash
# ==============================================================================
# OMO-R1 / Nav2 연속 실험용 상태 리셋 스크립트 (start_all.sh 유지 상태 재사용)
# ==============================================================================
# 다음 회차 rosbag 녹화를 시작하기 전에 실행하세요:
#   1) 활성 Nav2 Goal 일괄 취소 (/navigate_to_pose/_action/cancel_goal)
#   2) /cmd_vel (0, 0) 전송 및 정지 확인
#   3) /reset_odom (0, 0, 0) 호출 및 /odom 0점 복귀 확인
#   4) Local / Global Costmap 전체 초기화 후 최신 /scan 반영 대기
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source /opt/ros/humble/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi

python3 - <<'PY'
import math
import sys
import time
import rclpy
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from omo_r1_interfaces.srv import ResetOdom


class TrialResetter(Node):
    def __init__(self):
        super().__init__("trial_resetter")
        self.latest_odom = None
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cancel_cli = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.reset_odom_cli = self.create_client(ResetOdom, "/reset_odom")
        self.clear_local_cli = self.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")
        self.clear_global_cli = self.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _call_sync(self, cli, req, name: str, timeout_s: float = 2.0):
        if not cli.wait_for_service(timeout_sec=timeout_s):
            print(f"⚠️  [{name}] 서비스 미응답 ({timeout_s:.1f}s)")
            return None
        fut = cli.call_async(req)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not fut.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            print(f"⚠️  [{name}] 서비스 호출 타임아웃")
            return None
        return fut.result()

    def _wait_for_stable_odom(self, require_origin=False):
        deadline = time.monotonic() + 1.5
        previous_stamp = self.get_clock().now().nanoseconds
        stable_since = None
        zero_twist = Twist()
        while rclpy.ok() and time.monotonic() < deadline:
            self.cmd_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            msg = self.latest_odom
            if msg is None:
                continue
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            now = self.get_clock().now().nanoseconds
            if stamp <= previous_stamp or not 0 <= now - stamp <= 150_000_000:
                continue
            if stamp - previous_stamp > 150_000_000:
                stable_since = None
            previous_stamp = stamp

            twist = msg.twist.twist
            stopped = abs(twist.linear.x) < 0.02 and abs(twist.angular.z) < 0.02
            at_origin = True
            if require_origin:
                pos = msg.pose.pose.position
                q = msg.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                at_origin = abs(pos.x) < 0.05 and abs(pos.y) < 0.05 and abs(yaw) < math.radians(2.0)
            if not (stopped and at_origin):
                stable_since = None
                continue
            if stable_since is None:
                stable_since = stamp
            if stamp - stable_since >= 250_000_000:
                return True
        return False

    def run(self) -> int:
        print("🔄 [리셋 1/4] 활성 Nav2 Goal 일괄 취소 중...")
        cancel_req = CancelGoal.Request()
        cancel_res = self._call_sync(self.cancel_cli, cancel_req, "cancel_goal", timeout_s=2.0)
        if cancel_res is not None:
            print(f"   ✅ Goal 취소 응답 완료 (취소 건수: {len(cancel_res.goals_canceling)})")

        print("🛑 [리셋 2/4] /cmd_vel (0, 0) 정지 전송 및 속도 0 확인 중...")
        if not self._wait_for_stable_odom():
            print("❌ 새 /odom에서 정지를 확인하지 못했습니다. 다음 주행을 중단합니다.", file=sys.stderr)
            return 1
        print("   ✅ 로봇 정지 확인 (vx≈0, wz≈0)")
        if cancel_res is None:
            return 1

        print("📍 [리셋 3/4] /reset_odom (0, 0, 0) 초기화 중...")
        odom_req = ResetOdom.Request()
        odom_req.x = 0.0
        odom_req.y = 0.0
        odom_req.theta = 0.0
        if self._call_sync(self.reset_odom_cli, odom_req, "reset_odom", timeout_s=2.0) is None:
            return 1

        print("🧹 [리셋 4/4] Local / Global Costmap 초기화 중...")
        self._call_sync(self.clear_local_cli, ClearEntireCostmap.Request(), "clear_local_costmap", timeout_s=2.0)
        self._call_sync(self.clear_global_cli, ClearEntireCostmap.Request(), "clear_global_costmap", timeout_s=2.0)
        time.sleep(0.5)  # 현재 시작 위치 기준 최신 /scan이 costmap에 재반영될 시간 확보
        if not self._wait_for_stable_odom(require_origin=True):
            print("❌ 리셋 후 새 /odom에서 원점과 정지 상태를 확인하지 못했습니다.", file=sys.stderr)
            return 1
        print("   ✅ /odom 초기화 확인 (x≈0, y≈0, yaw≈0°, vx≈0, wz≈0)")
        print("✅ 연속 실험 준비 완료! 이제 ./scripts/record_rosbag.sh 로 녹화를 시작하세요.")
        return 0


def main():
    rclpy.init()
    node = TrialResetter()
    rc = 1
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
PY
