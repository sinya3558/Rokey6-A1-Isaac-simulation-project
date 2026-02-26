# move_pallet_server.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration

from action_pkg.action import MovePallet
from geometry_msgs.msg import Pose

import math
import time


class MovePalletActionServer(Node):
    def __init__(self):
        super().__init__('deliver_pallet_server')

        self._action_server = ActionServer(
            self,
            MovePallet,     # msg type
            'move_pallet',  # 액션 이름
            self.execute_callback
        )

        self.declare_parameter('speed_mps', 0.5)
        self._current_pose = Pose()

    def _distance(self, a: Pose, b: Pose) -> float:
        dx = a.position.x - b.position.x
        dy = a.position.y - b.position.y
        dz = a.position.z - b.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    async def execute_callback(self, goal_handle):
        self.get_logger().info(
            f'Received MovePallet goal: pallet_id = {goal_handle.request.pallet_id}'
        )

        feedback = MovePallet.Feedback()    # 피드백 메시지 객체 생성
        result = MovePallet.Result() 

        speed = float(self.get_parameter('speed_mps').value)
        if speed <= 0.0:
            speed = 0.5

        pallet_pose = goal_handle.request.pallet_pose 
        goal_pose = goal_handle.request.goal_pose

        # === STEP 1: 팔레트 위치까지 이동 ===
        start_time = self.get_clock().now()
        dist_total = self._distance(self._current_pose, pallet_pose)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Canceled while going to pallet'
                self.get_logger().info(result.message)
                return result

            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            traveled = min(dist_total, speed * elapsed)
            remaining = max(0.0, dist_total - traveled)

            # 단순 직선 보간 (실제 구현에서는 네비게이션/경로계획으로 대체)
            if dist_total > 1e-3:
                ratio = traveled / dist_total
            else:
                ratio = 1.0
            self._current_pose.position.x = (
                (1.0 - ratio) * self._current_pose.position.x
                + ratio * pallet_pose.position.x
            )
            self._current_pose.position.y = (
                (1.0 - ratio) * self._current_pose.position.y
                + ratio * pallet_pose.position.y
            )
            self._current_pose.position.z = (
                (1.0 - ratio) * self._current_pose.position.z
                + ratio * pallet_pose.position.z
            )

            eta = remaining / speed if speed > 1e-3 else 0.0

            feedback.distance_remaining = float(remaining)
            feedback.eta_seconds = float(eta)
            feedback.current_pose = self._current_pose
            feedback.status = 'moving_to_pallet'
            goal_handle.publish_feedback(feedback)

            if remaining < 0.01:
                break

            await rclpy.sleep(Duration(seconds=0.1))

        self.get_logger().info('Arrived at pallet. Picking up.')
        # 여기서 Isaac Sim API로 fixed joint 생성 or 팔레트 삭제 처리
        # attach_pallet_to_robot()

        # === STEP 2: 목표 위치까지 이동 ===
        start_time = self.get_clock().now()
        dist_total = self._distance(self._current_pose, goal_pose)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Canceled while going to goal'
                self.get_logger().info(result.message)
                return result

            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            traveled = min(dist_total, speed * elapsed)
            remaining = max(0.0, dist_total - traveled)

            if dist_total > 1e-3:
                ratio = traveled / dist_total
            else:
                ratio = 1.0
            self._current_pose.position.x = (
                (1.0 - ratio) * self._current_pose.position.x
                + ratio * goal_pose.position.x
            )
            self._current_pose.position.y = (
                (1.0 - ratio) * self._current_pose.position.y
                + ratio * goal_pose.position.y
            )
            self._current_pose.position.z = (
                (1.0 - ratio) * self._current_pose.position.z
                + ratio * goal_pose.position.z
            )

            eta = remaining / speed if speed > 1e-3 else 0.0

            feedback.distance_remaining = float(remaining)
            feedback.eta_seconds = float(eta)
            feedback.current_pose = self._current_pose
            feedback.status = 'moving_to_dest'
            goal_handle.publish_feedback(feedback)

            if remaining < 0.01:
                break

            await rclpy.sleep(Duration(seconds=0.1))

        # 여기서 팔레트 내려놓기 / joint 해제
        # detach_pallet()

        result.success = True
        result.message = 'Arrived'
        goal_handle.succeed()
        self.get_logger().info(result.message)
        return result


def main(args=None):

    rclpy.init(args=args)

    node = MovePalletActionServer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
