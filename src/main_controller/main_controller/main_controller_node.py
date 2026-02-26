# main_controller_node.py

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_pkg.action import MoveBoxes, MovePallet
from geometry_msgs.msg import Pose


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller_node')

        self.mani2_client = ActionClient(
            self, 
            MoveBoxes, 
            'mani2_classifier_client'
            )
        
        self.amr_client = ActionClient(
            self, 
            MovePallet, 
            'move_pallet_client'
            )

        self.pallet_state = {}  # {'P1': 'Ready'}
        self.current_pallet_id = 'P1'

        self._loop = asyncio.get_event_loop()
        self.create_timer(0.5, self._timer_callback)

        self._workflow_started = False

    def _timer_callback(self):
        if not self._workflow_started:
            self._workflow_started = True
            self._loop.create_task(self.run_workflow())

    async def run_workflow(self):
        pallet_id = self.current_pallet_id

        # 1) 협동로봇 2: 팔레트에 Fragile 2, Normal 2 적재
        success = await self.send_carry_boxes_goal(pallet_id)
        if not success:
            self.get_logger().error('MoveBoxes failed, aborting workflow')
            return

        self.pallet_state[pallet_id] = 'Ready'

        # 2) 팔레트 Ready면 이동로봇 호출
        if self.pallet_state.get(pallet_id) == 'Ready':
            success = await self.send_move_pallet_goal(pallet_id)
            if not success:
                self.get_logger().error('MovePallet failed')
                return

        self.get_logger().info('Workflow done.')

    async def send_carry_boxes_goal(self, pallet_id: str) -> bool:
        await self.mani2_client.wait_for_server()

        goal = MoveBoxes.Goal()
        goal.pallet_id = pallet_id
        goal.fragile_count = 2
        goal.normal_count = 2

        self.get_logger().info('Sending MoveBoxes goal to mani2...')
        send_future = self.mani2_client.send_goal_async(
            goal,
            feedback_callback = self.carry_feedback_cb
        )
        goal_handle = await send_future

        if not goal_handle.accepted: 
            self.get_logger().warn('MoveBoxes goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        result = await result_future
        self.get_logger().info(f'MoveBoxes result: {result.result.message}')
        return bool(result.result.success)

    def carry_feedback_cb(self, feedback_msg):

        fb = feedback_msg.feedback

        self.get_logger().info(
            f'[MoveBoxes feedback] Fragile # = {fb.fragile_loaded}, '
            f'Normal # = {fb.normal_loaded}, status = {fb.status}'
        )

    async def send_move_pallet_goal(self, pallet_id: str) -> bool:
        await self.amr_client.wait_for_server()

        goal = MovePallet.Goal()
        goal.pallet_id = pallet_id

        # 예시 좌표 (실제 프로젝트에 맞게 수정)
        pallet_pose = Pose()
        pallet_pose.position.x = 1.0
        pallet_pose.position.y = 0.0
        pallet_pose.position.z = 0.0

        goal_pose = Pose()
        goal_pose.position.x = 5.0
        goal_pose.position.y = 0.0
        goal_pose.position.z = 0.0

        goal.pallet_pose = pallet_pose
        goal.goal_pose = goal_pose

        self.get_logger().info('Sending MovePallet goal to AMR...')
        send_future = self.amr_client.send_goal_async(
            goal,
            feedback_callback=self.move_feedback_cb
        )

        goal_handle = await send_future

        if not goal_handle.accepted:
            self.get_logger().warn('MovePallet goal rejected')
            return False

        result_future = goal_handle.get_result_async()

        result = await result_future
        self.get_logger().info(f'MovePallet result: {result.result.message}')
        
        return bool(result.result.success)

    def move_feedback_cb(self, feedback_msg):

        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[MovePallet feedback] dist = {fb.distance_remaining:.2f}, '
            f'eta = {fb.eta_seconds:.1f}, status = {fb.status}'
        )


def main(args=None):
    
    rclpy.init(args=args)
    node = MainController()

    loop = asyncio.get_event_loop()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            loop.run_until_complete(asyncio.sleep(0.01))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
