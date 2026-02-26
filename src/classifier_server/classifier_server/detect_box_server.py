# detect_box_server.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration

from action_pkg.action import MoveBoxes


class CarryBoxesActionServer(Node):
    def __init__(self):
        super().__init__('classifier_server')

        self._action_server = ActionServer(
            self,
            MoveBoxes,
            'classifies_boxes',  # action name 
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(
            f'Received MoveBoxes''s goal : pallet = {req.pallet_id}, '
            f'Fragile # = {req.fragile_count}, Normal # = {req.normal_count}'
        )

        feedback = MoveBoxes.Feedback() # init feedback msg
        result = MoveBoxes.Result()

        target_fragile = int(req.fragile_count)
        target_normal = int(req.normal_count)

        fragile_loaded = 0
        normal_loaded = 0

        # 1초마다 박스 하나씩 처리한다고 가정한 경우.. 실제로는 협동로봇 pick & place + 카메라 분류 결과를 사용해야 함!
        while rclpy.ok():
            if goal_handle.is_cancel_requested: # 취소 요청 들어오면, 

                goal_handle.canceled() #update status

                result.success = False
                result.message = 'Canceled while loading boxes'
                self.get_logger().info(result.message)
                
                return result

            # 여기서 실제 로봇 pick & place + 카메라 분류 결과를 사용해야 함
            # 예시 로직: F, N 번갈아 가며 채움
            if fragile_loaded < target_fragile:
                fragile_loaded += 1
                status_str = 'placing_fragile(취급주의)'
            elif normal_loaded < target_normal:
                normal_loaded += 1
                status_str = 'placing_normal(일반)'
            else:
                # 목표 개수 달성
                break

            feedback.fragile_loaded = fragile_loaded    # update feedback
            feedback.normal_loaded = normal_loaded
            feedback.status = status_str

            goal_handle.publish_feedback(feedback)      # publish feedback!

            await rclpy.sleep(Duration(seconds = 1.0))  # simulate time taken to process each box

        result.success = True
        result.message = (
            f'Pallet [{req.pallet_id}] ready: '
            f'Fragile # = {fragile_loaded}, Normal # ={normal_loaded}'
        )
        goal_handle.succeed()
        self.get_logger().info(result.message)
        return result


def main(args=None):

    rclpy.init(args=args)

    node = CarryBoxesActionServer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
