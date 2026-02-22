import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from rfdetr import RFDETRBase

class IsaacCameraTest(Node):
    def __init__(self):
        super().__init__('isaac_camera_test')
        self.bridge = CvBridge()
        
        # '/rgb' 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.listener_callback,
            10)
        
        RF_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output/eval/latest.pth'
        YOLO_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output_yolo26/train/yolo26_seg/weights/best.pt'
        # YOLO 일때 사용
        # self.model = YOLO(YOLO_PATH)
        # RF-DETR 일때 사용
        self.model = RFDETRBase(RF_PATH)

    def listener_callback(self, data):
        # ROS2 이미지 메시지를 OpenCV(BGR) 이미지로 변환
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # conf=0.5: 확률이 50% 이상인 것만 표시
        # verbose=False: 터미널에 추론 로그가 너무 많이 찍히는 것을 방지
        results = self.model.predict(current_frame, conf=0.5, verbose=False)

        
        # BGR
        annotated_frame = results[0].plot()

        cv2.imshow("Fragile Test", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacCameraTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()