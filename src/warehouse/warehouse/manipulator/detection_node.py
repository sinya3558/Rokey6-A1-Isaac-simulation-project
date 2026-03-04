import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class IsaacCameraTest(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        
        # '/rgb' 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.listener_callback,
            10)
        
        # detection 결과 퍼블리셔 추가
        self.detection_publisher = self.create_publisher(Bool, '/detection_result', 10)
        
        YOLO26_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output_yolo26/train/yolo26_seg/weights/best.pt'
        YOLO8_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/yolov8/fragile_8_train_ver3/weights/best.pt'
        YOLO11_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/yolov11/fragile_11_train2/weights/best.pt'
        
        self.model = YOLO(YOLO8_PATH)

    def listener_callback(self, data):
        self.get_logger().info('이미지를 수신 중--') 
    
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        results = self.model.predict(current_frame, conf=0.3, verbose=False)

        # detection 여부 확인 후 퍼블리시
        detected = len(results[0].boxes) > 0
        
        msg = Bool()
        msg.data = detected
        self.detection_publisher.publish(msg)
        
        if detected:
            self.get_logger().info(f'Detection! 감지된 객체 수: {len(results[0].boxes)}개')

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
