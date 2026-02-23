import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        
        # RF_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output/eval/latest.pth'
        YOLO26_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output_yolo26/train/yolo26_seg/weights/best.pt'
        YOLO8_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/yolov8/fragile_8_train/weights/best.pt'
        YOLO11_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/yolov11/fragile_11_train2/weights/best.pt'
        
        # YOLO 일때 사용
        self.model = YOLO(YOLO8_PATH)
        
        # RF-DETR 일때 사용
        # self.model = RFDETRBase()
        # self.model.load_weights(RF_PATH)

    def listener_callback(self, data):
        ##
        self.get_logger().info('이미지를 수신 중--') 
    
        # ROS2 이미지 메시지를 OpenCV(BGR) 이미지로 변환
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # conf=0.n: 확률이 n0% 이상인 것만 표시
        # verbose=False: 터미널에 추론 로그가 너무 많이 찍히는 것을 방지
        results = self.model.predict(current_frame, conf= 0.3, verbose= False)

        ## 이건 RF-DETR 꺼 
        # if len(results) > 0:
        #     annotated_frame = draw_detections(current_frame, results[0])
        # else:
        #     annotated_frame = current_frame
    
        # BGR
        annotated_frame = results[0].plot()   ## 얘는 yolo 꺼

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