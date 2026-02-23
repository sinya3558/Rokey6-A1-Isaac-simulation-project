import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import cv2
import torch  # 가중치 로드를 위해 필수 추가
import supervision as sv
from rfdetr import RFDETRBase

class IsaacCameraTest(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.bridge = CvBridge()
        
        # 1. Supervision 시각화 도구 초기화
        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()
        
        # 2. 토픽 구독
        self.subscription = self.create_subscription(
            ROSImage,
            '/rgb',
            self.listener_callback,
            10)
        
        # 3. 모델 로드 및 가중치 수동 주입 (에러 해결 핵심)
        RF_PATH = '/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output/eval/latest.pth'
        
        self.get_logger().info('RF-DETR 모델 로드 시작...')
        self.model = RFDETRBase()
        
        try:
            # PyTorch 표준 방식으로 가중치 파일을 읽어옵니다.
            checkpoint = torch.load(RF_PATH, map_location='cpu')
            
            # 체크포인트 내부 구조에 따라 state_dict 추출
            if isinstance(checkpoint, dict) and 'model' in checkpoint:
                state_dict = checkpoint['model']
            elif isinstance(checkpoint, dict) and 'state_dict' in checkpoint:
                state_dict = checkpoint['state_dict']
            else:
                state_dict = checkpoint
            
            # 모델에 가중치 주입
            self.model.load_state_dict(state_dict)
            self.model.eval()  # 추론 모드 설정
            
            if torch.cuda.is_available():
                self.model.cuda()
                
            self.get_logger().info('가중치 로드 및 GPU 이동 완료!')
            
        except Exception as e:
            self.get_logger().error(f'가중치 로드 중 오류 발생: {e}')

        self.classes = ["Fragile-Label"] 

    def listener_callback(self, data):
        # ROS2 이미지를 OpenCV BGR로 변환
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # RF-DETR 추론 (supervision.Detections 객체 반환)
        results = self.model.predict(current_frame, conf=0.3, verbose=False)
        
        # 결과가 리스트인 경우 첫 번째 요소 추출
        detections = results[0] if isinstance(results, list) else results

        # 라벨 생성
        if self.classes and len(detections.class_id) > 0:
            labels = [
                f"{self.classes[class_id] if class_id < len(self.classes) else class_id} {conf:.2f}" 
                for class_id, conf in zip(detections.class_id, detections.confidence)
            ]
        else:
            labels = [f"ID:{class_id} {conf:.2f}" for class_id, conf in zip(detections.class_id, detections.confidence)]

        # Supervision 시각화 적용
        annotated_frame = self.box_annotator.annotate(
            scene=current_frame.copy(), 
            detections=detections
        )
        annotated_frame = self.label_annotator.annotate(
            scene=annotated_frame, 
            detections=detections, 
            labels=labels
        )

        # 결과 화면 출력
        cv2.imshow("RF-DETR Detection", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacCameraTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()