
from rfdetr import RFDETRBase
import os
from PIL import Image


model = RFDETRBase()

dataset_path = "/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/coco_dataset"
project_path = "/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/my_test/train"
run_name = "rf-detr_test"

# RF-DETR 학습
model.train(
    dataset_dir= dataset_path,
    epochs= 100,               
    batch_size= 2,             
    imgsz= 640,                
    workers= 1,                
    device= 0,                 

    grad_accum_steps= 8,        # batch_size(2) * grad_accum_steps(8) => batch_size 16 효과
    
    # [RF-DETR 최적화 튜닝]
    lr= 1e-4,                  # Transformer는 낮은 Learning Rate에서 안정적으로 수렴함
    optimizer= "AdamW",        # CNN용 SGD보다 Transformer의 어텐션 학습에 훨씬 유리
    weight_decay= 0.05,        # 강력한 규제로 Isaac Sim 환경의 과적합(Overfitting) 방지
    warmup_epochs= 5,          # 초반 5 에폭 동안 학습률을 서서히 올려 모델 폭주 방지
    
    num_queries= 300,          # 동시에 찾을 수 있는 객체 수
    project=project_path,
    name=run_name,
    pretrained=True          
)

# 추론 및 검증
test_img_path = os.path.join(dataset_path, "test/some_image.jpg")

if os.path.exists(test_img_path):
    image = Image.open(test_img_path)

    # 실시간성 확보를 위한 가속화 (TensorRT/ONNX 최적화 단계)
    # Isaac Sim 상단에서 추론 성능을 극대화하기 위해 반드시 필요
    model.optimize_for_inference() 
    
    # 높은 신뢰도(0.5 이상)의 결과만 출력
    detections = model.predict(image, threshold=0.5)
    
    print("-" * 30)
    print(f"검출된 객체 수: {len(detections)}")
    print(f"상세 정보: {detections}")
    print("-" * 30)
else:
    print(f"경고: 테스트 이미지를 찾을 수 없습니다: {test_img_path}")