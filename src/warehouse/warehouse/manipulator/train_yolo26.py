'''
import os
from ultralytics import YOLO
import urllib.request


MODEL_NAME = "yolo26s-seg.pt"
MODEL_PATH = os.path.join(os.getcwd(), MODEL_NAME)

model = YOLO(MODEL_PATH)

results = model.train(
    data="/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/dataset_26/data.yaml",
    epochs=500,
    patience=25,      
    imgsz=640,
    batch=32,
    workers=4,
    device=0,         # GPU 0번
    project="/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output_yolo26/train",
    name="yolo26_seg",
    pretrained=True,
    save=True         # 학습 완료 후 모델 저장 여부 확인
)


best_model_path = os.path.join(
    "/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output_yolo26/train",
    "yolo26_seg",
    "weights",
    "best.pt"
)

if os.path.exists(best_model_path):
    print("\n✅ 학습 완료")
    print(f"📌 best.pt 경로:\n{best_model_path}")
else:
    print("❌ best.pt 파일을 찾을 수 없습니다.")'''

import os
import torch  # CUDA 체크를 위해 추가
from ultralytics import YOLO

# 1. CUDA 초기화 오류 방지를 위한 환경 변수 설정 (코드 최상단)
os.environ['CUDA_LAUNCH_BLOCKING'] = "1"

# 2. 장치(Device) 자동 설정
# 시스템이 GPU를 인식하면 'cuda'를, 아니면 'cpu'를 사용하도록 방어 코드 작성
if torch.cuda.is_available():
    device_to_use = 0  # 혹은 'cuda'
    print(f"✅ CUDA 사용 가능: {torch.cuda.get_device_name(0)}")
else:
    device_to_use = 'cpu'
    print("⚠️ CUDA를 찾을 수 없어 CPU 모드로 전환합니다. (드라이버 확인 필요)")

MODEL_NAME = "yolo26s-seg.pt"
MODEL_PATH = os.path.join(os.getcwd(), MODEL_NAME)

# 모델 로드
model = YOLO(MODEL_PATH)

# 3. 학습 실행
results = model.train(
    data="/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/dataset_26/data.yaml",
    epochs=500,
    patience=25,      
    imgsz=640,
    batch=16,         
    workers=4,
    device=device_to_use, 
    project="/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output_yolo26/train",
    name="yolo26_seg",
    pretrained=True,
    save=True
)

# 4. 결과 확인 경로 설정
best_model_path = os.path.join(
    "/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output_yolo26/train",
    "yolo26_seg",
    "weights",
    "best.pt"
)

if os.path.exists(best_model_path):
    print("\n✅ 학습 완료")
    print(f"📌 best.pt 경로:\n{best_model_path}")
else:
    print("\n❌ 학습이 정상적으로 완료되지 않았거나 best.pt 파일을 찾을 수 없습니다.")