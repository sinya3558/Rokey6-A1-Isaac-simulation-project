
# import ultralytics
# from ultralytics import YOLO
# import os
# from IPython.display import display, Image

# # GPU 사용 가능 여부 확인
# ultralytics.checks()

# from google.colab import drive
# drive.mount('/content/drive')

# # 결과를 저장할 경로 설정 (원하는 경로로 수정 가능)
# ROOT_DIR = '/content/drive/MyDrive/YOLO_Training'
# if not os.path.exists(ROOT_DIR):
#     os.makedirs(ROOT_DIR)

# # 모델 선택 (n: nano, s: small, m: medium 등)
# model = YOLO('yolo11n.pt') # 또는 사용하고자 하는 버전의 모델 파일

# # 학습 파라미터 설정 및 실행
# model.train(
#     data='coco8.yaml',      # 데이터셋 설정 파일 경로 (본인의 yaml 파일로 변경)
#     epochs=25,               # 학습 횟수
#     imgsz=640,               # 이미지 크기
#     batch=16,                # 배치 사이즈
#     project=ROOT_DIR,        # 결과가 저장될 드라이브 경로
#     name='my_yolo_model',    # 실험 이름
#     device=0,                # GPU 사용
#     plots=True               # 성능 그래프 생성 활성화
# )

# # 학습 결과 그래프가 저장된 경로 (학습 시 설정한 경로/이름 확인)
# results_path = os.path.join(ROOT_DIR, 'my_yolo_model/results.png')

# if os.path.exists(results_path):
#     print("--- 학습 결과 그래프 ---")
#     display(Image(filename=results_path, width=800))
# else:
#     print("그래프 파일을 찾을 수 없습니다. 학습 경로를 확인해주세요.")

# import yaml

# args_path = os.path.join(ROOT_DIR, 'my_yolo_model/args.yaml')

# if os.path.exists(args_path):
#     with open(args_path, 'r') as f:
#         config = yaml.safe_load(f)
#         print("### 학습에 사용된 주요 파라미터 ###")
#         print(f"Model: {config.get('model')}")
#         print(f"Epochs: {config.get('epochs')}")
#         print(f"Image Size: {config.get('imgsz')}")
#         print(f"Optimizer: {config.get('optimizer')}")
# else:
#     print("설정 파일을 찾을 수 없습니다.")

import os
from ultralytics import YOLO
import urllib.request

# ----------------------------------------
# 1️⃣ 사용할 모델 이름
# ----------------------------------------
MODEL_NAME = "yolo26s-seg.pt"
MODEL_PATH = os.path.join(os.getcwd(), MODEL_NAME)


# ----------------------------------------
# 3️⃣ 모델 로드
# ----------------------------------------
model = YOLO(MODEL_PATH)

# ----------------------------------------
# 4️⃣ 학습 실행 (네 hyperparameter 그대로 적용)
# ----------------------------------------
results = model.train(
    data="/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/dataset_26/data.yaml",
    epochs=100,
    imgsz=640,
    batch=16,
    workers=4,
    device=0,   # GPU 0번 (CPU는 "cpu")
    project="/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/manipulator/output_yolo26/train",
    name="yolo26_seg",
    pretrained=True
)

# ----------------------------------------
# 5️⃣ best.pt 경로 출력
# ----------------------------------------
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
    print("❌ best.pt 파일을 찾을 수 없습니다.")