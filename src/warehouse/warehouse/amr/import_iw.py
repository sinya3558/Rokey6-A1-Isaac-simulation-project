# import_iw.py

from isaacsim import SimulationApp

# 1. 시뮬레이션 앱 초기화 (렌더링 모드 설정)
simulation_app = SimulationApp({"headless": False})

import sys
import numpy as np
import carb
import os
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.types import ArticulationAction # 
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
import omni.usd

# 환경 불러오기
assets_root_path = get_assets_root_path()
my_env_path = "/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/amr/usd/warehouse_room.usd" 
conveyer_usd_path = "/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/amr/conveyer_room_with_iw_hub.usd"
# iwhub_usd_path = "/home/rokey/Rokey6-A1-Isaac-simulation-project/src/warehouse/warehouse/amr/base_iw_hub.usd"
iwhub_usd_path = assets_root_path + "/Isaac/Samples/ROS2/Robots/iw_hub_ROS.usd"

if my_env_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()  # 바닥 추가

add_reference_to_stage(usd_path= my_env_path, prim_path="/World/my_warehouse")
add_reference_to_stage(usd_path= conveyer_usd_path, prim_path="/World/my_conveyer")
add_reference_to_stage(usd_path= iwhub_usd_path, prim_path="/World/my_iw_hub")

# 로봇 객체 생성 및 초기 위치 설정
my_robot = Articulation("/World/my_iw_hub", name="my_iw_hub_robot")
my_robot.set_world_poses(
    positions=np.array([[1.71931, 1.47697, 0.1]]), 
    orientations=np.array([[0.0, 0.0, 0.0, 1.0]]) # 기본 회전값 (w, x, y, z) -> 집는 로봇 기준 왼쪽이 시작 포인트
)

# 시뮬레이션 시작 전 월드 초기화
world.reset()

print("IW.Hub 로봇 로드 완료!.")

# 시뮬
while simulation_app.is_running():
    
    world.step(render=True)
    
    # 여기서 ROS 2 메시지를 기다리거나 로봇 제어 로직을 넣을 수 있습니다.


simulation_app.close()