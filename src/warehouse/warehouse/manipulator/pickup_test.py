# 시뮬레이션 시작
from isaacsim import SimulationApp

# GUI가 열리는 모드({"headless": False})로 시뮬레이션 앱 시작
simulation_app = SimulationApp({"headless": False})


# usd 월드 파일 불러오기

from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units

import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere
import numpy as np
from scipy.spatial.transform import Rotation as R

world = World(stage_units_in_meters=1.0)
world.scene.clear()


#bring usd file

background_usd = "/home/rokey/conveyer_test.usd"

add_reference_to_stage(usd_path=background_usd, prim_path="/World/Background")

# add ground
world.scene.add_default_ground_plane()

robots = world.scene.get_object("ur10_conveyor")
print(robots)



# robot 씬 추가

from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper

add_reference_to_stage(usd_path=background_usd, prim_path="/World/Background")

import omni.isaac.core.utils.prims as prim_utils

 
# USD 안의 특정 prim 경로를 RigidPrim으로 래핑
gripper = SurfaceGripper(
            end_effector_prim_path="/World/Background/ur10_conveyor/ur10_long_suction_01/ee_link", 
            surface_gripper_path="/World/Background/ur10_conveyor/ur10_long_suction_01/ee_link/SurfaceGripper"
        )


# SurfaceGripper prim을 ee_link 아래에 생성
# prim_utils.create_prim(
#     prim_path="/World/Background/ur10_conveyor/ur10_long_suction_01/ee_link/SurfaceGripper",
#     prim_type="SurfaceGripper"
# )


ur10 = world.scene.add(
            SingleManipulator(
                prim_path="/World/Background/ur10_conveyor/ur10_long_suction_01",
                name="ur10_long_suction_01", end_effector_prim_path="/World/Background/ur10_conveyor/ur10_long_suction_01/ee_link", gripper=gripper
            )
        )

ur10.set_joints_default_state(positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]))

# box 씬 추가
from omni.isaac.core.prims import RigidPrim

cube = world.scene.add(
    RigidPrim(
        prim_path="/World/Background/box",
        name="box"
    )
)


# 물체 불러오기ur10_long_suction_01
box = world.scene.get_object("box")

robot = world.scene.get_object("ur10_long_suction_01")
print(robot) 

# 컨트롤러 생성

my_controller = PickPlaceController(
            name="pick_place_controller", 
            gripper=robot.gripper, 
            robot_articulation=robot
        )

# ★ 반드시 reset 후 step 한 번 해서 초기화
world.reset()
world.step(render=True)  # 이 시점에 articulation 초기화 완료

# 액션 생성

placing = np.array([-0.35668085634140223, -0.16320017128181238, 1])
_end_effector_offset = np.array([0, 0, 0.02])
box_position, box_orientation = box.get_world_pose()
print("box position: ", box_position)
actions = my_controller.forward(
            picking_position=box_position,
            placing_position=placing,
            current_joint_positions=robot.get_joint_positions(),
            end_effector_offset=_end_effector_offset,
        )

articulation_controller = robot.get_articulation_controller()

reset_needed = False

while simulation_app.is_running():
    world.step(render=True)

    if world.is_playing():
        if reset_needed:
            world.reset()
            reset_needed = False
            my_controller.reset()

        if world.current_time_step_index == 0:
            my_controller.reset()

        # 매 프레임마다 박스/로봇 상태 읽기
        box_position, box_orientation = box.get_world_pose()
        box_position[2] += 1
        current_joints = robot.get_joint_positions()

        # 매 프레임마다 controller.forward 호출
        actions = my_controller.forward(
            picking_position=box_position,
            placing_position=placing,
            current_joint_positions=current_joints,
            end_effector_offset=_end_effector_offset,
        )

        articulation_controller.apply_action(actions)

        # 다 끝났는지 체크 (한 번만 실행하고 싶으면)
        if my_controller.is_done():
            print("done picking and placing")
            reset_needed = True # or break
'''
# 외부 루프 (총 4 사이클 실행)
for i in range(3):
    print("running cycle: ", i)
    if i == 0:
        my_controller.reset()
    
    # 사이클 1, 3: "moving"
    if i == 1:
        print("moving")
        articulation_controller.apply_action(actions)
        
    
    # 사이클 2: "stopping"
    if i == 2:
        print("stopping")
        
    if my_controller.is_done():
                carb.log_info("Pick and Place 완료")
                my_controller.reset()
    
    # 내부 루프 (각 사이클마다 100번의 시뮬레이션 스텝 실행)
    for j in range(1000):
        # 시뮬레이션 1스텝 진행 (물리 + 렌더링)
        world.step(render=True)
'''