# ============================
# 시뮬레이션 시작
# ============================
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ============================
# 기본 import
# ============================
from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World
from omni.isaac.core.prims import RigidPrim, XFormPrim
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import SurfaceGripper
from omni.isaac.core.objects import VisualSphere  # 시각화용 구체
import numpy as np

# ============================
# 월드 생성 및 초기화
# ============================
world = World(stage_units_in_meters=1.0)
world.scene.clear()

# ============================
# 배경 USD 로드
# ============================
background_usd = "/home/rokey/conveyer_test.usd"
add_reference_to_stage(usd_path=background_usd, prim_path="/World/Background")
world.scene.add_default_ground_plane()

# ============================
# 로봇 및 그리퍼 설정
# ============================
EE_LINK_PATH = "/World/Background/ur10_conveyor/ur10_long_suction_01/ee_link"
SURFACE_GRIPPER_PATH = "/World/Background/ur10_conveyor/ur10_long_suction_01/ee_link/SurfaceGripper"
ROBOT_ROOT_PATH = "/World/Background/ur10_conveyor/ur10_long_suction_01"

gripper = SurfaceGripper(
    end_effector_prim_path=EE_LINK_PATH,
    surface_gripper_path=SURFACE_GRIPPER_PATH,
)

ur10 = world.scene.add(
    SingleManipulator(
        prim_path=ROBOT_ROOT_PATH,
        name="ur10_long_suction_01",
        end_effector_prim_path=EE_LINK_PATH,
        gripper=gripper,
    )
)

# 초기 자세 (위로 쫙 펴진 느낌)
ur10.set_joints_default_state(
    positions=np.array([0, -np.pi / 2, 0, -np.pi / 2, 0, -np.pi / 2])
)

# ============================
# 박스 생성
# ============================
cube = world.scene.add(
    RigidPrim(
        prim_path="/World/Background/box",
        name="box",
        # 필요하면 position, scale 지정 가능
    )
)

box = world.scene.get_object("box")
robot = world.scene.get_object("ur10_long_suction_01")

# ============================
# Pick & Place 컨트롤러 생성
# ============================
my_controller = PickPlaceController(
    name="pick_place_controller",
    gripper=robot.gripper,
    robot_articulation=robot,
)

# ============================
# suction_cup를 XFormPrim으로 래핑 (TCP 위치용)
# ============================
suction_cup = XFormPrim(
    prim_path="/World/Background/ur10_conveyor/ur10_long_suction_01/ee_link/suction_cup")

# ============================
# 디버깅용 시각화 구체 추가
# ============================
# 빨간 공: controller가 목표로 하는 위치 (target)
target_visual = world.scene.add(
    VisualSphere(
        prim_path="/World/visual_target",
        name="visual_target",
        radius=0.02,
        color=np.array([1.0, 0.0, 0.0]),
    )
)

# 파란 공: 실제 TCP (suction_cup)의 world pose
ee_visual = world.scene.add(
    VisualSphere(
        prim_path="/World/visual_ee",
        name="visual_ee",
        radius=0.02,
        color=np.array([0.0, 0.0, 1.0]),
    )
)

# ============================
# 초기화
# ============================
world.reset()
world.step(render=True)

# ============================
# 픽 / 플레이스 설정
# ============================
placing = np.array([-0.35668085634140223, -0.16320017128181238, 1.0])

# 컨트롤러 offset은 일단 0으로 → TCP를 바로 target에 맞추게
_end_effector_offset = np.array([0.0, 0.0, 0.18])

articulation_controller = robot.get_articulation_controller()

# 박스 높이는 대략 값 (필요하면 실제 값으로 수정)
BOX_HEIGHT = 0.3  # 10cm라 가정
HALF_H = BOX_HEIGHT / 2.0

# ============================
# 상태 머신
# ============================
# 0: 박스 위로 접근
# 1: 박스 윗면 바로 위까지 내려가기
# 2: 그리퍼 닫기
# 3: 박스 든 채 위로 들어올리기
# 4: place 위치 위로 이동
# 5: place 위치로 내려가기
# 6: 박스 놓기 (그리퍼 열기)
# 7: 다시 위로 올라가기 → 끝
task_phase = 0
current_target_joint_positions = None

def joints_close_enough(current, target, tol=0.01):
    if current is None or target is None:
        return False
    current = np.array(current)
    target = np.array(target)
    return np.all(np.abs(current - target) < tol)

# ============================
# 메인 루프
# ============================
while simulation_app.is_running():
    world.step(render=True)
    if not world.is_playing():
        continue

    # 박스, 조인트 상태
    box_position, _ = box.get_world_pose()
    current_joints = robot.get_joint_positions()

    # TCP(world) = suction_cup의 world pose
    ee_position, _ = suction_cup.get_world_pose()
    ee_visual.set_world_pose(position=ee_position)

    # ----- 타겟 포인트 (월드 좌표) 계산 -----
    # 박스 윗면 기준
    approach_pos = box_position.copy()
    approach_pos[2] = box_position[2] + HALF_H + 0.20   # 윗면보다 20cm 위

    pick_pos = box_position.copy()
    pick_pos[2] = box_position[2] + HALF_H + 0.005     # 윗면보다 5mm 위

    place_approach = placing.copy()
    place_approach[2] += 0.20  # place 위치 위 20cm

    place_down = placing.copy()  # 실제 내려놓는 위치

    # ----- 현재 phase 기준 빨간 공 위치 -----
    if task_phase in [0, 3]:
        current_target = approach_pos
    elif task_phase == 1:
        current_target = pick_pos
    elif task_phase in [4, 5, 7]:
        current_target = place_approach if task_phase in [4, 7] else place_down
    else:
        current_target = placing

    target_visual.set_world_pose(position=current_target)

    # ============================
    # 상태별 동작
    # ============================
    # 0: 박스 위로 접근
    if task_phase == 0:
        actions = my_controller.forward(
            picking_position=approach_pos,
            placing_position=placing,
            current_joint_positions=current_joints,
            end_effector_offset=_end_effector_offset,
        )
        current_target_joint_positions = actions.joint_positions
        articulation_controller.apply_action(actions)

        if joints_close_enough(current_joints, current_target_joint_positions, tol=0.01):
            print("[Phase 0] 박스 위 접근 완료")
            my_controller.reset()
            task_phase = 1

    # 1: 박스 윗면 바로 위까지 내려가기
    elif task_phase == 1:
        actions = my_controller.forward(
            picking_position=pick_pos,
            placing_position=placing,
            current_joint_positions=current_joints,
            end_effector_offset=_end_effector_offset,
        )
        current_target_joint_positions = actions.joint_positions
        articulation_controller.apply_action(actions)

        # 컨트롤러의 내부 FSM이 끝났다고 판단하면 다음 단계
        if my_controller.is_done() or joints_close_enough(current_joints, current_target_joint_positions, tol=0.01):
            print("[Phase 1] 픽 위치 도달")
            my_controller.reset()
            task_phase = 2

    # 2: 그리퍼 닫기
    elif task_phase == 2:
        print("[Phase 2] 그리퍼 닫기")
        robot.gripper.close()
        task_phase = 3

    # 3: 박스 든 채 위로 들어올리기
    elif task_phase == 3:
        actions = my_controller.forward(
            picking_position=approach_pos,   # 다시 위쪽 포인트
            placing_position=placing,
            current_joint_positions=current_joints,
            end_effector_offset=_end_effector_offset,
        )
        current_target_joint_positions = actions.joint_positions
        articulation_controller.apply_action(actions)

        if joints_close_enough(current_joints, current_target_joint_positions, tol=0.01):
            print("[Phase 3] 박스 들어올림 완료")
            my_controller.reset()
            task_phase = 4

    # 4: place 위치 위로 이동
    elif task_phase == 4:
        actions = my_controller.forward(
            picking_position=place_approach,  # place 위 20cm
            placing_position=placing,
            current_joint_positions=current_joints,
            end_effector_offset=_end_effector_offset,
        )
        current_target_joint_positions = actions.joint_positions
        articulation_controller.apply_action(actions)

        if joints_close_enough(current_joints, current_target_joint_positions, tol=0.01):
            print("[Phase 4] place 상부 위치 도달")
            my_controller.reset()
            task_phase = 5

    # 5: place 위치로 내려가기
    elif task_phase == 5:
        actions = my_controller.forward(
            picking_position=place_down,   # 실제 place 위치
            placing_position=placing,
            current_joint_positions=current_joints,
            end_effector_offset=_end_effector_offset,
        )
        current_target_joint_positions = actions.joint_positions
        articulation_controller.apply_action(actions)

        if joints_close_enough(current_joints, current_target_joint_positions, tol=0.01):
            print("[Phase 5] place 위치 도달")
            my_controller.reset()
            task_phase = 6

    # 6: 박스 놓기 (그리퍼 열기)
    elif task_phase == 6:
        print("[Phase 6] 그리퍼 열기")
        robot.gripper.open()
        task_phase = 7

    # 7: 다시 위로 올라가기
    elif task_phase == 7:
        actions = my_controller.forward(
            picking_position=place_approach,  # 다시 위로
            placing_position=placing,
            current_joint_positions=current_joints,
            end_effector_offset=_end_effector_offset,
        )
        current_target_joint_positions = actions.joint_positions
        articulation_controller.apply_action(actions)

        if joints_close_enough(current_joints, current_target_joint_positions, tol=0.01):
            print("[Phase 7] 시퀀스 전체 종료")
            task_phase = 8

    # 8: 아무것도 안 함 (대기)
    elif task_phase == 8:
        pass

# 시뮬레이션 종료
simulation_app.close()